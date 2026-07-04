/*
 * sdk_serial.h - Serial port transport for MAVLink.
 *
 * No parsing here, just bytes in and bytes out - but "just bytes out"
 * is handled properly: partial writes are retried until complete,
 * transient EAGAIN (the port's output buffer is momentarily full,
 * expected on a non-blocking fd at high setpoint-stream rates) is
 * retried with a short backoff, and persistent failures are neither
 * silently dropped nor allowed to spin forever - they're logged,
 * counted, and surfaced through sdk_set_error()/sdk_set_state() so
 * the rest of the SDK (and the flight recorder) can see them.
 */
#ifndef SDK_SERIAL_H
#define SDK_SERIAL_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_error.h"
#include "sdk_fsm.h"

static inline speed_t sdk_baud_to_speed(int baud)
{
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        default:      return B115200;
    }
}

/* sdk_serial_open()
 *   Opens and configures the port as raw 8N1, no flow control.
 *   Returns the fd, or -1 on failure (errno set). */
static inline int sdk_serial_open(const char *port, int baud)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        SDK_LOG("serial: failed to open %s: %s", port, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        SDK_LOG("serial: tcgetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    speed_t speed = sdk_baud_to_speed(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; /* 100ms read timeout */

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        SDK_LOG("serial: tcsetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static inline void sdk_serial_close(int fd)
{
    if (fd >= 0) close(fd);
}

/* sdk_serial_write_all()
 *   Writes exactly len bytes, looping over partial writes and
 *   retrying transient EAGAIN/EWOULDBLOCK/EINTR with a short backoff.
 *   Returns false on a real transport error (EIO, ENODEV, ...) or if
 *   the retry budget is exhausted while still blocked. */
static inline bool sdk_serial_write_all(int fd, const uint8_t *buf, uint16_t len)
{
    uint16_t written_total = 0;
    int retries = 0;

    while (written_total < len) {
        ssize_t n = write(fd, buf + written_total, (size_t)(len - written_total));
        if (n > 0) {
            written_total += (uint16_t)n;
            retries = 0;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            if (++retries > SDK_SERIAL_WRITE_RETRY_LIMIT) {
                SDK_LOG("serial: write retry budget exhausted (port backed up)");
                return false;
            }
            struct timespec backoff = { 0, (long)SDK_SERIAL_WRITE_RETRY_DELAY_US * 1000L };
            nanosleep(&backoff, NULL);
            continue;
        }
        /* n == 0 with nothing written, or a real error (EIO, ENODEV, EBADF, ...) */
        SDK_LOG("serial: write failed: %s", (n < 0) ? strerror(errno) : "zero-length write");
        return false;
    }
    return true;
}

/* g_serial_consecutive_failures
 *   Tracks back-to-back sdk_serial_send() failures. A handful of
 *   isolated failures (a momentarily saturated USB-serial buffer) is
 *   normal and not reported; SDK_SERIAL_MAX_CONSECUTIVE_FAILURES in a
 *   row means the link is actually down, at which point this stops
 *   being silent. */
static pthread_mutex_t g_serial_fail_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_serial_consecutive_failures = 0;

static inline void sdk_serial_note_result(bool ok)
{
    pthread_mutex_lock(&g_serial_fail_lock);
    if (ok) {
        g_serial_consecutive_failures = 0;
    } else {
        g_serial_consecutive_failures++;
        int failures = g_serial_consecutive_failures;
        pthread_mutex_unlock(&g_serial_fail_lock);

        if (failures == SDK_SERIAL_MAX_CONSECUTIVE_FAILURES) {
            SDK_LOG("serial: %d consecutive write failures - link appears down", failures);
            sdk_set_error(SDK_CONNECTION_LOST);
            sdk_set_state(SDK_EMERGENCY);
        }
        return;
    }
    pthread_mutex_unlock(&g_serial_fail_lock);
}

/* sdk_serial_send()
 *   Thread-safe write of a fully-encoded MAVLink message. Safe to call
 *   from any thread (heartbeat, offboard stream, and mission calls all
 *   write concurrently). Failures are never silently dropped - see
 *   sdk_serial_note_result() above. */
static inline void sdk_serial_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

    pthread_mutex_lock(&g_serial_lock);
    bool ok = (g_fd >= 0) && sdk_serial_write_all(g_fd, buf, len);
    pthread_mutex_unlock(&g_serial_lock);

    sdk_serial_note_result(ok);
}

#endif /* SDK_SERIAL_H */
