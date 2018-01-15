/**
 * A small and simple replacement for minicom/cutecom/etc.
 * Useful for communicating with a Parrot target.
 *
 * Author: ivan.djelic@parrot.com
 *
 * Copyright (C) 2013 Parrot S.A.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/signalfd.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <string.h>

#define PFX                    "parrotcom: "
#define DEFAULT_DEVICE         "/dev/ttyUSB0"

#define FATAL(_fmt, args...)				\
	do {						\
		fprintf(stderr, PFX _fmt, ##args);	\
		terminal_cleanup();			\
		exit(EXIT_FAILURE);			\
	} while (0)

#define INFO(_fmt, args...) fprintf(stderr, PFX _fmt, ##args)

static struct pollfd fds[3];
static struct termios stdin_options;
static struct termios uart_options;
static int stdin_options_valid;
static int uart_options_valid;

static speed_t get_baudrate(const char *speed)
{
	speed_t br = B115200;
	const char *br_table[] = {
		[B9600]   = "9600",
		[B38400]  = "38400",
		[B57600]  = "57600",
		[B115200] = "115200",
	};

	if (!speed)
		goto exit;

	if (!strcmp(speed, "115200")) {
		br = B115200;
	} else if (!strcmp(speed, "57600")) {
		br = B57600;
	} else if (!strcmp(speed, "38400")) {
		br = B38400;
	} else if (!strcmp(speed, "9600")) {
		br = B9600;
	} else {
		br = B115200;
	}

exit:
        INFO("setting baud rate to: %s\n", br_table[br]);
	return br;
}

static void terminal_cleanup(void)
{
	/* restore terminal settings and exit */
	if (stdin_options_valid)
		tcsetattr(STDIN_FILENO, TCSANOW, &stdin_options);
	if (uart_options_valid)
		tcsetattr(fds[1].fd, TCSANOW, &uart_options);
}

static void set_stdin_mode(int rom_mode)
{
	struct termios options;

	if (tcgetattr(STDIN_FILENO, &options) < 0)
		FATAL("tcgetattr: %m\n");

	/* enable (ROM) / disable (Linux) local echo and canonical mode */
	if (rom_mode)
		options.c_lflag |= ECHO|ICANON;
	else
		options.c_lflag &= ~(ECHO|ICANON);

	if (tcsetattr(STDIN_FILENO, TCSANOW, &options) != 0)
		FATAL("tcsetattr on stdin: %m\n");

	INFO("switched to %s mode\n", rom_mode ? "ROM" : "Linux");
}

static int open_uart(const char *dev, speed_t speed)
{
	int fd;
	struct termios options;

	fd = open(dev, O_RDWR|O_NONBLOCK);
	if (fd < 0)
		FATAL("cannot open '%s': %m\n", dev);

	if ((tcgetattr(fd, &uart_options) < 0) ||
	    (tcgetattr(fd, &options) < 0))
		FATAL("tcgetattr on '%s': %m\n", dev);

	uart_options_valid = 1;

	/* 115200 bauds */
	cfsetispeed(&options, speed);
	cfsetospeed(&options, speed);
	/* raw mode */
	cfmakeraw(&options);
	/* 1 stop bit (8N1) */
	options.c_cflag &= ~CSTOPB;
	/* no hardware flow control */
	options.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &options) != 0)
		FATAL("tcsetattr on '%s': %m\n", dev);

	return fd;
}

static int open_sig(void)
{
	int fd;
	sigset_t mask;

	sigemptyset(&mask);
	sigaddset(&mask, SIGINT);
	sigaddset(&mask, SIGQUIT);
	sigaddset(&mask, SIGTERM);
	sigprocmask(SIG_BLOCK, &mask, NULL);

	fd = signalfd(-1, &mask, SFD_NONBLOCK|SFD_CLOEXEC);
	if (fd < 0)
		FATAL("signalfd: %m\n");

	return fd;
}

static int open_stdin(void)
{
	struct termios options;

	/* make stdin non-blocking */
	fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL)|O_NONBLOCK);

	/* input terminal configuration */
	if ((tcgetattr(STDIN_FILENO, &stdin_options) < 0) ||
	    (tcgetattr(STDIN_FILENO, &options) < 0))
		FATAL("tcgetattr on stdin: %m\n");

	stdin_options_valid = 1;

	/* remap INT signal on ^A */
	options.c_cc[VINTR] = 'a' & 0x1f;
	/* remap TSTP signal on ^B */
	options.c_cc[VSUSP] = 'b' & 0x1f;

	if (tcsetattr(STDIN_FILENO, TCSANOW, &options) != 0)
		FATAL("tcsetattr on stdin: %m\n");

	return STDIN_FILENO;
}

static uint64_t clock_us(void)
{
	struct timeval tv;
	(void)gettimeofday(&tv, NULL);
	return tv.tv_sec*1000000ULL + tv.tv_usec;
}

static void write_uart(int outfd, char *buf, ssize_t count)
{
	char *p;
	ssize_t wcount;
	uint64_t deadline;

	/* set a 1s timeout; 4096 byte @ 115200 baud = 400 ms approx. */
	deadline = clock_us() + 1000000ULL;

	p = buf;
	while (count > 0) {
		wcount = write(outfd, p, count);
		if (wcount <= 0) {
			if ((errno == EINTR) || (errno == EAGAIN))
				wcount = 0;
			else
				FATAL("write to uart: %m\n");
			/* retry later */
			usleep(10000);
		}
		count -= wcount;
		p += wcount;

		/* do not stay stuck forever */
		if (clock_us() >= deadline)
			break;
	}
}

static void send_file(int outfd, const char *filename)
{
	ssize_t count;
	int fd;
	char buf[4096];

	INFO("sending the contents of file '%s'\n", filename);

	fd = open(filename, O_RDONLY);
	if (fd < 0)
		FATAL("open(%s): %m\n", filename);

	while (1) {
		count = read(fd, buf, sizeof(buf));
		if (count > 0) {
			write_uart(outfd, buf, count);
		} else {
			if (count < 0)
				FATAL("read(%s): %m\n", filename);
			break;
		}
	}
	close(fd);
}

static void write_timestamped(char *buf, ssize_t rcount, int diff)
{
	char *p;
	ssize_t len;
	uint64_t now, s, us;
	static uint64_t last;
	static int newline = 1;
	char stamp[128];

	while (rcount > 0) {
		if (newline) {
			/* we are starting a new line */
			now = clock_us();
			if (!last)
				last = now;
			us = now - last;
			if (diff)
				last = now;
			s = us/1000000ULL;
			/* insert local timestamp */
			snprintf(stamp, sizeof(stamp), "[%02d:%02d:%02d.%06d] ",
				 (int)(s / 3600) % 24,
				 (int)(s / 60)   % 60,
				 (int)(s % 60),
				 (int)(us - s*1000000));
			(void)write(STDERR_FILENO, stamp, strlen(stamp));
			newline = 0;
		}
		/* split next write at '\n' location */
		p = memchr(buf, '\n', rcount);
		if (p) {
			len = p-buf+1;
			newline = 1;
		} else {
			len = rcount;
		}

		(void)write(STDERR_FILENO, buf, len);
		buf += len;
		rcount -= len;
	}
}

static void usage(void)
{
	fprintf(stderr,
		"Usage: parrotcom [-f FILE] [-l] [-r] [-h] [DEVICE]\n"
		"Connect the current terminal to a remote uart DEVICE.\n"
		"DEVICE should be a uart device path such as '/dev/ttyS0'.\n"
		"If DEVICE is not provided, it defaults to '%s'.\n"
		"Example: parrotcom /dev/ttyUSB1\n"
		"Available options:\n"
		" -f FILE  Send the contents of FILE to uart at session start\n"
		" -l       Linux mode (default, suitable for a busybox shell)\n"
		" -r       ROM mode (suitable for P5/P6/P7 ROM monitor)\n"
		" -t       Display timestamp (relative to first line) for each "
		"line\n"
		" -T       Display timestamp (diff) for each line\n"
		" -h       Show this help\n", DEFAULT_DEVICE);
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	const char *dev;
	char buf[4096];
	ssize_t rcount;
	int c, ret, rom_mode = 0;
	int timestamp_mode = 0, timestamp_diff_mode = 0;
	const char *initfile = NULL;
	struct signalfd_siginfo info;
	speed_t speed = B115200;

	while ((c = getopt(argc, argv, "b:f:hlrtT")) != -1) {
		switch (c) {
		case 'b':
			speed = get_baudrate(optarg);
			break;
		case 'f':
			initfile = optarg;
			break;
		case 'l':
			rom_mode = 0;
			break;
		case 'r':
			rom_mode = 1;
			break;
		case 't':
			timestamp_mode = 1;
			break;
		case 'T':
			timestamp_mode = 1;
			timestamp_diff_mode = 1;
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}

	if (optind >= argc) {
		dev = DEFAULT_DEVICE;
		INFO("device not provided, defaulting to '%s'\n", dev);
	} else {
		dev = argv[optind];
	}

	/* stdin */
	fds[0].fd = open_stdin();
	fds[0].events = POLLIN;

	/* serial input/output */
	fds[1].fd = open_uart(dev, speed);
	fds[1].events = POLLIN|POLLERR;

	/* signal input */
	fds[2].fd = open_sig();
	fds[2].events = POLLIN;

	set_stdin_mode(rom_mode);

	INFO("press CTRL-\\ to exit;\n");
	INFO("press CTRL-a to toggle between ROM and Linux modes\n");

	if (initfile)
		send_file(fds[1].fd, initfile);

	while (1) {
		ret = poll(fds, 3, -1);
		if ((ret < 0) && (errno != EINTR))
			break;

		if (fds[0].revents & POLLIN) {
			rcount = read(fds[0].fd, buf, (size_t)sizeof(buf));
			if (rcount > 0)
				write_uart(fds[1].fd, buf, rcount);
		}

		if (fds[1].revents & (POLLIN|POLLERR)) {
			if (fds[1].revents & POLLERR)
				FATAL("cannot access '%s', bailing out\n", dev);

			rcount = read(fds[1].fd, buf, (size_t)sizeof(buf));
			if (rcount > 0) {
				if (timestamp_mode) {
					write_timestamped(buf, rcount,
							  timestamp_diff_mode);
				} else {
					(void)write(STDERR_FILENO, buf, rcount);
				}
			}
		}

		if (fds[2].revents & POLLIN) {
			rcount = read(fds[2].fd, &info, sizeof(info));
			if (rcount > 0) {
				/* handle signal */
				if (info.ssi_signo == SIGINT) {
					/* toggle mode */
					rom_mode ^= 1;
					fprintf(stderr, "\r\n");
					set_stdin_mode(rom_mode);
				} else {
					/* bail out on QUIT or TERM */
					break;
				}
			}
		}

	}

	terminal_cleanup();
	fprintf(stderr, "\r\n");
	INFO("exiting...\r\n");

	close(fds[1].fd);
	close(fds[2].fd);
	return 0;
}
