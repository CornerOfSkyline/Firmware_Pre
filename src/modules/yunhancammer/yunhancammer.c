/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <poll.h>
#include <uORB/topics/vehicle_command.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static int serial_fd;

/**
 * daemon management function.
 */
__EXPORT int yunhancammer_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int yunhancammer_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

void yunhan_handle_command(struct vehicle_command_s *cmd);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: yunhancammer {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int yunhancammer_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("yunhancammer already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("yunhancammer",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 yunhancammer_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int yunhancammer_thread_main(int argc, char *argv[])
{

	if (argc < 2) {
		errx(1, "need a serial port name as argument");
	}

	const char *uart_name = argv[1];

	warnx("opening port %s", uart_name);

	serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	unsigned speed = 19200;

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(serial_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			close(serial_fd);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		close(serial_fd);
		return -1;
	}

	thread_running = true;

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	while (!thread_should_exit) {

		/*This runs at the rate of the sensors */
		struct pollfd fds[] = {
			{ .fd = cmd_sub, .events = POLLIN }
		};

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 500);

		if (ret < 0) {
			/* poll error, ignore */
			warnx("poll error\n");

		} else if (ret == 0) {
			/* no return value, ignore */
			//warnx("no commander data\n");

		} else {

			//warnx("get commander data\n");

			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle it */
			yunhan_handle_command(&cmd) ;
		}		
	}

	warnx("exiting");
	thread_running = false;

	fflush(stdout);
	return 0;
}

void yunhan_handle_command(struct vehicle_command_s *cmd)
{
	char buf[7];
	switch (cmd->command) {
		case 203:
			if((int)(cmd->param3 + 0.5f) == 1){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x00;
				buf[3] = 0x40;
				buf[4] = 0x00;
				buf[5] = 0x00;
				buf[6] = 0x41;
				write(serial_fd, buf, 7);
			}
			else if((int)(cmd->param3 + 0.5f) == 2){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x00;
				buf[3] = 0x20;
				buf[4] = 0x00;
				buf[5] = 0x00;
				buf[6] = 0x21;
				write(serial_fd, buf, 7);
			}
			break;
		case 205:
			if((int)(cmd->param1 + 0.5f) == 1){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x00;
				buf[3] = 0x08;
				buf[4] = 0x00;
				buf[5] = 0xff;
				buf[6] = 0x08;
				write(serial_fd, buf, 7);
			}
			else if((int)(cmd->param1 + 0.5f) == 2){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x00;
				buf[3] = 0x10;
				buf[4] = 0x00;
				buf[5] = 0xff;
				buf[6] = 0x10;
				write(serial_fd, buf, 7);
			}

			if((int)(cmd->param3 + 0.5f) == 1){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x00;
				buf[3] = 0x04;
				buf[4] = 0xff;
				buf[5] = 0x00;
				buf[6] = 0x04;
				write(serial_fd, buf, 7);
			}
			else if((int)(cmd->param3 + 0.5f) == 2){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x00;
				buf[3] = 0x02;
				buf[4] = 0xff;
				buf[5] = 0x00;
				buf[6] = 0x02;
				write(serial_fd, buf, 7);
			}

			if((int)(cmd->param7 + 0.5f) == 1){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x01;
				buf[3] = 0x01;
				buf[4] = 0x00;
				buf[5] = 0x00;
				buf[6] = 0x03;
				write(serial_fd, buf, 7);
			}
			else if((int)(cmd->param7 + 0.5f) == 2){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x01;
				buf[3] = 0x01;
				buf[4] = 0x01;
				buf[5] = 0x00;
				buf[6] = 0x04;
				write(serial_fd, buf, 7);
			}
			else if((int)(cmd->param7 + 0.5f) == 3){
				buf[0] = 0xff;
				buf[1] = 0x01;
				buf[2] = 0x01;
				buf[3] = 0x01;
				buf[4] = 0x02;
				buf[5] = 0x00;
				buf[6] = 0x05;
				write(serial_fd, buf, 7);
			}
			break;
		default:
			break;
	}
}
