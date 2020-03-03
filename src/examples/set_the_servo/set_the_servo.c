#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
// #include <px4_platform_common/cli.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
// #include <sys/ioctl.h>
// #include <sys/stat.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include <arch/board/board.h>

#include "systemlib/err.h"
#include <parameters/param.h>
#include "drivers/drv_pwm_output.h"


__EXPORT int set_the_servo(int argc, char *argv[]);

int set_the_servo_main(int argc, char *argv[])
{
	PX4_INFO("Hello I will set the servo for you!");



	while (1) {
		const char *dev = PWM_OUTPUT0_DEVICE_PATH;

		int fd = px4_open(dev, 0);

		if (fd < 0) {
			PX4_ERR("cannot open path to output");
		}

		/* get the number of servo channels */

		unsigned servo_count;
		int ret;
		int rv=1;


		ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
		PX4_INFO("servo count: %d", servo_count);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_COUNT");
			return 1;
		}

		// setup pwm port params


		// int mode_ret = px4_ioctl(fd, PWM_SERVO_SET(5), 1500);
		// int pitch_ret = px4_ioctl(fd, PWM_SERVO_SET(6), 1500);

		// if (mode_ret != OK) {
		// 	PX4_ERR("cannot set servo");
		// 	// PX4_ERR("cannot set servo");
		// 	return 1;
		// }
		// if (pitch_ret != OK) {
		// 	PX4_ERR("cannot set servo");
		// 	return 1;
		// }
		/* get current servo values */
		struct pwm_output_values last_spos;

		for (unsigned i = 0; i < servo_count; i++) {


			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_GET(%d)", i);
				return 1;
			}
		}

		struct pollfd fds;

		fds.fd = 0; /* stdin */

		fds.events = POLLIN;

		if (px4_ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
			PX4_ERR("Failed to Enter pwm test mode");
			goto err_out_no_test;
		}

		PX4_INFO("Press CTRL-C or 'c' to abort.");

		// for (unsigned i = 0; i < 8; i++) {


		// 	ret = px4_ioctl(fd, PWM_SERVO_GET(i), 1500);

		// 	if (ret != OK) {
		// 		PX4_ERR("PWM_SERVO_GET(%d)", i);
		// 		return 1;
		// 	}
		// }

		while (1) {
			for (unsigned i = 0; i < servo_count; i++) {
				ret = px4_ioctl(fd, PWM_SERVO_SET(i), 1500);

				if (ret != OK) {
					PX4_ERR("PWM_SERVO_SET(%d)", i);
					goto err_out;
				}

			}

			/* abort on user request */
			char c;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				ret = read(0, &c, 1);

				if (c == 0x03 || c == 0x63 || c == 'q') {
					/* reset output to the last value */
					for (unsigned i = 0; i < servo_count; i++) {
						ret = px4_ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);

						if (ret != OK) {
							PX4_ERR("PWM_SERVO_SET(%d)", i);
							goto err_out;
						}
					}

					PX4_INFO("User abort\n");
					rv = 0;
					goto err_out;
				}
			}

			/* Delay longer than the max Oneshot duration */

			px4_usleep(2542);

#ifdef __PX4_NUTTX
			/* Trigger all timer's channels in Oneshot mode to fire
			 * the oneshots with updated values.
			 */

			up_pwm_update();
#endif
		}

		rv = 0;
err_out:

		if (px4_ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
			rv = 1;
			PX4_ERR("Failed to Exit pwm test mode");
		}

err_out_no_test:
		return rv;





	}



	return 0;

}

