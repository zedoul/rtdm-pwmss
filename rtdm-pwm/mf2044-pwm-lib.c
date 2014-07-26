#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtdm/rtdm.h>
#include "mf2044-pwm-lib.h"

#define DEVICE_NAME "mf2044_pwm_drv"
#define PIN_CHECKER 0x150
#define DUTY_INIT 0

#define MF2044_IOCTL_MAGIC 0x00
#define MF2044_IOCTL_ON _IO(MF2044_IOCTL_MAGIC, 1)
#define MF2044_IOCTL_OFF _IO(MF2044_IOCTL_MAGIC, 2)
#define MF2044_IOCTL_GET_DUTY_CYCLE _IO(MF2044_IOCTL_MAGIC, 3)
#define MF2044_IOCTL_SET_DUTY_CYCLE _IO(MF2044_IOCTL_MAGIC, 4)
#define MF2044_IOCTL_GET_FREQUENCY _IO(MF2044_IOCTL_MAGIC, 5)
#define MF2044_IOCTL_SET_FREQUENCY _IO(MF2044_IOCTL_MAGIC, 6)

static int fd = -1;

int mf2044_pwm_open(void)
{
	fd = rt_dev_open(DEVICE_NAME, 0);
	if (fd < 0) {
		printf("ERROR : can't open device %s (%s)\n",
		       DEVICE_NAME, strerror(-fd));
		fflush(stdout);
		return EXIT_FAILURE;
	}
}

int mf2044_pwm_close(void)
{
	rt_dev_close(fd);
	return EXIT_SUCCESS;
}

int mf2044_pwm_duty_cycle_get(MF2044_PWM_PINS pin)
{
	int duty, freq = -1;
	int command = MF2044_IOCTL_GET_DUTY_CYCLE;
	command |= pin;

	if (MF2044_PWM_PNULL>=pin || MF2044_PWM_PMAX <= pin) {
		printf("param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}

	if (rt_dev_ioctl(fd, command, &duty) == -1) {
		printf("TIOCMGET failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}

	freq = mf2044_pwm_frequency_get(pin);

	return (duty/(freq*1.0)) * 100;
}

int mf2044_pwm_duty_cycle_set(MF2044_PWM_PINS pin, int duty)
{
	int freq = mf2044_pwm_frequency_get(pin);
	int duty_cycle = freq  * (duty/100.0);
	int command = MF2044_IOCTL_SET_DUTY_CYCLE;
	command |= pin;

	if (0 > duty || 100 < duty) {
		printf("param failed: duty[%d] should be from 0 to 100.\n", duty);
		return EXIT_FAILURE;
	}
	if (MF2044_PWM_PNULL>=pin || MF2044_PWM_PMAX <= pin) {
		printf("param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}

	if (rt_dev_ioctl(fd, command, duty_cycle) == -1) {
		printf("TIOCMGET failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int mf2044_pwm_frequency_get(MF2044_PWM_PINS pin)
{
	int freq;
	int command = MF2044_IOCTL_GET_FREQUENCY;
	command |= pin;

	if (MF2044_PWM_PNULL>=pin || MF2044_PWM_PMAX <= pin) {
		printf("param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}
	if (rt_dev_ioctl(fd, command, &freq) == -1) {
		printf("TIOCMGET failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}
	return freq;
}

int mf2044_pwm_frequency_set(MF2044_PWM_PINS pin, int freq)
{
	int command = MF2044_IOCTL_SET_FREQUENCY;
	command |= pin;
	int duty1 = mf2044_pwm_duty_cycle_get(pin);
	int duty2 = DUTY_INIT;

	if (0 > freq) {
		printf("param failed: freq[%d] should be more than 0.\n", freq);
		return EXIT_FAILURE;
	}
	if (MF2044_PWM_PNULL>=pin || MF2044_PWM_PMAX <= pin) {
		printf("param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}

	// Get duty cycle in same eHRPWM
	if (pin == (PIN_CHECKER & pin)) {
		duty2 = mf2044_pwm_duty_cycle_get(pin << 1);
	} else {
		duty2 = mf2044_pwm_duty_cycle_get(pin >> 1);
	}

	// Check A and B duty cycles in same eHRPWM are already set or not
	if (duty1 != DUTY_INIT && duty2 != DUTY_INIT) {
		fprintf(stderr,"You can not change frequency after setting both duty cycle in pwmss\n");
		fprintf(stderr,"current duty cycles: A[%d] / B[%d]\n",duty1,duty2);
		return EXIT_FAILURE;
	}
	if (rt_dev_ioctl(fd, command, freq) == -1) {
		fprintf(stderr,"TIOCMGET failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

