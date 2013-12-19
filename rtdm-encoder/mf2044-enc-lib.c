#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtdm/rtdm.h>
#include "mf2044-enc-lib.h"
#include <time.h>
#include <math.h>

#define DEVICE_NAME "mf2044_enc_drv"
#define MF2044_IOCTL_MAGIC 0x00
#define MF2044_IOCTL_ON _IO(MF2044_IOCTL_MAGIC, 1)
#define MF2044_IOCTL_OFF _IO(MF2044_IOCTL_MAGIC, 2)
#define MF2044_IOCTL_GET_POSITION _IO(MF2044_IOCTL_MAGIC, 3)
#define MF2044_IOCTL_SET_POSITION _IO(MF2044_IOCTL_MAGIC, 4)
#define MF2044_IOCTL_GET_PERIOD _IO(MF2044_IOCTL_MAGIC, 5)
#define MF2044_IOCTL_SET_PERIOD _IO(MF2044_IOCTL_MAGIC, 6)
#define MF2044_IOCTL_GET_MODE _IO(MF2044_IOCTL_MAGIC, 7)
#define MF2044_IOCTL_SET_MODE _IO(MF2044_IOCTL_MAGIC, 8)

#define SEC_PER_NSEC 1e-9
#define NSEC_PER_SEC 1000000000

static int fd = -1;

int mf2044_enc_open(void)
{
	fd = rt_dev_open(DEVICE_NAME, 0);
	if (fd < 0) {
		printf("ERROR : can't open device %s (%s)\n",
		       DEVICE_NAME, strerror(-fd));
		fflush(stdout);
		return EXIT_FAILURE;
	}
}

int mf2044_enc_close(void)
{
	rt_dev_close(fd);
	return EXIT_SUCCESS;
}

MF2044_ENC_MODES mf2044_enc_mode_get(MF2044_ENC_PINS pin)
{
	int mode = MF2044_ENC_MODE_NULL;
	int command = MF2044_IOCTL_GET_MODE;

	if (MF2044_ENC_PIN_NULL>=pin || MF2044_ENC_PIN_MAX <= pin) {
		fprintf(stderr,"param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}

	command |= pin;

	if (rt_dev_ioctl(fd, command, &mode) == -1)
	{
		fprintf(stderr,"TIOCMGET failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}
	return mode;
}

int mf2044_enc_mode_set(MF2044_ENC_PINS pin, MF2044_ENC_MODES mode)
{
	unsigned int det = 0;
	int command = MF2044_IOCTL_SET_MODE;

	if (MF2044_ENC_PIN_NULL>=pin || MF2044_ENC_PIN_MAX <= pin) {
		fprintf(stderr,"param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}
	if (MF2044_ENC_MODE_NULL>=mode || MF2044_ENC_MODE_MAX <= mode) {
		fprintf(stderr,"param failed: mode[%d] is too high or too low.\n", mode);
		return EXIT_FAILURE;
	}

	command |= pin;

	if (rt_dev_ioctl(fd, command, mode) == -1)
		printf("TIOCMGET failed: %s\n", strerror(errno));
	return det;
}

uint64_t mf2044_enc_period_get(MF2044_ENC_PINS pin)
{
	uint64_t ret = -1;
	int command = MF2044_IOCTL_GET_PERIOD;

	if (MF2044_ENC_PIN_NULL>=pin || MF2044_ENC_PIN_MAX <= pin) {
		fprintf(stderr,"param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}

	command |= pin;

	if (rt_dev_ioctl(fd, command, &ret) == -1)
	{
		fprintf(stderr,"TIOCMGET failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}
	return ret;
}

int mf2044_enc_period_set(MF2044_ENC_PINS pin, uint64_t period)
{
	int command = MF2044_IOCTL_SET_PERIOD;

	if (MF2044_ENC_PIN_NULL>=pin || MF2044_ENC_PIN_MAX <= pin) {
		fprintf(stderr,"param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}

	command |= pin;

	if (rt_dev_ioctl(fd, command, period) == -1)
	{
		fprintf(stderr,"TIOCMGET failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int32_t mf2044_enc_position_get(MF2044_ENC_PINS pin)
{
	uint32_t det = -1;
	int command = MF2044_IOCTL_GET_POSITION;

	if (MF2044_ENC_PIN_NULL>=pin || MF2044_ENC_PIN_MAX <= pin) {
		fprintf(stderr,"param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}

	command |= pin;

	if (rt_dev_ioctl(fd, command, &det) == -1) {
		fprintf(stderr,"TIOCMGET failed: %s\n", strerror(errno));
		return -1;
	}
	return det;
}

int mf2044_enc_position_set(MF2044_ENC_PINS pin, int32_t position)
{
	int command = MF2044_IOCTL_SET_POSITION;

	if (MF2044_ENC_PIN_NULL>=pin || MF2044_ENC_PIN_MAX <= pin) {
		fprintf(stderr,"param failed: pin[%d] is too high or too low.\n", pin);
		return EXIT_FAILURE;
	}

	command |= pin;

	if (rt_dev_ioctl(fd, command, position) == -1)
	{
		fprintf(stderr,"TIOCMGET failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

//double mf2044_enc_velocity_get(MF2044_ENC_PINS pin)
//{
//	double ret = -1;
//	int32_t position = -1;
//	uint64_t period = mf2044_enc_period_get(pin);
//	struct timespec ts;
//	MF2044_ENC_MODES mode = mf2044_enc_mode_get(pin);
//
//	if (MF2044_ENC_PIN_NULL>=pin || MF2044_ENC_PIN_MAX <= pin) {
//		printf("param failed: pin[%d] is too high or too low.\n", pin);
//		return EXIT_FAILURE;
//	}
//
//	switch(mode)
//	{
//		case MF2044_ENC_MODE_RELATIVE:
//			position = mf2044_enc_position_get(pin);
//			break;
//		case MF2044_ENC_MODE_ABSOLUTE:
//			ts.tv_sec = period / NSEC_PER_SEC;
//			ts.tv_nsec = period % NSEC_PER_SEC;
//			position = mf2044_enc_position_get(pin);
//			nanosleep(&ts,NULL);
//			#warning TODO - I need to consider overflow. Even though QPOSMAX considers.
//			position = position - mf2044_enc_position_get(pin);
//			break;
//		default:
//			return (float) EXIT_FAILURE;
//	}
//	ret = position / (period * SEC_PER_NSEC);
//	return fabs(ret);
//}

