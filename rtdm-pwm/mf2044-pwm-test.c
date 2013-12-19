#include <stdio.h>
#include <string.h>
#include <rtdm/rtdm.h>
#include <unistd.h>
#include "mf2044-pwm-lib.h"
#include <assert.h>

int main(int argc, char** argv)
{
	int param_opt;
	MF2044_PWM_PINS pin=1;
	int freq = -1;
	int duty = -1;
	opterr = 0;
	while (-1 != (param_opt = getopt(argc, argv, "f:od:op:o")))
	{
		switch (param_opt)
		{
			case 'p': // pin
				pin = atoi(optarg);
				break;
			case 'f': // frequency
				freq = atoi(optarg);
				break;
			case 'd': // duty cycle
				duty = atoi(optarg);
				break;
			default:
				fprintf(stderr,"usage: mf2044-pwm-test [-p <pinnum>] [-f <frequency ns>] [-d <ducy percent>]\n");
				return -1;
		}
	}

	switch(pin)
	{
		case 1:
			pin = MF2044_PWM_P9_14;
			break;
		case 2:
			pin = MF2044_PWM_P9_16;
			break;
		case 3:
			pin = MF2044_PWM_P9_22;
			break;
		case 4:
			pin = MF2044_PWM_P9_21;
			break;
		case 5: 
			pin = MF2044_PWM_P8_19;
			break;
		case 6:
			pin = MF2044_PWM_P8_13;
			break;
		default:
			fprintf(stderr,"pinnum should be from 1 to 6.\n");
			return -1;
	}

	mf2044_pwm_open();

	if (-1 != freq) {
		mf2044_pwm_frequency_set(pin,freq);
	}
	if (-1 != duty) {
		mf2044_pwm_duty_cycle_set(pin,duty);
	}
	freq = mf2044_pwm_frequency_get(pin);
	duty = mf2044_pwm_duty_cycle_get(pin);

	printf("current frequency %d (nanosecond)\n", freq);
	printf("current duty %d (percent)\n", duty);

	return 0;
}

