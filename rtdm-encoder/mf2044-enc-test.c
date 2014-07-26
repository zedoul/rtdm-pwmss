#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtdm/rtdm.h>
#include <unistd.h>
#include "mf2044-enc-lib.h"
#include <assert.h>

int main(int argc, char** argv)
{
	int param_opt;
	uint64_t period = 0;
	int32_t position = -1;
	double velocity = 0.0;
	opterr = 0;

	MF2044_ENC_PINS pin = MF2044_ENC_PIN_NULL;
	MF2044_ENC_MODES mode = MF2044_ENC_MODE_NULL;

	while (-1 != (param_opt = getopt(argc, argv, "f:od:op:o")))
	{
		switch (param_opt)
		{
			case 'p': //pin
				pin = atoi(optarg);
				break;
			case 'i': //interval
				period = atoi(optarg);
				break;
			default:
				assert(0);
		}
	}

	switch(pin)
	{
		case 0:
			pin = MF2044_ENC_0;
			break;
		case 1:
			pin = MF2044_ENC_1;
			break;
		case 2:
			pin = MF2044_ENC_2;
			break;
		default:
			fprintf(stderr,"pinnum should be from 0 to 2.\n");
			return -1;
	}

	mf2044_enc_open();

	// mode test
	mf2044_enc_mode_set(pin,MF2044_ENC_MODE_ABSOLUTE);
	mode = mf2044_enc_mode_set(pin,MF2044_ENC_MODE_ABSOLUTE);
	printf("[eQEP] Mode :[%d]\n", mode);

	// period
	period = 100000000L;
	mf2044_enc_period_set(pin,period);
	printf ("[eQEP] Period = %llu ns\n",period);

	// position and velocity
	while (1) {
		position = mf2044_enc_position_get(pin);
		printf ("[eQEP] Position = [%ld]\n",position);
//		velocity = mf2044_enc_velocity_get(pin);
//		printf ("[eQEP] Velocity = [%f]\n",velocity);
		sleep(1);
	}

	return 0;
}
