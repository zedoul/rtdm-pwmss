#include <rtdm/rtdm_driver.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Seonghyun Kim");

#define LSIZE_MAX		1024
#define DEVICE_NAME		"mf2044_pwm_drv"
#define SOME_SUB_CLASS		4711

#define SYSCLK 100000000

void __iomem *cm_per_map;
void __iomem *epwm1_0_map;
void __iomem *epwm1_1_map;
void __iomem *epwm0_0_map;
void __iomem *epwm0_1_map;
void __iomem *epwm2_0_map;
void __iomem *epwm2_1_map;
void __iomem *mmio_base;

#define CM_PER_BASE 0x44e00000
#define CM_PER_SZ 0x44e03fff-CM_PER_BASE
#define EPWMSS1_CLK_CTRL 0xcc
#define EPWMSS0_CLK_CTRL 0xd4
#define EPWMSS2_CLK_CTRL 0xd8

#define EPWM1_0_BASE 0x48302200
#define EPWM1_1_BASE 0x48302260
#define EPWM0_0_BASE 0x48300200
#define EPWM0_1_BASE 0x48300260
#define EPWM2_0_BASE 0x48304200
#define EPWM2_1_BASE 0x48304260

#define EPWM_SZ 0x5f

#define TBCNT 0x8
#define CMPAHR 0x10

/* EHRPWM registers and bits definitions */
/* Time base module registers */
#define TBCTL			0x00
#define TBPRD			0x0A

#define TBCTL_RUN_MASK		(BIT(15) | BIT(14))
#define TBCTL_STOP_NEXT		0
#define TBCTL_STOP_ON_CYCLE	BIT(14)
#define TBCTL_FREE_RUN		(BIT(15) | BIT(14))
#define TBCTL_PRDLD_MASK	BIT(3)
#define TBCTL_PRDLD_SHDW	0
#define TBCTL_PRDLD_IMDT	BIT(3)
#define TBCTL_CLKDIV_MASK	(BIT(12) | BIT(11) | BIT(10) | BIT(9) | \
				BIT(8) | BIT(7))
#define TBCTL_CTRMODE_MASK	(BIT(1) | BIT(0))
#define TBCTL_CTRMODE_UP	0
#define TBCTL_CTRMODE_DOWN	BIT(0)
#define TBCTL_CTRMODE_UPDOWN	BIT(1)
#define TBCTL_CTRMODE_FREEZE	(BIT(1) | BIT(0))

#define TBCTL_HSPCLKDIV_SHIFT	7
#define TBCTL_CLKDIV_SHIFT	10

#define CLKDIV_MAX		7
#define HSPCLKDIV_MAX		7
#define PERIOD_MAX		0xFFFF

/* compare module registers */
#define CMPA			0x12
#define CMPB			0x14

/* Action qualifier module registers */
#define AQCTLA			0x16
#define AQCTLB			0x18
#define AQSFRC			0x1A
#define AQCSFRC			0x1C

#define AQCTL_CBU_MASK		(BIT(9) | BIT(8))
#define AQCTL_CBU_FRCLOW	BIT(8)
#define AQCTL_CBU_FRCHIGH	BIT(9)
#define AQCTL_CBU_FRCTOGGLE	(BIT(9) | BIT(8))
#define AQCTL_CAU_MASK		(BIT(5) | BIT(4))
#define AQCTL_CAU_FRCLOW	BIT(4)
#define AQCTL_CAU_FRCHIGH	BIT(5)
#define AQCTL_CAU_FRCTOGGLE	(BIT(5) | BIT(4))
#define AQCTL_PRD_MASK		(BIT(3) | BIT(2))
#define AQCTL_PRD_FRCLOW	BIT(2)
#define AQCTL_PRD_FRCHIGH	BIT(3)
#define AQCTL_PRD_FRCTOGGLE	(BIT(3) | BIT(2))
#define AQCTL_ZRO_MASK		(BIT(1) | BIT(0))
#define AQCTL_ZRO_FRCLOW	BIT(0)
#define AQCTL_ZRO_FRCHIGH	BIT(1)
#define AQCTL_ZRO_FRCTOGGLE	(BIT(1) | BIT(0))

#define AQCTL_CHANA_POLNORMAL	(AQCTL_CAU_FRCLOW | AQCTL_PRD_FRCHIGH | \
				AQCTL_ZRO_FRCHIGH)
#define AQCTL_CHANA_POLINVERSED	(AQCTL_CAU_FRCHIGH | AQCTL_PRD_FRCLOW | \
				AQCTL_ZRO_FRCLOW)
#define AQCTL_CHANB_POLNORMAL	(AQCTL_CBU_FRCLOW | AQCTL_PRD_FRCHIGH | \
				AQCTL_ZRO_FRCHIGH)
#define AQCTL_CHANB_POLINVERSED	(AQCTL_CBU_FRCHIGH | AQCTL_PRD_FRCLOW | \
				AQCTL_ZRO_FRCLOW)

#define AQSFRC_RLDCSF_MASK	(BIT(7) | BIT(6))
#define AQSFRC_RLDCSF_ZRO	0
#define AQSFRC_RLDCSF_PRD	BIT(6)
#define AQSFRC_RLDCSF_ZROPRD	BIT(7)
#define AQSFRC_RLDCSF_IMDT	(BIT(7) | BIT(6))

#define AQCSFRC_CSFB_MASK	(BIT(3) | BIT(2))
#define AQCSFRC_CSFB_FRCDIS	0
#define AQCSFRC_CSFB_FRCLOW	BIT(2)
#define AQCSFRC_CSFB_FRCHIGH	BIT(3)
#define AQCSFRC_CSFB_DISSWFRC	(BIT(3) | BIT(2))
#define AQCSFRC_CSFA_MASK	(BIT(1) | BIT(0))
#define AQCSFRC_CSFA_FRCDIS	0
#define AQCSFRC_CSFA_FRCLOW	BIT(0)
#define AQCSFRC_CSFA_FRCHIGH	BIT(1)
#define AQCSFRC_CSFA_DISSWFRC	(BIT(1) | BIT(0))

#define NUM_PWM_CHANNEL		2	/* EHRPWM channels */

#define MF2044_PWM_1_0 (1<<4) //P9.14
#define MF2044_PWM_1_1 (1<<5) //P9.16
#define MF2044_PWM_0_0 (1<<6) //P9.22
#define MF2044_PWM_0_1 (1<<7) //P9.21
#define MF2044_PWM_2_0 (1<<8) //P8.19
#define MF2044_PWM_2_1 (1<<9) //P8.13

#define MF2044_IOCTL_MAGIC 0x00
#define MF2044_IOCTL_ON _IO(MF2044_IOCTL_MAGIC, 1)
#define MF2044_IOCTL_OFF _IO(MF2044_IOCTL_MAGIC, 2)
#define MF2044_IOCTL_GET_DUTY_CYCLE _IO(MF2044_IOCTL_MAGIC, 3)
#define MF2044_IOCTL_SET_DUTY_CYCLE _IO(MF2044_IOCTL_MAGIC, 4)
#define MF2044_IOCTL_GET_FREQUENCY _IO(MF2044_IOCTL_MAGIC, 5)
#define MF2044_IOCTL_SET_FREQUENCY _IO(MF2044_IOCTL_MAGIC, 6)

#define FREQ_INIT 0
#define DUTY_INIT 0

int pm_init[4]={1,FREQ_INIT,DUTY_INIT,1};
int frequencies[6]={FREQ_INIT,FREQ_INIT,FREQ_INIT,FREQ_INIT,FREQ_INIT,FREQ_INIT};
int dutycycles[6]={DUTY_INIT,DUTY_INIT,DUTY_INIT,DUTY_INIT,DUTY_INIT,DUTY_INIT};

compat_module_param_array(pm_init,int,NULL,0);

/**
 * The context of a device instance
 *
 * A context is created each time a device is opened and passed to
 * other device handlers when they are called.
 *
 */
typedef struct buffer_s {
	int size;
	char data[LSIZE_MAX];
} buffer_t;

/**
 * set_prescale_div -	Set up the prescaler divider function
 * @rqst_prescaler:	prescaler value min
 * @prescale_div:	prescaler value set
 * @tb_clk_div:		Time Base Control prescaler bits
 */
static int set_prescale_div(unsigned long rqst_prescaler,
		unsigned short *prescale_div, unsigned short *tb_clk_div)
{
	unsigned int clkdiv, hspclkdiv;

	for (clkdiv = 0; clkdiv <= CLKDIV_MAX; clkdiv++) {
		for (hspclkdiv = 0; hspclkdiv <= HSPCLKDIV_MAX; hspclkdiv++) {
			/*
			 * calculations for prescaler value :
			 * prescale_div = HSPCLKDIVIDER * CLKDIVIDER.
			 * HSPCLKDIVIDER =  2 ** hspclkdiv
			 * CLKDIVIDER = (1),		if clkdiv == 0 *OR*
			 *		(2 * clkdiv),	if clkdiv != 0
			 *
			 * Configure prescale_div value such that period
			 * register value is less than 65535.
			 */
			*prescale_div = (1 << clkdiv) *
					(hspclkdiv ? (hspclkdiv * 2) : 1);
			if (*prescale_div > rqst_prescaler) {
				*tb_clk_div = (clkdiv << TBCTL_CLKDIV_SHIFT) |
					(hspclkdiv << TBCTL_HSPCLKDIV_SHIFT);
				return 0;
			}
		}
	}
	return 1;
}

static void ehrpwm_write(void *base, int offset, unsigned int val)
{
	writew(val & 0xFFFF, base + offset);
}

static void ehrpwm_modify(void *base, int offset,
		unsigned short mask, unsigned short val)
{
	unsigned short regval;

	regval = readw(base + offset);
	regval &= ~mask;
	regval |= val & mask;
	writew(regval, base + offset);
}

/**
 * Open the device
 *
 * This function is called when the device shall be opened.
 *
 */
static int mf2044_rtdm_open_nrt(struct rtdm_dev_context *context,
				rtdm_user_info_t * user_info, int oflags)
{
	buffer_t * buffer = (buffer_t *) context->dev_private;
	buffer->size = 0; /* clear the buffer */

	return 0;
}

/**
 * Close the device
 *
 * This function is called when the device shall be closed.
 *
 */
static int mf2044_rtdm_close_nrt(struct rtdm_dev_context *context,
				 rtdm_user_info_t * user_info)
{
	return 0;
}

static int _set_freq_duty(void __iomem *target,
	int freq_, int duty_, int isA)
{
	unsigned short ps_divval, tb_divval;
	unsigned short aqcsfrc_val, aqcsfrc_mask;
	unsigned long long c;
	unsigned long period_cycles, duty_cycles;

	period_cycles = freq_;
	duty_cycles   = duty_;

	ehrpwm_modify(target, AQSFRC, AQSFRC_RLDCSF_MASK,
			AQSFRC_RLDCSF_ZRO);
	if (1 == isA) {
		aqcsfrc_val = AQCSFRC_CSFA_FRCDIS;
		aqcsfrc_mask = AQCSFRC_CSFA_MASK;
	} else {
		aqcsfrc_val = AQCSFRC_CSFB_FRCDIS;
		aqcsfrc_mask = AQCSFRC_CSFB_MASK;
	}

	ehrpwm_modify(target, AQCSFRC, aqcsfrc_mask, aqcsfrc_val);

	/* Enable time counter for free_run */
	ehrpwm_modify(target, TBCTL, TBCTL_RUN_MASK, TBCTL_FREE_RUN);

	c = SYSCLK;
	c = (c * period_cycles);
	do_div(c, NSEC_PER_SEC);
	period_cycles = (unsigned long) c;

	c = SYSCLK;
	c = (c * duty_cycles);
	do_div(c, NSEC_PER_SEC);
	duty_cycles = (unsigned long) c;

	/* Configure clock prescaler to support Low frequency PWM wave */
	if (set_prescale_div(period_cycles/PERIOD_MAX, &ps_divval,
				&tb_divval)) {
		rtdm_printk("Unsupported values\n");
		return -EINVAL;
	}

	/* Update clock prescaler values */
	ehrpwm_modify(target, TBCTL, TBCTL_CLKDIV_MASK, tb_divval);
	/* Update period & duty cycle with presacler division */
	period_cycles = period_cycles / ps_divval;
	duty_cycles = duty_cycles / ps_divval;

	/* Configure shadow loading on Period register */
	ehrpwm_modify(target, TBCTL, TBCTL_PRDLD_MASK, TBCTL_PRDLD_SHDW);
	ehrpwm_write(target, TBPRD, period_cycles);

	/* Configure ehrpwm counter for up-count mode */
	ehrpwm_modify(target, TBCTL, TBCTL_CTRMODE_MASK,
			TBCTL_CTRMODE_UP);

	if (1 == isA) {
		ehrpwm_write(target, CMPA, duty_cycles);
	} else {
		ehrpwm_write(target, CMPB, duty_cycles);
	}

	return 0;
}

static int mf2044_rtdm_ioctl_nrt(struct rtdm_dev_context *context,
		rtdm_user_info_t *user_info, 
		unsigned int request, void __user *arg)
{
	int pindex = 0;
	void __iomem *target;
	int freq_ = 0;
	int duty_ = 0;
	int isA = 0;

	target = epwm1_0_map;

	if (request & MF2044_PWM_1_0) {
		target = epwm1_0_map;
		request &= ~(MF2044_PWM_1_0);
		pindex = 0;
		freq_ = frequencies[pindex];
		duty_ = dutycycles[pindex];
		isA = 1;
	} else if (request & MF2044_PWM_1_1) {
		target = epwm1_1_map;
		request &= ~(MF2044_PWM_1_1);
		pindex = 1;
		freq_ = frequencies[pindex-1];
		duty_ = dutycycles[pindex];
		isA = 0;
	} else if (request & MF2044_PWM_0_0) {
		target = epwm0_0_map;
		request &= ~(MF2044_PWM_0_0);
		pindex = 2;
		freq_ = frequencies[pindex];
		duty_ = dutycycles[pindex];
		isA = 1;
	} else if (request & MF2044_PWM_0_1) {
		target = epwm0_1_map;
		request &= ~(MF2044_PWM_0_1);
		pindex = 3;
		freq_ = frequencies[pindex-1];
		duty_ = dutycycles[pindex];
		isA = 0;
	} else if (request & MF2044_PWM_2_0) {
		target = epwm2_0_map;
		request &= ~(MF2044_PWM_2_0);
		pindex = 4;
		freq_ = frequencies[pindex];
		duty_ = dutycycles[pindex];
		isA = 1;
	} else if (request & MF2044_PWM_2_1) {
		target = epwm2_1_map;
		request &= ~(MF2044_PWM_2_1);
		pindex = 5;
		freq_ = frequencies[pindex-1];
		duty_ = dutycycles[pindex];
		isA = 0;
	}

	switch(request)
	{
		case MF2044_IOCTL_GET_DUTY_CYCLE:
			*(int*)arg = dutycycles[pindex];
			return 0;
		case MF2044_IOCTL_SET_DUTY_CYCLE:
			duty_ = (int)arg;
			dutycycles[pindex]=duty_;
			break;
		case MF2044_IOCTL_GET_FREQUENCY:
			*(int*)arg = frequencies[pindex];
			return 0;
		case MF2044_IOCTL_SET_FREQUENCY:
			freq_ = (int)arg;
			frequencies[pindex]=freq_;
			break;
	}

	_set_freq_duty(target,(int)freq_,(int)duty_,isA);

	return 0;
}

/**
 * This structure describe the RTDM device
 *
 */
static struct rtdm_device device = {
	.struct_version = RTDM_DEVICE_STRUCT_VER,

	.device_flags = RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.context_size = sizeof(buffer_t),
	.device_name = DEVICE_NAME,

	.open_nrt = mf2044_rtdm_open_nrt,

	.ops = {
		.close_nrt = mf2044_rtdm_close_nrt,
		.ioctl_rt = mf2044_rtdm_ioctl_nrt,
		.ioctl_nrt = mf2044_rtdm_ioctl_nrt,
	},

	.device_class = RTDM_CLASS_EXPERIMENTAL,
	.device_sub_class = SOME_SUB_CLASS,
	.profile_version = 1,
	.driver_name = DEVICE_NAME,
	.driver_version = RTDM_DRIVER_VER(0, 0, 1),
	.peripheral_name = "pwm driver for mf2044",
	.provider_name = "Seonghyun Kim",
	.proc_name = device.device_name,
};

/**
 * This function is called when the module is loaded
 *
 * It registers the RTDM device.
 * parameters:
 * pin : same as define orders. i.e.) P9.14 = 1, P8.13 = 6
 * freq : nanoseconds.
 * duty cycle : nanoseconds, not percent.
 * isA : a=1, b=0
 */
int __init rt_pwm_init(void)
{
	int res = -1;
	unsigned long period_cycles, duty_cycles;

	int pin_ = (int) pm_init[0];
	int freq_ = (int) pm_init[1];
	int duty_ = (int) pm_init[2];
	int isA = (int) pm_init[3];

	period_cycles = freq_;
	duty_cycles = duty_;

	res = rtdm_dev_register(&device);

	if (0 == res) {
		rtdm_printk("PWM driver registered without errors\n");
	} else {
		rtdm_printk("PWM driver registration failed: \n");
		switch(res) {
			case -EINVAL: 
				rtdm_printk("The device structure contains invalid entries. "
						"Check kernel log for further details.");
				break;
			case -ENOMEM: 
				rtdm_printk("The context for an exclusive device cannot be allocated.");
				break;
			case -EEXIST:
				rtdm_printk("The specified device name of protocol ID is already in use.");
				break;
			case -EAGAIN: 
				rtdm_printk("Some /proc entry cannot be created.");
				break;
			default:
				rtdm_printk("Unknown error code returned");
				break;
		}
		rtdm_printk("\n");
	}
	cm_per_map = ioremap(CM_PER_BASE,CM_PER_SZ);
	epwm1_0_map = ioremap(EPWM1_0_BASE,EPWM_SZ);
	epwm1_1_map = ioremap(EPWM1_1_BASE,EPWM_SZ);
	epwm0_0_map = ioremap(EPWM0_0_BASE,EPWM_SZ);
	epwm0_1_map = ioremap(EPWM0_1_BASE,EPWM_SZ);
	epwm2_0_map = ioremap(EPWM2_0_BASE,EPWM_SZ);
	epwm2_1_map = ioremap(EPWM2_1_BASE,EPWM_SZ);

	// enabling PWMSS clocks for eqep
	iowrite32(0x2, cm_per_map+EPWMSS1_CLK_CTRL);
	iowrite32(0x2, cm_per_map+EPWMSS0_CLK_CTRL);
	iowrite32(0x2, cm_per_map+EPWMSS2_CLK_CTRL);

	switch(pin_) {
		case 1:
			mmio_base = epwm1_0_map;
			break;
		case 2:
			mmio_base = epwm1_1_map;
			break;
		case 3:
			mmio_base = epwm0_0_map;
			break;
		case 4:
			mmio_base = epwm0_1_map;
			break;
		case 5:
			mmio_base = epwm2_0_map;
			break;
		case 6:
			mmio_base = epwm2_1_map;
			break;
		default:
			break;
	}
	if (freq_ != duty_) {
		_set_freq_duty(mmio_base, period_cycles, duty_cycles, isA);
	}

	return res;
}

/**
 * This function is called when the module is unloaded
 *
 * It unregister the RTDM device, polling at 1000 ms for pending users.
 *
 */
void __exit rt_pwm_exit(void)
{
	iowrite32(0x0, cm_per_map+EPWMSS1_CLK_CTRL);
	iowrite32(0x0, cm_per_map+EPWMSS0_CLK_CTRL);
	iowrite32(0x0, cm_per_map+EPWMSS2_CLK_CTRL);

	memset(frequencies, FREQ_INIT, sizeof(frequencies));
	memset(dutycycles, DUTY_INIT, sizeof(dutycycles));

	rtdm_dev_unregister(&device, 1000);
}

module_init(rt_pwm_init);
module_exit(rt_pwm_exit);
