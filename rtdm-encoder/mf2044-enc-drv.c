/**
 * This kernel driver demonstrates how an RTDM device can be set up.
 *
 * It is a simple device, only 4 operation are provided:
 *  - open:  start device usage
 *  - close: ends device usage
 *  - write: store transfered data in an internal buffer
 *  - read:  return previously stored data and erase buffer
 *
 */

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
#include <linux/interrupt.h>
#include <linux/pinctrl/consumer.h>
#include <linux/input.h>
#include "../../../linux-dev/KERNEL/drivers/pwm/pwm-tipwmss.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Seonghyun Kim");

#define LSIZE_MAX		1024
#define DEVICE_NAME		"mf2044_enc_drv"
#define SOME_SUB_CLASS		4711

#define SYSCLK 100000000

#define CM_PER_BASE 0x44e00000
#define CM_PER_SZ 0x44e03fff-CM_PER_BASE
#define EPWMSS1_CLK_CTRL 0xcc
#define EPWMSS0_CLK_CTRL 0xd4
#define EPWMSS2_CLK_CTRL 0xd8

#define EQEP0_BASE 0x48300180
#define EQEP1_BASE 0x48302180
#define EQEP2_BASE 0x48304180
#define EQEP_SZ 0x7f

#define MF2044_ENC_0 1 << 4 
#define MF2044_ENC_1 1 << 5 
#define MF2044_ENC_2 1 << 6 

#define MF2044_MODE_ABSOLUTE 1
#define MF2044_MODE_RELATIVE 2

int mode = MF2044_MODE_ABSOLUTE;

void __iomem *cm_per_map;
void __iomem *eqep0_map;
void __iomem *eqep1_map;
void __iomem *eqep2_map;
void __iomem *mmio_base;

//Current hardware clock rate
u64 clk_rate = -1;//SYSCLK;

#define MF2044_IOCTL_MAGIC 0x00
#define MF2044_IOCTL_ON _IO(MF2044_IOCTL_MAGIC, 1)
#define MF2044_IOCTL_OFF _IO(MF2044_IOCTL_MAGIC, 2)
#define MF2044_IOCTL_GET_POSITION _IO(MF2044_IOCTL_MAGIC, 3)
#define MF2044_IOCTL_SET_POSITION _IO(MF2044_IOCTL_MAGIC, 4)
#define MF2044_IOCTL_GET_PERIOD _IO(MF2044_IOCTL_MAGIC, 5)
#define MF2044_IOCTL_SET_PERIOD _IO(MF2044_IOCTL_MAGIC, 6)
#define MF2044_IOCTL_GET_MODE _IO(MF2044_IOCTL_MAGIC, 7)
#define MF2044_IOCTL_SET_MODE _IO(MF2044_IOCTL_MAGIC, 8)

// eQEP register offsets from its base IO address
#define QPOSCNT    0x0000 // Position counter
#define QPOSINIT   0x0004
#define QPOSMAX    0x0008
#define QPOSCMP    0x000C
#define QPOSILAT   0x0010
#define QPOSSLAT   0x0014
#define QPOSLAT    0x0018 // Position count latch
#define QUTMR      0x001C // Unit timer
#define QUPRD      0x0020 // Unit period register
#define QWDTMR     0x0024
#define QWDPRD     0x0026
#define QDECCTL    0x0028
#define QEPCTL     0x002A // Control register
#define QCAPCTL    0x002C
#define QPOSCTL    0x002E
#define QEINT      0x0030
#define QFLG       0x0032
#define QCLR       0x0034
#define QFRC       0x0036
#define QEPSTS     0x0038
#define QCTMR      0x003A
#define QCPRD      0x003C
#define QCTMRLAT   0x003E
#define QCPRDLAT   0x0040
#define QREVID     0x005C

// Bits for the QDECTL register
#define QSRC1      (0x0001 << 15)
#define QSRC0      (0x0001 << 14)
#define SOEN       (0x0001 << 13)
#define SPSEL      (0x0001 << 12)
#define XCR        (0x0001 << 11)
#define SWAP       (0x0001 << 10)
#define IGATE      (0x0001 << 9)
#define QAP        (0x0001 << 8)
#define QBP        (0x0001 << 7)
#define QIP        (0x0001 << 6)
#define QSP        (0x0001 << 5)

// Bits for the QEPCTL register
#define FREESOFT1  (0x0001 << 15)
#define FREESOFT0  (0x0001 << 14)
#define PCRM1      (0x0001 << 13)
#define PCRM0      (0x0001 << 12)
#define SEI1       (0x0001 << 11)
#define SEI0       (0x0001 << 10)
#define IEI1       (0x0001 << 9)
#define IEI0       (0x0001 << 8)
#define SWI        (0x0001 << 7)
#define SEL        (0x0001 << 6)
#define IEL1       (0x0001 << 5)
#define IEL0       (0x0001 << 4)
#define PHEN       (0x0001 << 3)
#define QCLM       (0x0001 << 2)
#define UTE        (0x0001 << 1)
#define WDE        (0x0001 << 0)

// Bits for the QCAPCTL register
#define CEN        (0x0001 << 15)
#define CCPS2      (0x0001 << 6)
#define CCPS0      (0x0001 << 5)
#define CCPS1      (0x0001 << 4)
#define UPPS3      (0x0001 << 3)
#define UPPS2      (0x0001 << 2)
#define UPPS1      (0x0001 << 1)
#define UPPS0      (0x0001 << 0)

// Bits for the QPOSCTL register
#define PCSHDW     (0x0001 << 15)
#define PCLOAD     (0x0001 << 14)
#define PCPOL      (0x0001 << 13)
#define PCE        (0x0001 << 12)
#define PCSPW11    (0x0001 << 11)
#define PCSPW10    (0x0001 << 10)
#define PCSPW9    (0x0001 << 9)
#define PCSPW8    (0x0001 << 8)
#define PCSPW7    (0x0001 << 7)
#define PCSPW6    (0x0001 << 6)
#define PCSPW5    (0x0001 << 5)
#define PCSPW4    (0x0001 << 4)
#define PCSPW3    (0x0001 << 3)
#define PCSPW2    (0x0001 << 2)
#define PCSPW1    (0x0001 << 1)
#define PCSPW0    (0x0001 << 0)

// Bits for the interrupt registers
#define EQEP_INTERRUPT_MASK (0x0FFF)
#define UTOF                (0x0001 << 11)

// Bits to control the clock in the PWMSS subsystem
#define PWMSS_EQEPCLK_EN        BIT(4)
#define PWMSS_EQEPCLK_STOP_REQ  BIT(5)
#define PWMSS_EQEPCLK_EN_ACK    BIT(4)

// Modes for the eQEP unit
//  Absolute - the position entry represents the current position of the encoder.
//             Poll this value and it will be notified every period nanoseconds
//  Relative - the position entry represents the last latched position of the encoder
//             This value is latched every period nanoseconds and the internal counter
//             is subsequenty reset
#define TIEQEP_MODE_ABSOLUTE    0
#define TIEQEP_MODE_RELATIVE    1

// Structure defining the characteristics of the eQEP unit
struct rt_eqep_chip
{
    // Platform device for this eQEP unit
    struct platform_device *pdev;
    
    // Pointer to the base of the memory of the eQEP unit
    void __iomem           *mmio_base;
    
    // SYSCLKOUT to the eQEP unit
    u32                     clk_rate;
    
    // IRQ for the eQEP unit
    u16                     irq;
    
    // Mode of the eQEP unit
    u8                      mode;
    
    // work stuct for the notify userspace work
    struct work_struct      notify_work;
    
    // Backup for driver suspension
    u16                     prior_qepctl;
    u16                     prior_qeint;
};

static int mf2044_rtdm_ioctl_nrt(struct rtdm_dev_context *context,
		rtdm_user_info_t *user_info, 
		unsigned int request, void __user *arg)
{
	int32_t position = 0;
	u16 tmp;
	u64 period;
	u16 modeval = 0;
	mmio_base = eqep1_map;

	if (request & MF2044_ENC_0) {
		mmio_base = eqep0_map;
		request &= ~(MF2044_ENC_0);
	} else if (request & MF2044_ENC_1) {
		mmio_base = eqep1_map;
		request &= ~(MF2044_ENC_1);
	} else if (request & MF2044_ENC_2) {
		mmio_base = eqep2_map;
		request &= ~(MF2044_ENC_2);
	}

	switch (request)
	{
		case MF2044_IOCTL_GET_POSITION:
			if (mode == MF2044_MODE_ABSOLUTE) {
				position = readl(mmio_base + QPOSCNT);
			} else if (mode == MF2044_MODE_RELATIVE) {
				position = readl(mmio_base + QPOSLAT);
			} else {
				rtdm_printk("You have set MODE properly. It should be Absolute or Relative.\n");
				return -1;
			}
			*(int32_t*)arg = position;
			break;
		case MF2044_IOCTL_SET_POSITION:
			position = (int32_t)arg;
			if (mode == MF2044_MODE_ABSOLUTE) {
				 writel(position, mmio_base + QPOSCNT);
			}
			break;
		case MF2044_IOCTL_GET_PERIOD:
			if(!(readw(mmio_base + QEPCTL) & UTE))
			{
				*(uint64_t*)arg=0;
				return 0;
			}
			period = readl(mmio_base + QUPRD);
			period = period * NSEC_PER_SEC;
			do_div(period, clk_rate);
			*(uint64_t*)arg = period;
			break;
		case MF2044_IOCTL_SET_PERIOD:
			period = (u64)arg;
			// Disable the unit timer before modifying its period register
			tmp = readw(mmio_base + QEPCTL);
			tmp = tmp & ~UTE & ~QCLM;
			writew(tmp, mmio_base + QEPCTL);
			// Zero the unit timer counter register
			writel(0x0, mmio_base + QUTMR);
			if(period)
			{
				// Otherwise calculate the period
				period = period * clk_rate;
				do_div(period, NSEC_PER_SEC);
				// Set this period into the unit timer period register
				writel(period & 0x00000000FFFFFFFF, mmio_base + QUPRD);
				// Enable the unit timer, and latch QPOSLAT to QPOSCNT on overflow
				tmp = readw(mmio_base + QEPCTL);
				tmp = tmp | UTE | QCLM;
				writew(tmp, mmio_base + QEPCTL);
			}
			break;
		case MF2044_IOCTL_GET_MODE:
			*(int*)arg = mode;
			break;
		case MF2044_IOCTL_SET_MODE:
			mode = (int)arg;
			modeval = readw(mmio_base + QEPCTL);
			if (MF2044_MODE_ABSOLUTE == mode) {
				modeval = modeval & ~PCRM1 & ~PCRM0;
			} else if (MF2044_MODE_RELATIVE == mode){
				modeval = modeval | PCRM1 | PCRM0;
			} else {
				rtdm_printk("mode should be Absolute or Relative.\n");
				return -1;
			}
			writew(modeval, mmio_base + QEPCTL);
			break;

	}

	return 0;
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
	u64               period;
	u16               status;
	u32                value;
	int i = 0;

	for (i=0;i<3;i++) {
		switch (i) {
			case 0:
				mmio_base = eqep0_map;
				break;
			case 1:
				mmio_base = eqep1_map;
				break;
			case 2:
				mmio_base = eqep2_map;
				break;
			default:
				return -1;
		}
	
		// Read decoder control settings
		status = readw(mmio_base + QDECCTL);
	
		// Setting control register
		value = 1;
		status = ((value) ? status | QSRC0 : status & ~QSRC0) & ~QSRC1;
		value = 0;
		status = (value) ? status | QAP : status & ~QAP;
		status = (value) ? status | QBP : status & ~QBP;
		status = (value) ? status | QIP : status & ~QIP;
		status = (value) ? status | QSP : status & ~QSP;
		status = (value) ? status | SWAP : status & ~SWAP;
	
		writew(status, mmio_base + QDECCTL);
	
		// Initialize the position counter to zero
		writel(0, mmio_base + QPOSINIT);
	
		// To prevent overflow problem by setting maximum value as -1. The eQEP subsystem has a register
		// that defined the maximum value of the encoder as an unsigned.  If the counter is zero
		// and decrements again, it is set to QPOSMAX.  If you cast -1 (signed int) to
		// to an unsigned, you will notice that -1 == UINT_MAX.  So when the counter is set to this
		// maximum position, when read into a signed int, it will equal -1.  Two's complement for 
		// the WIN!!
		writel(-1, mmio_base + QPOSMAX);
	
		// Enable some interrupts
		status = readw(mmio_base + QEINT);
		status = status | UTOF; 
		// UTOF - Unit Time Period interrupt.  This is triggered when the unit timer period expires
	
		writew(status, mmio_base + QEINT);
	
		// Calculate the timer ticks per second
		period = 1000000000;
		period = period * clk_rate;
		do_div(period, NSEC_PER_SEC);
	
		// Set this period into the unit timer period register
		writel(period & 0x00000000FFFFFFFF, mmio_base + QUPRD);
	
		// Enable the eQEP with basic position counting turned on
		status = readw(mmio_base + QEPCTL);
		status = status | PHEN | IEL0 | SWI | UTE | QCLM;
		// PHEN - Quadrature position counter enable bit
		// IEL0 - Latch QPOSILAT on index signal.  This can be rising or falling, IEL[1:0] = 0 is reserved
		// SWI  - Software initialization of position count register.  Basic set QPOSCNT <= QPOSINIT
		// UTE  - unit timer enable
		// QCLM - latch QPOSLAT to QPOSCNT upon unit timer overflow
		writew(status, mmio_base + QEPCTL);
	}

	rtdm_printk("PWM driver open without errors!\n");

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

/**
 * This structure describe the simple RTDM device
 *
 */
static struct rtdm_device device = {
	.struct_version = RTDM_DEVICE_STRUCT_VER,

	.device_flags = RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
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
	.peripheral_name = "encoder driver for mf2044",
	.provider_name = "Seonghyun Kim",
	.proc_name = device.device_name,
};

/**
 * This function is called when the module is loaded
 *
 * It registers the RTDM device.
 *
 */
static int __init rt_enc_init(void)
{
	int res = -1;
	res = rtdm_dev_register(&device);

	if (0 == res) {
		rtdm_printk("PWM driver registered without errors!\n");
		cm_per_map = ioremap(CM_PER_BASE,CM_PER_SZ);
		eqep0_map = ioremap(EQEP0_BASE,EQEP_SZ);
		eqep1_map = ioremap(EQEP1_BASE,EQEP_SZ);
		eqep2_map = ioremap(EQEP2_BASE,EQEP_SZ);

		// enabling PWMSS clocks for eqep
		iowrite32(0x2, cm_per_map+EPWMSS1_CLK_CTRL);
		iowrite32(0x2, cm_per_map+EPWMSS0_CLK_CTRL);
		iowrite32(0x2, cm_per_map+EPWMSS2_CLK_CTRL);

		clk_rate = SYSCLK;
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
	return res;
}

/**
 * This function is called when the module is unloaded
 *
 * It unregister the RTDM device, polling at 1000 ms for pending users.
 *
 */
static void __exit rt_enc_exit(void)
{
	rtdm_dev_unregister(&device, 1000);
}

module_init(rt_enc_init);
module_exit(rt_enc_exit);

