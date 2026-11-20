#include <stdint.h>
#include <stdio.h>

#define __SYS_MACHINE_H__
#define _KERNEL_
#define __TM_TMONITOR_H__
#define _SYSDEPEND_TARGET_SYSDEPEND_
#define PICO_RP2040
#define TM_WIFI_CYW43 0
#define USE_PTMR 0
#define USE_SDEV_DRV 0
#define USE_SHUTDOWN 0
#define BOARD_UART0_FREE 1
#define BOARD_DIAG_GPIO_SCAN 0
#define LOCAL static
#define EXPORT
#define E_NOSPT (-1)
#define E_PAR (-2)
#define SYSTEM_MESSAGE(message) ((void)0)
#define CLKATR_USB 1U
#define CLKATR_ADC 2U
#define CLKATR_RTC 4U
#define CLKATR_PREI 8U
#define RESETS_RESET 0x4000c000U
#define RESETS_RESET_DONE 0x4000c008U
#define RESETS_RESET_IO_BANK0 0x20U
#define RESETS_RESET_PADS_BANK0 0x100U
#define RESETS_RESET_UART0 0x400000U
#define RESETS_RESET_PWM 0x4000U

typedef uint32_t UW;
typedef int INT;
typedef int ER;
typedef int W;

static UW reset_register = 0x01ffffffU;

void startup_clock(UW flags) { (void)flags; }
void mock_set(UW address, UW bits) { (void)address; reset_register |= bits; }
void mock_clear(UW address, UW bits) { (void)address; reset_register &= ~bits; }
UW mock_in(UW address) { (void)address; return ~reset_register; }
void mock_out(UW address, UW value) { (void)address; (void)value; }
#define set_w(address, bits) mock_set((address), (bits))
#define clr_w(address, bits) mock_clear((address), (bits))
#define in_w(address) mock_in(address)
#define out_w(address, value) mock_out((address), (value))

#include "../../../kernel/sysdepend/pico_rp2040/hw_setting.c"

int main(void)
{
	knl_startup_hw();
	if ((reset_register & RESETS_RESET_PWM) != 0U) {
		puts("FAIL: PWM remains in reset with USE_PTMR=0");
		return 1;
	}
	puts("PASS: PWM released from reset with USE_PTMR=0");
	return 0;
}
