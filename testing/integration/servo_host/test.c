#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define APP_H
#define HARDWARE_SENSOR_TEST_H
#define GPIO_DISPATCHER_H
#define LSM303DLHC_I2C_H
#define LINE_ADC_READER_H
#define OBS_PICO_PORT_H
#define OBS_TASK_H
#define VEHICLE_BUFFERS_H
#define __SYS_SYSDEF_H__
#define __TK_SYSLIB_H__
#define APP_SENSOR_TEST 4

typedef int INT;
typedef int ER;
typedef int ID;
typedef unsigned char UB;
typedef void (*FP)(INT, void *);

#define E_OK 0
#define TRUE 1
#define FALSE 0
#define TMO_FEVR (-1)
#define TP_PRC1 1
#define PWM_BASE 0x40050000U
#define PWM_CHx_CSR 0x00U
#define PWM_CHx_DIV 0x04U
#define PWM_CHx_CC 0x0cU
#define PWM_CHx_TOP 0x10U
#define RESETS_RESET_DONE 0x4000c008U
#define RESETS_RESET_PWM 0x4000U
#define GPIO_CTRL(pin) (0x40014004U + (pin) * 8U)
#define GPIO(pin) (0x4001c004U + (pin) * 4U)

static char events[32];
static unsigned event_count;
static unsigned last_pulse;

static void record(char event)
{
	if (event_count < sizeof(events) - 1U) events[event_count++] = event;
}

ER pwm_set_enabled(unsigned pin, int enabled)
{
	if (pin != 15U) return -1;
	record(enabled ? 'E' : 'D');
	return E_OK;
}

ER pwm_set_cc(unsigned pin, unsigned pulse)
{
	if (pin != 15U) return -1;
	last_pulse = pulse;
	record('C');
	return E_OK;
}

ER pwm_set_wrap(unsigned pin, unsigned wrap)
{
	if (pin != 15U || wrap != 19999U) return -1;
	record('W');
	return E_OK;
}

ER pwm_set_pin(unsigned pin)
{
	if (pin != 15U) return -1;
	record('P');
	return E_OK;
}

void mock_out_w(unsigned address, unsigned value)
{
	if (address == PWM_BASE + PWM_CHx_DIV + 7U * 0x14U &&
	    value == (125U << 4))
		record('I');
	else
		record('!');
}
#define out_w(address, value) mock_out_w((address), (value))

unsigned mock_in_w(unsigned address)
{
	(void)address;
	return 0U;
}
#define in_w(address) mock_in_w(address)

ER tk_dly_tsk(unsigned delay)
{
	record(delay == 5000U ? 'T' : '!');
	return E_OK;
}

ER tk_slp_tsk(INT timeout)
{
	record(timeout == TMO_FEVR ? 'S' : '!');
	return E_OK;
}

INT tm_printf(const UB *format, ...)
{
	(void)format;
	return 0;
}

ID app_start_task(FP entry, int priority, unsigned core, const char *name)
{
	(void)entry;
	(void)priority;
	(void)core;
	(void)name;
	return 1;
}

#include "../../../app_program/hardware_sensor_test.c"

int main(void)
{
	sensor_test_task(0, NULL);
#if SERVO_TEST_US == 0
	if (strcmp(events, "DCS") != 0 || last_pulse != 0U) {
#else
	if (strcmp(events, "DCWIPTCES") != 0 || last_pulse != SERVO_TEST_US) {
#endif
		printf("FAIL: events=%s pulse=%u\n", events, last_pulse);
		return 1;
	}
	printf("PASS: servo diagnostic pulse=%u\n", last_pulse);
	return 0;
}
