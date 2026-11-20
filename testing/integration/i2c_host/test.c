#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>

#define __DEV_I2C_H__
#define __DEVINC_DEVDEF_H__
#define DEV_IIC_ENABLE 1
#define DEVCNF_I2C0_INTPRI 2
#define DEVCNF_I2C1_INTPRI 2
#define DEVCNF_I2C0_TMO 1000
#define DEVCNF_I2C1_TMO 1000
#define DEVCNF_I2C_RESET 0
#define DEVCNF_I2C_SETPINFUNC 0
#include "../../../device/i2c/sysdepend/rp2040/i2c_sysdep.h"
#include "../../../device/include/dev_i2c.h"

typedef struct { UW unit; } T_I2C_DCB;
enum { LLD_I2C_OPEN, LLD_I2C_CLOSE, LLD_I2C_READ, LLD_I2C_WRITE, LLD_I2C_EXEC };
#include "../../../device/i2c/sysdepend/rp2040/i2c_rp2040.c"

static UW mask, raw, commands[32], rx_value;
UINT i2c_host_irq_masked;
static unsigned flag_set_masked, flag_set_unmasked;
#if defined(APP_SENSOR_TEST) && APP_SENSOR_TEST == 2
static UW fake_time_us;
#endif
static unsigned command_count, received_count, stop_waits, stop_cleared;
static unsigned wait_calls, spurious_on_wait;
static BOOL flag, suppress_stop, abort_first_command, early_disable;

UW in_w(UW address)
{
#if defined(APP_SENSOR_TEST) && APP_SENSOR_TEST == 2
	if (address == TIMER_TIMERAWL) return fake_time_us;
	if (address == I2C_RAW_INTR_STAT(1)) return raw;
#endif
	if (address == I2C_INTR_STAT(1)) return (raw | I2C_INT_TX_EMPTY) & mask;
	if (address == I2C_DATA_CMD(1)) {
		raw &= ~I2C_INT_RX_FULL;
		return rx_value;
	}
	if (address == I2C_CLR_INTR(1)) { raw = 0; return 0; }
	if (address == I2C_CLR_STOP_DET(1)) {
		raw &= ~I2C_INT_STOP_DET;
		stop_cleared++;
		return 0;
	}
	if (address == I2C_CLR_TX_ABRT(1)) { raw &= ~I2C_INT_TX_ABRT; return 0; }
	return 0;
}

void out_w(UW address, UW value)
{
	if (address == I2C_INTR_MASK(1)) mask = value;
	if (address == I2C_ENABLE(1) && value == 0 && !stop_cleared &&
	    !suppress_stop && !abort_first_command) early_disable = TRUE;
	if (address != I2C_DATA_CMD(1)) return;
	if (command_count < sizeof(commands) / sizeof(commands[0]))
		commands[command_count++] = value;
	if (abort_first_command) {
		raw |= I2C_INT_TX_ABRT;
		return;
	}
	if (value & I2C_DATA_CMD_CMD_READ) {
		rx_value = 0x40U + received_count++;
		raw |= I2C_INT_RX_FULL;
	}
	if ((value & I2C_DATA_CMD_STOP) && !suppress_stop) raw |= I2C_INT_STOP_DET;
}

void set_w(UW address, UW value)
{
	if (address == I2C_INTR_MASK(1)) mask |= value;
}

void clr_w(UW address, UW value)
{
	if (address == I2C_INTR_MASK(1)) mask &= ~value;
}

void ClearInt(UINT intno) { (void)intno; }
void EnableInt(UINT intno, PRI priority) { (void)intno; (void)priority; }
ID tk_cre_flg(const T_CFLG *config) { (void)config; return 1; }
ER tk_del_flg(ID id) { (void)id; return E_OK; }
ER tk_set_flg(ID id, UINT bits)
{
	(void)id;
	(void)bits;
	if (i2c_host_irq_masked) flag_set_masked++;
	else flag_set_unmasked++;
	flag = TRUE;
	return E_OK;
}
ER tk_clr_flg(ID id, UINT bits) { (void)id; (void)bits; flag = FALSE; return E_OK; }
ER tk_def_int(UINT intno, const T_DINT *config)
{
	(void)intno;
	(void)config;
	return E_OK;
}

ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *pattern, TMO timeout)
{
	unsigned count;
	(void)id;
	(void)bits;
	(void)mode;
	(void)timeout;
	if (ll_devcb[1].state == I2C_STS_WAIT_STOP) stop_waits++;
	if (++wait_calls == spurious_on_wait) {
		(void)tk_set_flg(id, bits);
		flag = FALSE;
		*pattern = 1U;
		return E_OK;
	}
	for (count = 0; count < 64U && !flag; count++) {
		if (!in_w(I2C_INTR_STAT(1))) return E_TMOUT;
#if defined(APP_SENSOR_TEST) && APP_SENSOR_TEST == 2
		fake_time_us += 100U;
#endif
		i2c_inthdr(INTNO_I2C1);
#if defined(APP_SENSOR_TEST) && APP_SENSOR_TEST == 2
		fake_time_us += 700U;
#endif
	}
	if (!flag) return E_TMOUT;
	flag = FALSE;
	*pattern = 1U;
	return E_OK;
}

static void reset_mock(void)
{
	mask = raw = command_count = received_count = stop_waits = stop_cleared = 0U;
	i2c_host_irq_masked = flag_set_masked = flag_set_unmasked = 0U;
	wait_calls = spurious_on_wait = 0U;
	flag = suppress_stop = abort_first_command = early_disable = FALSE;
#if defined(APP_SENSOR_TEST) && APP_SENSOR_TEST == 2
	fake_time_us = 0U;
#endif
	ll_devcb[1] = (T_I2C_LLDCB){.done_flgid = 1};
}

static BOOL transaction_test(void)
{
	UB send[2] = {0x28U, 0x57U};
	UB receive[6] = {0};
	T_I2C_EXEC exec = {0x19U, 1, send, 6, receive};
	BOOL ok;
	unsigned i;

	reset_mock();
	ok = dev_i2c_llctl(1, LLD_I2C_WRITE, 0x19U, 2U, (UW *)send) == 2 &&
	     command_count == 2U && commands[0] == 0x28U &&
	     commands[1] == (0x57U | I2C_DATA_CMD_STOP) &&
	     stop_waits == 1U && stop_cleared == 1U && !early_disable;
	reset_mock();
	ok = dev_i2c_llctl(1, LLD_I2C_READ, 0x19U, 1U, (UW *)receive) == 1 &&
	     command_count == 1U &&
	     commands[0] == (I2C_DATA_CMD_CMD_READ | I2C_DATA_CMD_STOP) &&
	     receive[0] == 0x40U && stop_waits == 1U && stop_cleared == 1U &&
	     !early_disable && ok;
	reset_mock();
	ok = dev_i2c_llctl(1, LLD_I2C_READ, 0x19U, 2U, (UW *)receive) == 2 &&
	     command_count == 2U && commands[0] == I2C_DATA_CMD_CMD_READ &&
	     commands[1] == (I2C_DATA_CMD_CMD_READ | I2C_DATA_CMD_STOP) &&
	     stop_waits == 1U && stop_cleared == 1U && !early_disable && ok;
	reset_mock();
	exec.rcv_size = 1;
	ok = dev_i2c_llctl(1, LLD_I2C_EXEC, 0, 0, (UW *)&exec) == 2 &&
	     command_count == 2U && commands[0] == 0x28U &&
	     commands[1] == (I2C_DATA_CMD_CMD_READ | I2C_DATA_CMD_RESTART |
			     I2C_DATA_CMD_STOP) &&
	     stop_waits == 1U && stop_cleared == 1U && !early_disable && ok;
	reset_mock();
	exec.rcv_size = 6;
	ok = dev_i2c_llctl(1, LLD_I2C_EXEC, 0, 0, (UW *)&exec) == 7 &&
	     command_count == 7U && commands[0] == 0x28U &&
	     commands[1] == (I2C_DATA_CMD_CMD_READ | I2C_DATA_CMD_RESTART) &&
	     commands[6] == (I2C_DATA_CMD_CMD_READ | I2C_DATA_CMD_STOP) &&
	     stop_waits == 1U && stop_cleared == 1U && !early_disable && ok;
	for (i = 2U; i < 6U; i++) ok = commands[i] == I2C_DATA_CMD_CMD_READ && ok;
	for (i = 0U; i < 6U; i++) ok = receive[i] == 0x40U + i && ok;
	ok = flag_set_masked == 0U && flag_set_unmasked == 3U && ok;
	return ok;
}

static BOOL spurious_wake_test(void)
{
	UB send = 0x28U, receive[2] = {0};
	T_I2C_EXEC exec = {0x19U, 1, &send, 2, receive};
	unsigned phase;

	for (phase = 1U; phase <= 3U; phase++) {
		reset_mock();
		suppress_stop = TRUE;
		if (dev_i2c_llctl(1, LLD_I2C_WRITE, 0x19U, 1U, (UW *)&send) != E_TMOUT)
			return FALSE;
		suppress_stop = FALSE;
		spurious_on_wait = wait_calls + phase;
		if (dev_i2c_llctl(1, LLD_I2C_EXEC, 0, 0, (UW *)&exec) != E_IO ||
		    wait_calls != spurious_on_wait || mask != 0U || stop_cleared != 0U)
			return FALSE;
		spurious_on_wait = 0U;
		if (dev_i2c_llctl(1, LLD_I2C_EXEC, 0, 0, (UW *)&exec) != 3 ||
		    stop_cleared != 1U)
			return FALSE;
	}
	reset_mock();
	i2c_inthdr(INTNO_I2C1);
	if (flag || ll_devcb[1].ioerr != E_OK ||
	    dev_i2c_llctl(1, LLD_I2C_EXEC, 0, 0, (UW *)&exec) != 3)
		return FALSE;
	return TRUE;
}

static BOOL failure_test(void)
{
	UB data[2] = {0x20U, 0x57U};
	BOOL ok;

	reset_mock();
	suppress_stop = TRUE;
	ok = dev_i2c_llctl(1, LLD_I2C_WRITE, 0x19U, 1U, (UW *)data) == E_TMOUT &&
	     stop_waits == 1U && mask == 0U;
	reset_mock();
	abort_first_command = TRUE;
	ok = dev_i2c_llctl(1, LLD_I2C_WRITE, 0x19U, 1U, (UW *)data) == E_IO &&
	     stop_waits == 1U && mask == 0U && ok;
	reset_mock();
	abort_first_command = TRUE;
	ok = dev_i2c_llctl(1, LLD_I2C_WRITE, 0x19U, 2U, (UW *)data) == E_IO &&
	     stop_waits == 0U && mask == 0U && ok;
	return ok;
}

#if defined(APP_SENSOR_TEST) && APP_SENSOR_TEST == 2
static BOOL trace_test(void)
{
	UB send = 0xA7U, receive[7] = {0};
	T_I2C_EXEC exec = {0x19U, 1, &send, 7, receive};
	T_I2C_PROBE_TRACE trace;

	reset_mock();
	if (dev_i2c_probe_trace(2U, &trace) != E_PAR ||
	    dev_i2c_probe_trace(1U, NULL) != E_PAR ||
	    dev_i2c_llctl(1, LLD_I2C_EXEC, 0, 0, (UW *)&exec) != 8 ||
	    dev_i2c_probe_trace(1U, &trace) != E_OK)
		return FALSE;
	if (ll_devcb[1].probe_active || trace.irq_count != 9U ||
	    trace.receive_irq_count != 7U ||
	    trace.first_irq_us - trace.start_us != 100U ||
	    trace.receive_command_us - trace.start_us != 800U ||
	    trace.first_receive_irq_us - trace.start_us != 900U ||
	    trace.last_receive_irq_us - trace.start_us != 5700U ||
	    trace.receive_resume_us - trace.start_us != 6400U ||
	    trace.stop_arm_us - trace.start_us != 6400U ||
	    trace.stop_pending != 1U ||
	    trace.last_irq_us - trace.first_irq_us != 6400U ||
	    trace.end_us - trace.last_irq_us != 700U) {
		printf("trace: count=%u start=%u first=%u last=%u end=%u\n",
		       trace.irq_count, trace.start_us, trace.first_irq_us,
		       trace.last_irq_us, trace.end_us);
		return FALSE;
	}
	reset_mock();
	suppress_stop = TRUE;
	if (dev_i2c_llctl(1, LLD_I2C_EXEC, 0, 0, (UW *)&exec) != E_TMOUT ||
	    ll_devcb[1].probe_active ||
	    dev_i2c_probe_trace(1U, &trace) != E_OK || trace.irq_count == 0U)
		return FALSE;
	return TRUE;
}
#endif

int main(void)
{
	BOOL transactions = transaction_test();
	BOOL failures = failure_test();
	BOOL spurious = spurious_wake_test();
	printf("[TEST] I2C-U01 %s  command sequencing and STOP completion\n",
	       transactions ? "PASS" : "FAIL");
	printf("[TEST] I2C-U02 %s  timeout and abort propagation\n",
	       failures ? "PASS" : "FAIL");
	printf("[TEST] I2C-U04 %s  premature completion signals\n",
	       spurious ? "PASS" : "FAIL");
#if defined(APP_SENSOR_TEST) && APP_SENSOR_TEST == 2
	{
		BOOL trace = trace_test();
		printf("[TEST] I2C-U03 %s  diagnostic interrupt timing\n", trace ? "PASS" : "FAIL");
		return transactions && failures && spurious && trace ? 0 : 1;
	}
#else
	return transactions && failures && spurious ? 0 : 1;
#endif
}
