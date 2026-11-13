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

typedef struct { UW unit; } T_I2C_DCB;
typedef struct {
	UW sadr;
	INT snd_size;
	UB *snd_data;
	INT rcv_size;
	UB *rcv_data;
} T_I2C_EXEC;
enum { LLD_I2C_OPEN, LLD_I2C_CLOSE, LLD_I2C_READ, LLD_I2C_WRITE, LLD_I2C_EXEC };
#include "../../../device/i2c/sysdepend/rp2040/i2c_rp2040.c"

static UW mask, raw, commands[32], rx_value;
static unsigned command_count, received_count, stop_waits, stop_cleared;
static BOOL flag, suppress_stop, abort_first_command, early_disable;

UW in_w(UW address)
{
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
ER tk_set_flg(ID id, UINT bits) { (void)id; (void)bits; flag = TRUE; return E_OK; }
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
	for (count = 0; count < 64U && !flag; count++) {
		if (!in_w(I2C_INTR_STAT(1))) return E_TMOUT;
		i2c_inthdr(INTNO_I2C1);
	}
	if (!flag) return E_TMOUT;
	flag = FALSE;
	*pattern = 1U;
	return E_OK;
}

static void reset_mock(void)
{
	mask = raw = command_count = received_count = stop_waits = stop_cleared = 0U;
	flag = suppress_stop = abort_first_command = early_disable = FALSE;
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
	return ok;
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

int main(void)
{
	BOOL transactions = transaction_test();
	BOOL failures = failure_test();
	printf("[TEST] I2C-U01 %s  command sequencing and STOP completion\n",
	       transactions ? "PASS" : "FAIL");
	printf("[TEST] I2C-U02 %s  timeout and abort propagation\n",
	       failures ? "PASS" : "FAIL");
	return transactions && failures ? 0 : 1;
}
