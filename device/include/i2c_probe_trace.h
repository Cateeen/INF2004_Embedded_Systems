#ifndef I2C_PROBE_TRACE_H
#define I2C_PROBE_TRACE_H

typedef struct {
	UW start_us;
	UW first_irq_us;
	UW receive_command_us;
	UW first_receive_irq_us;
	UW last_receive_irq_us;
	UW receive_resume_us;
	UW stop_arm_us;
	UW stop_pending;
	UW last_irq_us;
	UW end_us;
	UW irq_count;
	UW receive_irq_count;
} T_I2C_PROBE_TRACE;

EXPORT ER dev_i2c_probe_trace(UW unit, T_I2C_PROBE_TRACE *trace);

#endif
