#ifndef LSM303DLHC_READER_H
#define LSM303DLHC_READER_H

#include <tk/tkernel.h>
#include "lsm303dlhc_accel.h"

typedef struct {
	uint8_t bytes[6];
	uint8_t length;
	bool fresh;
	bool timestamp_valid;
	uint32_t timestamp_ms;
} lsm303dlhc_transfer_t;

/* The transport exclusively owns the bus and must verify the sensor is
 * configured for the decoder's mode. E_OK with fresh=true means it verified
 * a new sample and a complete transfer, not merely a successful bus call.
 * The timestamp must be on the tk_get_otm clock and no later than acquisition.
 * It may block for its documented sensor-ready and kernel I2C timeouts; it
 * must not use network APIs. */
typedef ER (*lsm303dlhc_read_fresh_t)(void *context, lsm303dlhc_transfer_t *transfer);

typedef struct {
	lsm303dlhc_axis_map_t mapping;
	lsm303dlhc_read_fresh_t read_fresh;
	void *transport_context;
	bool initialized;
	bool faulted;
} lsm303dlhc_reader_t;

/* Single owner: initialize a zeroed reader once, after reserving the bus.
 * This adapter neither configures nor accesses the sensor or bus itself. */
ER lsm303dlhc_reader_init(lsm303dlhc_reader_t *reader,
			  const lsm303dlhc_axis_map_t *mapping,
			  lsm303dlhc_read_fresh_t read_fresh, void *transport_context);

/* Compatible with imu_read_t. Errors leave outputs unchanged and latch the
 * reader faulted until restart. The IMU task independently checks timestamps. */
ER lsm303dlhc_reader_read(void *context, accel_data_t *sample, uint32_t *timestamp_ms);

#endif
