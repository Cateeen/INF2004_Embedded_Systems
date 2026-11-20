#include "lsm303dlhc_reader.h"

ER lsm303dlhc_reader_init(lsm303dlhc_reader_t *reader,
			  const lsm303dlhc_axis_map_t *mapping,
			  lsm303dlhc_read_fresh_t read_fresh, void *transport_context)
{
	const uint8_t zero[6] = {0};
	accel_data_t decoded;
	lsm303dlhc_reader_t initial = {0};

	if (reader == NULL || mapping == NULL || read_fresh == NULL ||
	    !lsm303dlhc_accel_decode(zero, mapping, &decoded))
		return E_PAR;
	if (reader->initialized) return E_OBJ;
	initial.mapping = *mapping;
	initial.read_fresh = read_fresh;
	initial.transport_context = transport_context;
	initial.initialized = true;
	*reader = initial;
	return E_OK;
}

ER lsm303dlhc_reader_read(void *context, accel_data_t *sample, uint32_t *timestamp_ms)
{
	lsm303dlhc_reader_t *reader = context;
	lsm303dlhc_transfer_t transfer = {0};
	accel_data_t decoded;
	ER result;

	if (reader == NULL || sample == NULL || timestamp_ms == NULL) return E_PAR;
	if (!reader->initialized || reader->faulted) return E_OBJ;
	result = reader->read_fresh(reader->transport_context, &transfer);
	if (result != E_OK || transfer.length != sizeof(transfer.bytes) ||
	    !transfer.fresh || !transfer.timestamp_valid ||
	    !lsm303dlhc_accel_decode(transfer.bytes, &reader->mapping, &decoded)) {
		reader->faulted = true;
		return result < E_OK ? result : E_IO;
	}
	decoded.i2c_ok = true;
	*sample = decoded;
	*timestamp_ms = transfer.timestamp_ms;
	return E_OK;
}
