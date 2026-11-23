#include "../../app.h"
#include "line_adc_reader.h"

#include <sys/sysdef.h>

#define LINE_ADC_OVERSAMPLES 8U
#define LINE_ADC_MAX_VALUE 4095U
#define LINE_ADC_FIRST_PIN 26U
#define LINE_ADC_BLACK_THRESHOLD 1000U

static line_adc_reader_t s_task_reader;
static line_adc_config_t s_task_reader_config;
static bool s_task_start_attempted;

static ER initialize_task_reader(void *context)
{
	return line_adc_reader_init(context, &s_task_reader_config);
}

static bool valid_channel(const line_adc_channel_t *channel)
{
	return channel->channel <= 2U && channel->threshold <= LINE_ADC_MAX_VALUE;
}

static bool valid_config(const line_adc_config_t *config)
{
	return config != NULL && valid_channel(&config->barcode) &&
	       valid_channel(&config->center) && valid_channel(&config->right) &&
	       config->barcode.channel != config->center.channel &&
	       config->barcode.channel != config->right.channel &&
	       config->center.channel != config->right.channel &&
	       config->max_sample_span_ms > 0U &&
	       config->max_sample_span_ms <= INT32_MAX;
}

static bool is_black(const line_adc_channel_t *config, uint16_t raw)
{
	return config->black_above ? raw > config->threshold : raw < config->threshold;
}

static uint64_t clock_ms(const SYSTIM *time)
{
	return ((uint64_t)(uint32_t)time->hi << 32) | time->lo;
}

ER line_adc_reader_default_config(line_adc_config_t *config,
				  uint32_t max_sample_span_ms)
{
	line_adc_config_t imported = {
		{2U, LINE_ADC_BLACK_THRESHOLD, true},
		{1U, LINE_ADC_BLACK_THRESHOLD, true},
		{0U, LINE_ADC_BLACK_THRESHOLD, true},
		max_sample_span_ms
	};

	if (config == NULL || !valid_config(&imported)) return E_PAR;
	*config = imported;
	return E_OK;
}

ER line_adc_reader_init(line_adc_reader_t *reader, const line_adc_config_t *config)
{
	const line_adc_channel_t *channels[3];
	ID device;
	unsigned i;

	if (reader == NULL || !valid_config(config)) return E_PAR;
	if (reader->initialized) return E_OBJ;
	device = tk_opn_dev((const UB *)"adca", TD_READ | TD_EXCL);
	if (device < E_OK) return device;
	if (device == E_OK) return E_SYS;

	channels[0] = &config->barcode;
	channels[1] = &config->center;
	channels[2] = &config->right;
	for (i = 0U; i < 3U; i++) {
		UINT pin = LINE_ADC_FIRST_PIN + channels[i]->channel;
		out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
		clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
	}
	reader->config = *config;
	reader->device = device;
	reader->faulted = false;
	reader->initialized = true;
	return E_OK;
}

ER line_adc_reader_start_task(const line_adc_config_t *reader_config,
			      const line_barcode_task_config_t *task_config)
{
	line_barcode_task_config_t task;
	ER er;

	if (!valid_config(reader_config) || task_config == NULL) return E_PAR;
	if (s_task_start_attempted) return E_OBJ;
	s_task_reader_config = *reader_config;
	task = *task_config;
	task.initialize = initialize_task_reader;
	task.read = line_adc_reader_read;
	task.read_context = &s_task_reader;
	er = line_barcode_task_start(&task);
	if (er == E_OK) s_task_start_attempted = true;
	return er;
}

ER line_adc_reader_start_default_task(uint32_t max_sample_span_ms,
				      const line_barcode_task_config_t *task_config)
{
	line_adc_config_t reader_config;
	ER er = line_adc_reader_default_config(&reader_config, max_sample_span_ms);

	if (er != E_OK) return er;
	return line_adc_reader_start_task(&reader_config, task_config);
}

static ER read_channel(line_adc_reader_t *reader, const line_adc_channel_t *channel,
		       ir_reading_t *reading)
{
	UW sum = 0U;
	unsigned i;

	for (i = 0U; i < LINE_ADC_OVERSAMPLES; i++) {
		SZ actual = 0;
		ER er;
		reader->pending_value = 0U;
		er = tk_srea_dev(reader->device, channel->channel,
				 &reader->pending_value, 1, &actual);
		if (er != E_OK) {
			tm_printf((UB*)"line ADC read failed: channel=%d error=%d\n",
				  (int)channel->channel, (int)er);
			return er < E_OK ? er : E_IO;
		}
		if (actual != 1 || reader->pending_value > LINE_ADC_MAX_VALUE) return E_IO;
		sum += reader->pending_value;
	}
	reading->raw = (uint16_t)(sum / LINE_ADC_OVERSAMPLES);
	reading->black = is_black(channel, reading->raw);
	return E_OK;
}

static ER read_sample(line_adc_reader_t *reader, ir_readings_t *readings,
		      uint32_t *timestamp_ms, bool line, bool barcode)
{
	ir_readings_t result = {0};
	SYSTIM start, barcode_start, end;
	uint64_t start_ms, barcode_ms, end_ms;
	ER er;

	if (reader == NULL || readings == NULL || timestamp_ms == NULL) return E_PAR;
	if (!reader->initialized || reader->faulted) return E_OBJ;
	er = tk_get_otm(&start);
	if (er != E_OK) goto fault;
	barcode_start = start;
	if (line) {
		er = read_channel(reader, &reader->config.center, &result.center);
		if (er != E_OK) goto fault;
		er = read_channel(reader, &reader->config.right, &result.right);
		if (er != E_OK) goto fault;
	}
	if (barcode) {
		if (line) {
			er = tk_get_otm(&barcode_start);
			if (er != E_OK) goto fault;
		}
		er = read_channel(reader, &reader->config.barcode, &result.barcode);
		if (er != E_OK) goto fault;
	}
	er = tk_get_otm(&end);
	if (er != E_OK) goto fault;
	start_ms = clock_ms(&start);
	barcode_ms = clock_ms(&barcode_start);
	end_ms = clock_ms(&end);
	if (barcode_ms < start_ms || end_ms < barcode_ms) {
		tm_printf((UB*)"line ADC sampling clock moved backward\n");
		er = E_TMOUT;
		goto fault;
	}
	if (end_ms - start_ms > reader->config.max_sample_span_ms) {
		tm_printf((UB*)"line ADC sample span exceeded budget: limit=%u ms\n",
			  (unsigned)reader->config.max_sample_span_ms);
		er = E_TMOUT;
		goto fault;
	}
	*readings = result;
	*timestamp_ms = barcode_start.lo;
	return E_OK;

fault:
	reader->faulted = true;
	return er < E_OK ? er : E_IO;
}

ER line_adc_reader_read(void *context, ir_readings_t *readings,
			uint32_t *timestamp_ms)
{
	return read_sample(context, readings, timestamp_ms, true, true);
}

ER line_adc_reader_read_line(line_adc_reader_t *reader, ir_readings_t *readings,
			     uint32_t *timestamp_ms)
{
	return read_sample(reader, readings, timestamp_ms, true, false);
}

ER line_adc_reader_read_barcode(line_adc_reader_t *reader, ir_readings_t *readings,
				uint32_t *timestamp_ms)
{
	return read_sample(reader, readings, timestamp_ms, false, true);
}

ER line_adc_reader_close(line_adc_reader_t *reader)
{
	ER er;

	if (reader == NULL) return E_PAR;
	if (!reader->initialized || reader->faulted) return E_OBJ;
	er = tk_cls_dev(reader->device, 0U);
	if (er != E_OK) return er < E_OK ? er : E_IO;
	reader->initialized = false;
	reader->device = 0;
	return E_OK;
}

#if APP_TEST
#include "../../test_report.h"

BOOL line_adc_reader_test(void)
{
	line_adc_config_t config = {{2U, 1000U, true}, {1U, 2000U, false},
				    {0U, 3000U, true}, 50U};
	line_adc_config_t imported = {0};
	BOOL config_ok;
	BOOL threshold_ok;
	BOOL imported_ok;
	TEST_INIT();

	config_ok = valid_config(&config);
	config.right.channel = 1U;
	config_ok = config_ok && !valid_config(&config);
	config.right.channel = 3U;
	config_ok = config_ok && !valid_config(&config);
	config.right.channel = 0U;
	config.right.threshold = 4096U;
	config_ok = config_ok && !valid_config(&config);
	config.right.threshold = 3000U;
	config.max_sample_span_ms = 0U;
	config_ok = config_ok && !valid_config(&config) && !valid_config(NULL);
	threshold_ok = is_black(&config.barcode, 1001U) &&
		       !is_black(&config.barcode, 1000U) &&
		       is_black(&config.center, 1999U) &&
		       !is_black(&config.center, 2000U);
	imported_ok = line_adc_reader_default_config(&imported, 25U) == E_OK &&
		      imported.barcode.channel == 2U &&
		      imported.barcode.threshold == 1000U &&
		      imported.barcode.black_above &&
		      imported.center.channel == 1U &&
		      imported.center.threshold == 1000U &&
		      imported.center.black_above &&
		      imported.right.channel == 0U &&
		      imported.right.threshold == 1000U &&
		      imported.right.black_above &&
		      imported.max_sample_span_ms == 25U;
	imported_ok = imported_ok &&
		      line_adc_reader_default_config(&imported, 0U) == E_PAR &&
		      imported.max_sample_span_ms == 25U;

	TEST_CHECK("ADC-U01", config_ok, "channel ownership and sample limits");
	TEST_CHECK("ADC-U02", threshold_ok, "per-sensor black polarity");
	TEST_CHECK("ADC-U03", imported_ok, "original ADC channel and threshold migration");
	TEST_SUMMARY("line ADC reader");
	return config_ok && threshold_ok && imported_ok;
}
#endif
