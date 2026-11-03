#include "buddy3.h"
#include "hardware/adc.h"

static const uint8_t s_ir_gpio[NUM_SENSORS] =
{
    IR_BARCODE_GPIO,
    IR_CENTER_GPIO,
    IR_RIGHT_GPIO
};

static const uint16_t s_ir_threshold[NUM_SENSORS] =
{
    IR_BARCODE_THRESHOLD,
    IR_CENTER_THRESHOLD,
    IR_RIGHT_THRESHOLD
};

static uint16_t ir_read_raw(ir_index_t sensor)
{
    uint32_t sum = 0U;
    uint8_t sample;

    adc_select_input((uint)(s_ir_gpio[sensor] - ADC_GPIO_BASE));
    for (sample = 0U; sample < OVERSAMPLE_COUNT; sample++)
    {
        sum += adc_read();
    }

    return (uint16_t)(sum / OVERSAMPLE_COUNT);
}

static bool ir_is_black(ir_index_t sensor, uint16_t raw)
{
    return raw > s_ir_threshold[sensor];
}


void line_following_init(void)
{
    uint8_t sensor;

    adc_init();
    for (sensor = 0U; sensor < NUM_SENSORS; sensor++)
    {
        adc_gpio_init(s_ir_gpio[sensor]);
    }
}

bool line_following_read(ir_readings_t *readings)
{
    uint16_t raw[NUM_SENSORS];
    uint8_t sensor;

    if (readings == NULL)
    {
        return false;
    }

    for (sensor = 0U; sensor < NUM_SENSORS; sensor++)
    {
        raw[sensor] = ir_read_raw((ir_index_t)sensor);
    }

    readings->barcode.raw = raw[IR_BARCODE];
    readings->barcode.black = ir_is_black(IR_BARCODE, raw[IR_BARCODE]);
    readings->center.raw = raw[IR_CENTER];
    readings->center.black = ir_is_black(IR_CENTER, raw[IR_CENTER]);
    readings->right.raw = raw[IR_RIGHT];
    readings->right.black = ir_is_black(IR_RIGHT, raw[IR_RIGHT]);

    return true;
}

uint8_t line_following_position(const ir_readings_t *readings)
{
    if (readings == NULL)
    {
        return POS_LOST;
    }

    if (readings->center.black && readings->right.black)
    {
        return POS_JUNCTION;
    }

    if (readings->center.black)
    {
        return POS_LEFT;
    }

    if (readings->right.black)
    {
        return POS_RIGHT;
    }

    return POS_LOST;
}