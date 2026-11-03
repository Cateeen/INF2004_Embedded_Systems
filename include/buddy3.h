#ifndef BUDDY3_H
#define BUDDY3_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

#define IR_BARCODE_GPIO         (28U)   /* ADC2 - EDIT to match wiring */
#define IR_CENTER_GPIO          (27U)   /* ADC1 - EDIT to match wiring */
#define IR_RIGHT_GPIO           (26U)   /* ADC0 - EDIT to match wiring */

#define ADC_GPIO_BASE           (26U)   /* GPIO26 == ADC input 0       */
#define NUM_SENSORS             (3U)
#define ADC_MAX_COUNTS          (4095U)

/* Raw ADC above this = dark surface (black line). Calibrate by
 * reading values on white and on black, then set midpoint here.
 * Flip IR_BLACK_IS_HIGH if your module outputs LOW on black.        */
#define IR_BARCODE_THRESHOLD    (2000U)
#define IR_CENTER_THRESHOLD     (2000U)
#define IR_RIGHT_THRESHOLD      (2000U)

#define OVERSAMPLE_COUNT        (8U)
#define BARCODE_MAX_PAYLOAD     (32U)

/* Line position: 0 = junction, 1 = to the left, 2 = to the right, 3 = line lost */
#define POS_JUNCTION            (0)
#define POS_LEFT                (1)
#define POS_RIGHT               (2)
#define POS_LOST                (3)

typedef enum
{
    IR_BARCODE = 0,
    IR_CENTER,
    IR_RIGHT
} ir_index_t;

typedef struct
{
    uint16_t raw;
    bool black;
} ir_reading_t;

typedef struct
{
    ir_reading_t barcode;
    ir_reading_t center;
    ir_reading_t right;
} ir_readings_t;

typedef struct
{
    uint32_t counts_per_module;
    uint8_t module_tolerance_percent;
} barcode_scanner_config_t;

typedef struct
{
    barcode_scanner_config_t config;
    bool initialized;
    bool last_black;
    int32_t last_encoder_count;
    uint8_t bit_buffer[48U];
    uint16_t bit_count;
    uint8_t payload[BARCODE_MAX_PAYLOAD];
    uint8_t payload_length;
    bool result_ready;
} barcode_scanner_t;

void line_following_init(void);
bool line_following_read(ir_readings_t *readings);
uint8_t line_following_position(const ir_readings_t *readings);

bool barcode_scanner_init(
    barcode_scanner_t *scanner,
    const barcode_scanner_config_t *config,
    bool initial_black,
    int32_t initial_encoder_count);

bool barcode_scanner_update(
    barcode_scanner_t *scanner,
    bool black,
    int32_t encoder_count);

bool barcode_scanner_take_result(
    barcode_scanner_t *scanner,
    uint8_t *payload,
    uint8_t *length);

void barcode_scanner_reset(barcode_scanner_t *scanner);

#endif