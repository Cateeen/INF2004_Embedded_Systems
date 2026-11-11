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
#define IR_BARCODE_THRESHOLD    (1000U)
#define IR_CENTER_THRESHOLD     (1000U)
#define IR_RIGHT_THRESHOLD      (1000U)

#define OVERSAMPLE_COUNT        (8U)

/* Line position: 0 = junction, 1 = to the left, 2 = to the right, 3 = line lost */
#define POS_JUNCTION            (0)
#define POS_LEFT                (1)
#define POS_RIGHT               (2)
#define POS_LOST                (3)

/* ------------------------------------------------------------------ */
/* Line following                                                      */
/* ------------------------------------------------------------------ */

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

void line_following_init(void);
bool line_following_read(ir_readings_t *readings);
uint8_t line_following_position(const ir_readings_t *readings);

/* ------------------------------------------------------------------ */
/* Code 39 barcode scanner                                             */
/*                                                                     */
/* Driven by an IR black/white signal and an encoder count. Besides    */
/* decoding, the scanner records what it saw (edges, per-character     */
/* events, running statistics) so a test program can print calibration */
/* data without re-implementing any decoding logic.                    */
/* ------------------------------------------------------------------ */

#define BARCODE_MAX_PAYLOAD     (32U)
#define BARCODE_ELEMENTS        (9U)
#define BARCODE_PATTERN_LENGTH  (BARCODE_ELEMENTS + 1U)

/* Why a character (or the frame around it) was rejected. NONE = decoded OK. */
typedef enum
{
    BARCODE_REJECT_NONE = 0,
    BARCODE_REJECT_GLITCH,             /* a run was shorter than min_run     */
    BARCODE_REJECT_GAP,                /* gap between characters too long    */
    BARCODE_REJECT_DIRECTION,          /* travel direction changed mid-frame */
    BARCODE_REJECT_SEPARATION,         /* wide/narrow runs not clearly apart */
    BARCODE_REJECT_NO_MATCH,           /* pattern is not a Code 39 character */
    BARCODE_REJECT_WRONG_ORIENTATION,  /* only matches when read the other way */
    BARCODE_REJECT_OVERFLOW,           /* payload longer than the buffer     */
    BARCODE_REJECT_COUNT
} barcode_reject_t;

/*
 * Tunable settings. Fields set to 0 use a default derived from
 * counts_per_module, so a config with only the first two fields still works.
 */
typedef struct
{
    uint32_t counts_per_module;        /* nominal narrow width, must be > 0   */
    uint8_t  module_tolerance_percent; /* 0..50, shrinks the default min run  */
    uint32_t min_run_counts;           /* 0 = counts_per_module*(100-tol)/200 */
    uint32_t max_gap_counts;           /* 0 = 4 * counts_per_module           */
    uint8_t  wide_ratio_percent;       /* 0 = 130, must otherwise be >= 100   */
} barcode_scanner_config_t;

/* The most recent colour change. */
typedef struct
{
    uint32_t sequence;   /* increments on every edge; 0 = no edge yet */
    bool     to_black;
    int32_t  delta;      /* signed encoder change since the previous edge */
    uint32_t width;      /* |delta| */
} barcode_edge_t;

/* What happened when a character finished or a frame was aborted. */
typedef struct
{
    barcode_reject_t reject;
    char     symbol;            /* decoded character ('*' included), 0 if none  */
    char     alt_symbol;        /* what the opposite reading direction decodes to */
    int8_t   direction;         /* +1 forward, -1 reversed                      */
    bool     in_frame;          /* frame state after this event                 */
    bool     aborted_frame;     /* this event dropped a frame in progress       */
    uint8_t  run_count;         /* how many entries of run_counts are valid     */
    uint32_t run_counts[BARCODE_ELEMENTS];
    char     raw_pattern[BARCODE_PATTERN_LENGTH]; /* w/n in travel order        */
    char     pattern[BARCODE_PATTERN_LENGTH];     /* w/n actually looked up     */
    uint32_t gap_counts;        /* white gap before this character              */
    uint32_t offending_counts;  /* the run or gap that caused GLITCH/GAP        */
    uint32_t narrow_max;        /* widest run classified narrow                 */
    uint32_t wide_min;          /* narrowest run classified wide                */
} barcode_char_info_t;

typedef struct
{
    uint32_t count;
    uint32_t min;
    uint32_t max;
    uint64_t sum;
} barcode_sample_stats_t;

typedef struct
{
    uint32_t rejects[BARCODE_REJECT_COUNT]; /* [NONE] = characters decoded OK */
    uint32_t frames_ok;
    uint32_t events_dropped;       /* events overwritten before being taken */
    barcode_sample_stats_t narrow; /* runs of successfully decoded characters */
    barcode_sample_stats_t wide;
    barcode_sample_stats_t gap;    /* every gap seen inside a frame */
    uint32_t gap_ok_max;           /* largest gap before a decoded character */
} barcode_stats_t;

typedef struct
{
    barcode_scanner_config_t config;
    bool initialized;
    bool last_black;
    bool collecting;
    bool in_frame;
    bool result_ready;
    int32_t last_encoder_count;
    int8_t scan_direction;
    uint32_t run_counts[BARCODE_ELEMENTS];
    uint8_t run_count;
    uint8_t payload[BARCODE_MAX_PAYLOAD];
    uint8_t payload_length;
    uint32_t gap_counts;
    barcode_edge_t edge;
    barcode_char_info_t event;
    bool event_pending;
    barcode_stats_t stats;
} barcode_scanner_t;

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
    uint8_t payload_capacity,
    uint8_t *length);

/* Clears decoding state but keeps the config and the statistics. */
void barcode_scanner_reset(barcode_scanner_t *scanner);

/* Calibration support. */
bool barcode_scanner_set_config(
    barcode_scanner_t *scanner,
    const barcode_scanner_config_t *config);
uint32_t barcode_scanner_min_run_counts(const barcode_scanner_t *scanner);
uint32_t barcode_scanner_max_gap_counts(const barcode_scanner_t *scanner);
uint8_t  barcode_scanner_wide_ratio_percent(const barcode_scanner_t *scanner);

bool barcode_scanner_get_edge(
    const barcode_scanner_t *scanner,
    barcode_edge_t *edge);
bool barcode_scanner_take_event(
    barcode_scanner_t *scanner,
    barcode_char_info_t *info);
const barcode_stats_t *barcode_scanner_get_stats(
    const barcode_scanner_t *scanner);
void barcode_scanner_reset_stats(barcode_scanner_t *scanner);

/* Builds a config from what has been observed; false if there is no data. */
bool barcode_scanner_suggest_config(
    const barcode_scanner_t *scanner,
    barcode_scanner_config_t *suggested);

const char *barcode_reject_name(barcode_reject_t reject);

#endif