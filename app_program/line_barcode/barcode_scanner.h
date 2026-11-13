#ifndef BARCODE_SCANNER_H
#define BARCODE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#define BARCODE_MAX_PAYLOAD     (32U)
#define BARCODE_ELEMENTS        (9U)
#define BARCODE_PATTERN_LENGTH  (BARCODE_ELEMENTS + 1U)

typedef enum
{
    BARCODE_REJECT_NONE = 0,
    BARCODE_REJECT_GLITCH,
    BARCODE_REJECT_GAP,
    BARCODE_REJECT_DIRECTION,
    BARCODE_REJECT_SEPARATION,
    BARCODE_REJECT_NO_MATCH,
    BARCODE_REJECT_WRONG_ORIENTATION,
    BARCODE_REJECT_OVERFLOW,
    BARCODE_REJECT_COUNT
} barcode_reject_t;

/* counts_per_module must be positive; tolerance must be 0..50. The last
 * three fields use derived defaults when zero; a nonzero ratio must be >=100. */
typedef struct
{
    uint32_t counts_per_module;
    uint8_t module_tolerance_percent;
    uint32_t min_run_counts;
    uint32_t max_gap_counts;
    uint8_t wide_ratio_percent;
} barcode_scanner_config_t;

typedef struct
{
    uint32_t sequence;
    bool to_black;
    int32_t delta;
    uint32_t width;
} barcode_edge_t;

typedef struct
{
    barcode_reject_t reject;
    char symbol;
    char alt_symbol;
    int8_t direction;
    bool in_frame;
    bool aborted_frame;
    uint8_t run_count;
    uint32_t run_counts[BARCODE_ELEMENTS];
    char raw_pattern[BARCODE_PATTERN_LENGTH];
    char pattern[BARCODE_PATTERN_LENGTH];
    uint32_t gap_counts;
    uint32_t offending_counts;
    uint32_t narrow_max;
    uint32_t wide_min;
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
    uint32_t rejects[BARCODE_REJECT_COUNT];
    uint32_t frames_ok;
    uint32_t events_dropped;
    barcode_sample_stats_t narrow;
    barcode_sample_stats_t wide;
    barcode_sample_stats_t gap;
    uint32_t gap_ok_max;
} barcode_stats_t;

/* Owned by the line/barcode task; do not share this mutable state. */
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

/* Feed colour samples with a signed, free-running 32-bit encoder count.
 * Direction changes are detectable only when reflected between colour edges. */
bool barcode_scanner_init(barcode_scanner_t *scanner,
                          const barcode_scanner_config_t *config,
                          bool initial_black, int32_t initial_encoder_count);
bool barcode_scanner_update(barcode_scanner_t *scanner, bool black,
                            int32_t encoder_count);
/* An undersized destination leaves the completed result available. */
bool barcode_scanner_take_result(barcode_scanner_t *scanner, uint8_t *payload,
                                 uint8_t payload_capacity, uint8_t *length);
/* Reset decoding state without clearing config or accumulated statistics. */
void barcode_scanner_reset(barcode_scanner_t *scanner);
bool barcode_scanner_set_config(barcode_scanner_t *scanner,
                                const barcode_scanner_config_t *config);
uint32_t barcode_scanner_min_run_counts(const barcode_scanner_t *scanner);
uint32_t barcode_scanner_max_gap_counts(const barcode_scanner_t *scanner);
uint8_t barcode_scanner_wide_ratio_percent(const barcode_scanner_t *scanner);
bool barcode_scanner_get_edge(const barcode_scanner_t *scanner,
                              barcode_edge_t *edge);
bool barcode_scanner_take_event(barcode_scanner_t *scanner,
                                barcode_char_info_t *info);
const barcode_stats_t *barcode_scanner_get_stats(const barcode_scanner_t *scanner);
void barcode_scanner_reset_stats(barcode_scanner_t *scanner);
bool barcode_scanner_suggest_config(const barcode_scanner_t *scanner,
                                    barcode_scanner_config_t *suggested);
const char *barcode_reject_name(barcode_reject_t reject);

#endif
