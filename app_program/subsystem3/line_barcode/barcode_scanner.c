#include "barcode_scanner.h"

#include <string.h>

#define CODE39_PATTERN_COUNT        44U
#define CODE39_STAR_INDEX           43U   /* '*' start/stop is the 44th pattern */
#define CODE39_WIDE_ELEMENTS        3U    /* every Code 39 character has 3 wide */
#define DEFAULT_WIDE_RATIO_PERCENT  130U  /* narrowest wide vs widest narrow    */
#define DEFAULT_MAX_GAP_MODULES     4U    /* max inter-character gap            */

static const char code39_characters[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static const char *const code39_patterns[CODE39_PATTERN_COUNT] =
{
    "nnnwwnwnn", "wnnwnnnnw", "nnwwnnnnw", "wnwwnnnnn",
    "nnnwwnnnw", "wnnwwnnnn", "nnwwwnnnn", "nnnwnnwnw",
    "wnnwnnwnn", "nnwwnnwnn", "wnnnnwnnw", "nnwnnwnnw",
    "wnwnnwnnn", "nnnnwwnnw", "wnnnwwnnn", "nnwnwwnnn",
    "nnnnnwwnw", "wnnnnwwnn", "nnwnnwwnn", "nnnnwwwnn",
    "wnnnnnnww", "nnwnnnnww", "wnwnnnnwn", "nnnnwnnww",
    "wnnnwnnwn", "nnwnwnnwn", "nnnnnnwwn", "wnnnnnwwn",
    "nnwnnnwwn", "nnnnwnwwn", "wwnnnnnnw", "nwwnnnnnw",
    "wwwnnnnnn", "nwnnwnnnw", "wwnnwnnnn", "nwwnwnnnn",
    "nwnnnnwnw", "wwnnnnwnn", "nwwnnnwnn", "nwnwnwnnn",
    "nwnwnnnwn", "nwnnnwnwn", "nnnwnwnwn",
    "nwnnwnwnn"   /* '*' */
};

/* Must match the order of barcode_reject_t. */
static const char *const reject_names[BARCODE_REJECT_COUNT] =
{
    "OK", "GLITCH", "GAP", "DIRECTION", "SEPARATION",
    "NO_MATCH", "WRONG_ORIENTATION", "OVERFLOW"
};

const char *barcode_reject_name(barcode_reject_t reject)
{
    return ((unsigned)reject < (unsigned)BARCODE_REJECT_COUNT) ?
           reject_names[reject] : "?";
}

/* ---------------------------------------------------------------- config */

static bool config_valid(const barcode_scanner_config_t *config)
{
    return (config != NULL) &&
           (config->counts_per_module != 0U) &&
           (config->module_tolerance_percent <= 50U) &&
           ((config->wide_ratio_percent == 0U) ||
            (config->wide_ratio_percent >= 100U));
}

/*
 * Runs shorter than this are treated as noise. By default the limit is half
 * a module, scaled down by the tolerance (tolerance 0 -> 0.5 module,
 * tolerance 50 -> 0.25 module).
 */
uint32_t barcode_scanner_min_run_counts(const barcode_scanner_t *scanner)
{
    uint64_t limit;

    if (scanner->config.min_run_counts != 0U)
    {
        return scanner->config.min_run_counts;
    }
    limit = ((uint64_t)scanner->config.counts_per_module *
             (100U - scanner->config.module_tolerance_percent)) / 200U;
    return (limit < 1U) ? 1U : (uint32_t)limit;
}

uint32_t barcode_scanner_max_gap_counts(const barcode_scanner_t *scanner)
{
    uint64_t limit;

    if (scanner->config.max_gap_counts != 0U)
    {
        return scanner->config.max_gap_counts;
    }
    limit = (uint64_t)scanner->config.counts_per_module *
            DEFAULT_MAX_GAP_MODULES;
    return (limit > UINT32_MAX) ? UINT32_MAX : (uint32_t)limit;
}

uint8_t barcode_scanner_wide_ratio_percent(const barcode_scanner_t *scanner)
{
    return (scanner->config.wide_ratio_percent != 0U) ?
           scanner->config.wide_ratio_percent :
           (uint8_t)DEFAULT_WIDE_RATIO_PERCENT;
}

bool barcode_scanner_set_config(
    barcode_scanner_t *scanner,
    const barcode_scanner_config_t *config)
{
    if ((scanner == NULL) || !scanner->initialized || !config_valid(config))
    {
        return false;
    }
    scanner->config = *config;
    return true;
}

/* ------------------------------------------------------------ arithmetic */

/* Wrap-safe difference of two free-running 32-bit encoder counters. */
static int32_t signed_count_delta(int32_t from, int32_t to)
{
    return (int32_t)((uint32_t)to - (uint32_t)from);
}

static uint32_t absolute_value(int32_t value)
{
    return (value < 0) ? (uint32_t)(0U - (uint32_t)value) : (uint32_t)value;
}

static void add_sample(barcode_sample_stats_t *stats, uint32_t value)
{
    if (stats->count == 0U)
    {
        stats->min = value;
        stats->max = value;
    }
    else
    {
        if (value < stats->min)
        {
            stats->min = value;
        }
        if (value > stats->max)
        {
            stats->max = value;
        }
    }
    stats->sum += value;
    stats->count++;
}

/* ---------------------------------------------------------------- events */

static void init_info(const barcode_scanner_t *scanner, barcode_char_info_t *info)
{
    memset(info, 0, sizeof(*info));
    info->run_count = scanner->run_count;
    memcpy(info->run_counts, scanner->run_counts, sizeof(info->run_counts));
    info->gap_counts = scanner->gap_counts;
}

static void publish_event(barcode_scanner_t *scanner, barcode_char_info_t *info)
{
    if (info->direction == 0)
    {
        info->direction = scanner->scan_direction;
    }
    info->in_frame = scanner->in_frame;

    scanner->stats.rejects[info->reject]++;
    if (scanner->event_pending)
    {
        scanner->stats.events_dropped++;
    }
    scanner->event = *info;
    scanner->event_pending = true;
}

/* -------------------------------------------------------------- decoding */

static void reset_character(barcode_scanner_t *scanner)
{
    memset(scanner->run_counts, 0, sizeof(scanner->run_counts));
    scanner->run_count = 0U;
}

/*
 * Drop the frame in progress, returning whether there was one. A finished
 * result that has not been collected yet keeps its payload and length.
 */
static bool abort_frame(barcode_scanner_t *scanner)
{
    bool was_in_frame = scanner->in_frame;

    scanner->in_frame = false;
    if (!scanner->result_ready)
    {
        scanner->payload_length = 0U;
    }
    return was_in_frame;
}

static int find_pattern(const char *pattern)
{
    uint8_t j;

    for (j = 0U; j < CODE39_PATTERN_COUNT; j++)
    {
        if (memcmp(pattern, code39_patterns[j], BARCODE_ELEMENTS) == 0)
        {
            return (int)j;
        }
    }
    return -1;
}

static char symbol_for_index(int index)
{
    return (index == (int)CODE39_STAR_INDEX) ? '*' : code39_characters[index];
}

/*
 * Every Code 39 character has exactly 3 wide elements out of 9, so the 3
 * widest runs are marked wide. This needs no speed or module calibration.
 * The character is rejected if the wide/narrow split is not clearly
 * separated. Fills the pattern, symbol and width fields of info.
 */
static barcode_reject_t classify_character(
    const barcode_scanner_t *scanner,
    bool reverse,
    barcode_char_info_t *info)
{
    bool is_wide[BARCODE_ELEMENTS] = { false };
    char reversed[BARCODE_PATTERN_LENGTH];
    uint32_t narrow_max = 0U;
    uint32_t wide_min = UINT32_MAX;
    int index;
    int alt_index;
    uint8_t i;
    uint8_t k;

    for (k = 0U; k < CODE39_WIDE_ELEMENTS; k++)
    {
        uint8_t best = BARCODE_ELEMENTS;
        for (i = 0U; i < BARCODE_ELEMENTS; i++)
        {
            if (!is_wide[i] &&
                ((best == BARCODE_ELEMENTS) ||
                 (scanner->run_counts[i] > scanner->run_counts[best])))
            {
                best = i;
            }
        }
        is_wide[best] = true;
    }

    for (i = 0U; i < BARCODE_ELEMENTS; i++)
    {
        uint32_t count = scanner->run_counts[i];

        if (is_wide[i])
        {
            if (count < wide_min)
            {
                wide_min = count;
            }
        }
        else if (count > narrow_max)
        {
            narrow_max = count;
        }
        info->raw_pattern[i] = is_wide[i] ? 'w' : 'n';
        reversed[i] = is_wide[BARCODE_ELEMENTS - 1U - i] ? 'w' : 'n';
    }
    info->raw_pattern[BARCODE_ELEMENTS] = '\0';
    reversed[BARCODE_ELEMENTS] = '\0';
    memcpy(info->pattern, reverse ? reversed : info->raw_pattern,
           BARCODE_PATTERN_LENGTH);
    info->narrow_max = narrow_max;
    info->wide_min = wide_min;

    if ((uint64_t)wide_min * 100U <
        (uint64_t)narrow_max * barcode_scanner_wide_ratio_percent(scanner))
    {
        return BARCODE_REJECT_SEPARATION;
    }

    index = find_pattern(info->pattern);
    alt_index = find_pattern(reverse ? info->raw_pattern : reversed);
    if (alt_index >= 0)
    {
        info->alt_symbol = symbol_for_index(alt_index);
    }
    if (index >= 0)
    {
        info->symbol = symbol_for_index(index);
        return BARCODE_REJECT_NONE;
    }
    return (alt_index >= 0) ? BARCODE_REJECT_WRONG_ORIENTATION :
                              BARCODE_REJECT_NO_MATCH;
}

/* Returns false if the payload buffer overflowed (the frame is dropped). */
static bool accept_character(barcode_scanner_t *scanner, char character)
{
    if (character == '*')
    {
        if (!scanner->in_frame)
        {
            /* Do not start a new frame while a result is waiting to be taken. */
            if (!scanner->result_ready)
            {
                scanner->in_frame = true;
                scanner->payload_length = 0U;
            }
        }
        else if (scanner->payload_length > 0U)
        {
            if (scanner->scan_direction < 0)
            {
                uint8_t i;
                uint8_t half_length = scanner->payload_length / 2U;

                for (i = 0U; i < half_length; i++)
                {
                    uint8_t opposite = (uint8_t)(
                        scanner->payload_length - 1U - i);
                    uint8_t value = scanner->payload[i];
                    scanner->payload[i] = scanner->payload[opposite];
                    scanner->payload[opposite] = value;
                }
            }
            scanner->result_ready = true;
            scanner->in_frame = false;
            scanner->stats.frames_ok++;
        }
        return true;
    }

    if (scanner->in_frame)
    {
        if (scanner->payload_length < BARCODE_MAX_PAYLOAD)
        {
            scanner->payload[scanner->payload_length++] = (uint8_t)character;
        }
        else
        {
            (void)abort_frame(scanner);
            return false;
        }
    }
    return true;
}

static void update_ok_stats(
    barcode_scanner_t *scanner,
    const barcode_char_info_t *info,
    bool was_in_frame)
{
    uint8_t i;

    for (i = 0U; i < BARCODE_ELEMENTS; i++)
    {
        add_sample((info->raw_pattern[i] == 'w') ? &scanner->stats.wide :
                                                   &scanner->stats.narrow,
                   info->run_counts[i]);
    }
    /* The gap before the start '*' is the quiet zone, not a character gap. */
    if (was_in_frame && (info->gap_counts > scanner->stats.gap_ok_max))
    {
        scanner->stats.gap_ok_max = info->gap_counts;
    }
}

static void finish_character(barcode_scanner_t *scanner)
{
    barcode_char_info_t info;
    barcode_reject_t reject;
    bool was_in_frame = scanner->in_frame;

    init_info(scanner, &info);
    reject = classify_character(scanner, scanner->scan_direction < 0, &info);

    if (reject == BARCODE_REJECT_NONE)
    {
        update_ok_stats(scanner, &info, was_in_frame);
        if (!accept_character(scanner, info.symbol))
        {
            reject = BARCODE_REJECT_OVERFLOW;
            info.aborted_frame = true;
        }
    }
    else
    {
        info.aborted_frame = abort_frame(scanner);
    }

    info.reject = reject;
    publish_event(scanner, &info);
    reset_character(scanner);
}

/* White-to-black edge: a character starts here. Inside a frame, gap_counts
 * is the white gap since the previous character. */
static void start_character(
    barcode_scanner_t *scanner,
    int32_t count_delta,
    uint32_t gap_counts)
{
    int8_t direction = (count_delta < 0) ? -1 : 1;

    scanner->gap_counts = gap_counts;

    if (scanner->in_frame)
    {
        barcode_reject_t reject = BARCODE_REJECT_NONE;

        add_sample(&scanner->stats.gap, gap_counts);
        if (direction != scanner->scan_direction)
        {
            reject = BARCODE_REJECT_DIRECTION;
        }
        else if (gap_counts > barcode_scanner_max_gap_counts(scanner))
        {
            reject = BARCODE_REJECT_GAP;
        }

        if (reject != BARCODE_REJECT_NONE)
        {
            barcode_char_info_t info;

            init_info(scanner, &info);
            info.reject = reject;
            info.direction = direction;
            info.offending_counts = gap_counts;
            info.aborted_frame = abort_frame(scanner);
            publish_event(scanner, &info);
        }
    }

    if (!scanner->in_frame)
    {
        scanner->scan_direction = direction;
    }
    scanner->collecting = true;
    reset_character(scanner);
}

static void reject_run(barcode_scanner_t *scanner, barcode_reject_t reject,
                       uint32_t width_counts)
{
    barcode_char_info_t info;

    init_info(scanner, &info);
    info.reject = reject;
    if (reject == BARCODE_REJECT_DIRECTION)
    {
        info.direction = (int8_t)-scanner->scan_direction;
    }
    info.offending_counts = width_counts;
    info.aborted_frame = abort_frame(scanner);
    publish_event(scanner, &info);

    reset_character(scanner);
    scanner->collecting = false;
}

/* ------------------------------------------------------------ public API */

bool barcode_scanner_init(
    barcode_scanner_t *scanner,
    const barcode_scanner_config_t *config,
    bool initial_black,
    int32_t initial_encoder_count)
{
    if ((scanner == NULL) || !config_valid(config))
    {
        return false;
    }

    memset(scanner, 0, sizeof(*scanner));
    scanner->config = *config;
    scanner->initialized = true;
    scanner->last_black = initial_black;
    scanner->last_encoder_count = initial_encoder_count;
    /*
     * If we start on black, we are inside a bar of unknown length, so wait
     * for the next white-to-black edge instead of collecting a partial run.
     */
    scanner->collecting = false;
    scanner->scan_direction = 0;
    return true;
}

bool barcode_scanner_update(
    barcode_scanner_t *scanner,
    bool black,
    int32_t encoder_count)
{
    int32_t count_delta;
    uint32_t width_counts;

    if ((scanner == NULL) || !scanner->initialized)
    {
        return false;
    }
    if (black == scanner->last_black)
    {
        return true;
    }

    count_delta = signed_count_delta(scanner->last_encoder_count,
                                     encoder_count);
    width_counts = absolute_value(count_delta);

    scanner->edge.sequence++;
    scanner->edge.to_black = black;
    scanner->edge.delta = count_delta;
    scanner->edge.width = width_counts;

    if (!scanner->collecting)
    {
        if (black)
        {
            start_character(scanner, count_delta, width_counts);
        }
    }
    else if (width_counts < barcode_scanner_min_run_counts(scanner))
    {
        reject_run(scanner, BARCODE_REJECT_GLITCH, width_counts);
    }
    else if (((count_delta < 0) ? -1 : 1) != scanner->scan_direction)
    {
        reject_run(scanner, BARCODE_REJECT_DIRECTION, width_counts);
    }
    else
    {
        scanner->run_counts[scanner->run_count++] = width_counts;
        if (scanner->run_count == BARCODE_ELEMENTS)
        {
            /* The 9th element is a bar, so this is a black-to-white edge. */
            finish_character(scanner);
            scanner->collecting = false;
        }
    }

    scanner->last_black = black;
    scanner->last_encoder_count = encoder_count;
    return true;
}

bool barcode_scanner_take_result(
    barcode_scanner_t *scanner,
    uint8_t *payload,
    uint8_t payload_capacity,
    uint8_t *length)
{
    if ((scanner == NULL) || (payload == NULL) || (length == NULL) ||
        !scanner->result_ready ||
        (payload_capacity < scanner->payload_length))
    {
        return false;
    }

    memcpy(payload, scanner->payload, scanner->payload_length);
    *length = scanner->payload_length;
    scanner->result_ready = false;
    return true;
}

void barcode_scanner_reset(barcode_scanner_t *scanner)
{
    if (scanner == NULL)
    {
        return;
    }

    reset_character(scanner);
    scanner->collecting = false;
    scanner->in_frame = false;
    scanner->scan_direction = 0;
    scanner->payload_length = 0U;
    scanner->result_ready = false;
    scanner->gap_counts = 0U;
}

bool barcode_scanner_get_edge(
    const barcode_scanner_t *scanner,
    barcode_edge_t *edge)
{
    if ((scanner == NULL) || (edge == NULL) || (scanner->edge.sequence == 0U))
    {
        return false;
    }
    *edge = scanner->edge;
    return true;
}

bool barcode_scanner_take_event(
    barcode_scanner_t *scanner,
    barcode_char_info_t *info)
{
    if ((scanner == NULL) || (info == NULL) || !scanner->event_pending)
    {
        return false;
    }
    *info = scanner->event;
    scanner->event_pending = false;
    return true;
}

const barcode_stats_t *barcode_scanner_get_stats(
    const barcode_scanner_t *scanner)
{
    return (scanner != NULL) ? &scanner->stats : NULL;
}

void barcode_scanner_reset_stats(barcode_scanner_t *scanner)
{
    if (scanner != NULL)
    {
        memset(&scanner->stats, 0, sizeof(scanner->stats));
    }
}

bool barcode_scanner_suggest_config(
    const barcode_scanner_t *scanner,
    barcode_scanner_config_t *suggested)
{
    const barcode_stats_t *stats;
    uint32_t module;

    if ((scanner == NULL) || (suggested == NULL) || !scanner->initialized ||
        (scanner->stats.narrow.count == 0U))
    {
        return false;
    }

    stats = &scanner->stats;
    *suggested = scanner->config;

    module = (uint32_t)(stats->narrow.sum / stats->narrow.count);
    suggested->counts_per_module = (module < 1U) ? 1U : module;

    suggested->min_run_counts = stats->narrow.min / 2U;
    if (suggested->min_run_counts < 1U)
    {
        suggested->min_run_counts = 1U;
    }

    if (stats->gap_ok_max != 0U)
    {
        suggested->max_gap_counts = stats->gap_ok_max + (stats->gap_ok_max / 2U);
    }
    return true;
}