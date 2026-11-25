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

uint32_t barcode_scanner_module_counts(const barcode_scanner_t *scanner)
{
    uint64_t module;

    if (!scanner->in_frame || (scanner->frame_narrow_count == 0U))
    {
        return scanner->config.counts_per_module;
    }
    module = scanner->frame_narrow_sum / scanner->frame_narrow_count;
    return (module < 1U) ? 1U : (uint32_t)module;
}

/*
 * The inter-character gap is about one narrow module of the barcode being
 * read, so the default limit scales with the module measured in this frame
 * rather than a fixed calibration: bigger or smaller labels both work.
 */
uint32_t barcode_scanner_max_gap_counts(const barcode_scanner_t *scanner)
{
    uint64_t limit;

    if (scanner->config.max_gap_counts != 0U)
    {
        return scanner->config.max_gap_counts;
    }
    limit = (uint64_t)barcode_scanner_module_counts(scanner) *
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
    info->label_reversed = scanner->label_reversed;

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

const char *barcode_code39_pattern(char symbol)
{
    uint8_t i;

    if (symbol == '*')
    {
        return code39_patterns[CODE39_STAR_INDEX];
    }
    for (i = 0U; i < CODE39_STAR_INDEX; i++)
    {
        if (code39_characters[i] == symbol)
        {
            return code39_patterns[i];
        }
    }
    return NULL;
}

typedef struct
{
    bool is_wide[BARCODE_ELEMENTS];
    uint32_t narrow_max;
    uint32_t wide_min;
    uint64_t score;        /* limiting wide/narrow ratio, in percent */
    bool separated;
    int index;
    int alt_index;
} code39_split_t;

/* Marks the `wide` widest runs among elements first, first + 2, ... */
static void mark_widest(const uint32_t *runs, uint8_t first, uint8_t wide,
                        bool *is_wide)
{
    uint8_t k;
    uint8_t i;

    for (k = 0U; k < wide; k++)
    {
        uint8_t best = BARCODE_ELEMENTS;
        for (i = first; i < BARCODE_ELEMENTS; i += 2U)
        {
            if (!is_wide[i] &&
                ((best == BARCODE_ELEMENTS) || (runs[i] > runs[best])))
            {
                best = i;
            }
        }
        is_wide[best] = true;
    }
}

static void group_limits(const uint32_t *runs, const bool *is_wide,
                         uint8_t first, uint32_t *narrow_min,
                         uint32_t *narrow_max, uint32_t *wide_min)
{
    uint8_t i;

    *narrow_min = UINT32_MAX;
    *narrow_max = 0U;
    *wide_min = UINT32_MAX;
    for (i = first; i < BARCODE_ELEMENTS; i += 2U)
    {
        if (is_wide[i])
        {
            if (runs[i] < *wide_min) *wide_min = runs[i];
        }
        else
        {
            if (runs[i] < *narrow_min) *narrow_min = runs[i];
            if (runs[i] > *narrow_max) *narrow_max = runs[i];
        }
    }
}

static uint64_t ratio_percent(uint32_t wide, uint32_t narrow)
{
    return ((uint64_t)wide * 100U) / ((narrow == 0U) ? 1U : narrow);
}

/*
 * Code 39 characters have either 2 wide bars and 1 wide space, or no wide
 * bars and 3 wide spaces. Bars and spaces are split separately so that a
 * sensor which systematically reads bars thinner (or thicker) than spaces
 * does not mix up the two groups.
 */
static void evaluate_split(const barcode_scanner_t *scanner,
                           uint8_t wide_bars, uint8_t wide_spaces,
                           code39_split_t *split)
{
    const uint32_t *runs = scanner->run_counts;
    uint64_t ratio = barcode_scanner_wide_ratio_percent(scanner);
    uint32_t bar_narrow_min, bar_narrow_max, bar_wide_min;
    uint32_t space_narrow_min, space_narrow_max, space_wide_min;
    uint64_t space_score;
    char pattern[BARCODE_PATTERN_LENGTH];
    char reversed[BARCODE_PATTERN_LENGTH];
    uint8_t i;

    memset(split, 0, sizeof(*split));
    mark_widest(runs, 0U, wide_bars, split->is_wide);
    mark_widest(runs, 1U, wide_spaces, split->is_wide);
    group_limits(runs, split->is_wide, 0U,
                 &bar_narrow_min, &bar_narrow_max, &bar_wide_min);
    group_limits(runs, split->is_wide, 1U,
                 &space_narrow_min, &space_narrow_max, &space_wide_min);
    (void)space_narrow_min;

    space_score = ratio_percent(space_wide_min, space_narrow_max);
    split->narrow_max = space_narrow_max;
    split->wide_min = space_wide_min;
    split->score = space_score;
    split->separated = space_score >= ratio;
    if (wide_bars > 0U)
    {
        uint64_t bar_score = ratio_percent(bar_wide_min, bar_narrow_max);

        split->separated = split->separated && (bar_score >= ratio);
        if (bar_score < space_score)
        {
            split->narrow_max = bar_narrow_max;
            split->wide_min = bar_wide_min;
            split->score = bar_score;
        }
    }
    else
    {
        /* With no wide bars, every bar must look narrow. */
        split->separated = split->separated &&
            (ratio_percent(bar_narrow_max, bar_narrow_min) < ratio);
    }

    for (i = 0U; i < BARCODE_ELEMENTS; i++)
    {
        pattern[i] = split->is_wide[i] ? 'w' : 'n';
        reversed[i] = split->is_wide[BARCODE_ELEMENTS - 1U - i] ? 'w' : 'n';
    }
    split->index = find_pattern(pattern);
    split->alt_index = find_pattern(reversed);
}

/*
 * Needs no speed or module calibration: each character is classified from
 * the relative widths of its own bars and spaces. The character is rejected
 * if the wide/narrow split is not clearly separated. Fills the pattern,
 * symbol and width fields of info.
 */
static barcode_reject_t classify_character(
    const barcode_scanner_t *scanner,
    bool reverse,
    barcode_char_info_t *info)
{
    code39_split_t splits[2];
    const code39_split_t *chosen = NULL;
    int index;
    int alt_index;
    uint8_t s;
    uint8_t i;

    evaluate_split(scanner, 2U, 1U, &splits[0]);
    evaluate_split(scanner, 0U, CODE39_WIDE_ELEMENTS, &splits[1]);

    /* Prefer a separated split that decodes, then any separated split,
     * then the best-separated one for diagnostics. */
    for (s = 0U; s < 2U; s++)
    {
        const code39_split_t *split = &splits[s];
        bool decodes = (split->index >= 0) || (split->alt_index >= 0);

        if (!split->separated || !decodes) continue;
        if ((chosen == NULL) || (split->score > chosen->score)) chosen = split;
    }
    for (s = 0U; (chosen == NULL) && (s < 2U); s++)
    {
        if (splits[s].separated) chosen = &splits[s];
    }
    if (chosen == NULL)
    {
        chosen = (splits[1].score > splits[0].score) ? &splits[1] : &splits[0];
    }

    for (i = 0U; i < BARCODE_ELEMENTS; i++)
    {
        info->raw_pattern[i] = chosen->is_wide[i] ? 'w' : 'n';
        info->pattern[i] = chosen->is_wide[reverse ?
            (BARCODE_ELEMENTS - 1U - i) : i] ? 'w' : 'n';
    }
    info->raw_pattern[BARCODE_ELEMENTS] = '\0';
    info->pattern[BARCODE_ELEMENTS] = '\0';
    info->narrow_max = chosen->narrow_max;
    info->wide_min = chosen->wide_min;

    if (!chosen->separated)
    {
        return BARCODE_REJECT_SEPARATION;
    }

    index = reverse ? chosen->alt_index : chosen->index;
    alt_index = reverse ? chosen->index : chosen->alt_index;
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

/*
 * The bars arrive mirrored when the robot moves backwards or the label
 * faces the other way; both together cancel out.
 */
static bool reading_reversed(const barcode_scanner_t *scanner)
{
    return (scanner->scan_direction < 0) != scanner->label_reversed;
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
            if (reading_reversed(scanner))
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

/* Learn the barcode's module size from every accepted character in a frame
 * (starting with the start '*'), restarting when a new frame begins. */
static void update_frame_module(
    barcode_scanner_t *scanner,
    const barcode_char_info_t *info,
    bool was_in_frame)
{
    uint8_t i;

    if (!scanner->in_frame)
    {
        return;
    }
    if (!was_in_frame)
    {
        scanner->frame_narrow_sum = 0U;
        scanner->frame_narrow_count = 0U;
    }
    for (i = 0U; i < BARCODE_ELEMENTS; i++)
    {
        if (info->raw_pattern[i] == 'n')
        {
            scanner->frame_narrow_sum += info->run_counts[i];
            scanner->frame_narrow_count++;
        }
    }
}

static void finish_character(barcode_scanner_t *scanner)
{
    barcode_char_info_t info;
    barcode_reject_t reject;
    bool was_in_frame = scanner->in_frame;

    init_info(scanner, &info);
    if (!was_in_frame)
    {
        scanner->label_reversed = false;
    }
    reject = classify_character(scanner, reading_reversed(scanner), &info);

    /*
     * Outside a frame, a character that is '*' only when read backwards
     * means the label faces the other way; read the frame flipped. The
     * stop '*' must then also read backwards, guarding against false starts.
     */
    if (!was_in_frame && (info.symbol != '*') && (info.alt_symbol == '*') &&
        ((reject == BARCODE_REJECT_NONE) ||
         (reject == BARCODE_REJECT_WRONG_ORIENTATION)))
    {
        scanner->label_reversed = true;
        init_info(scanner, &info);
        reject = classify_character(scanner, reading_reversed(scanner), &info);
    }

    if (reject == BARCODE_REJECT_NONE)
    {
        update_ok_stats(scanner, &info, was_in_frame);
        if (!accept_character(scanner, info.symbol))
        {
            reject = BARCODE_REJECT_OVERFLOW;
            info.aborted_frame = true;
        }
        update_frame_module(scanner, &info, was_in_frame);
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
    scanner->label_reversed = false;
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