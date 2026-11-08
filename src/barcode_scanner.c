#include "buddy3.h"

#include <string.h>

static const char code39_characters[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static const char *const code39_patterns[] =
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
    "nwnwnnnwn", "nwnnnwnwn", "nnnwnwnwn"
};

static uint32_t absolute_count_delta(int32_t first, int32_t second)
{
    int64_t delta = (int64_t)second - (int64_t)first;
    return (uint32_t)((delta < 0) ? -delta : delta);
}

static void reset_character(barcode_scanner_t *scanner)
{
    memset(scanner->run_counts, 0, sizeof(scanner->run_counts));
    scanner->run_count = 0U;
}

static char decode_character(
    const barcode_scanner_t *scanner,
    bool reverse)
{
    char pattern[10];
    uint8_t i;
    uint8_t j;
    uint32_t wide_threshold;

    if ((scanner->run_count != 9U) ||
        (scanner->config.counts_per_module == 0U))
    {
        return '\0';
    }

    /*
     * Code 39 has narrow and wide elements. A wide element is normally at
     * least twice the narrow width; 1.5 times gives margin for calibration.
     */
    wide_threshold = scanner->config.counts_per_module * 150U;
    for (i = 0U; i < 9U; i++)
    {
        uint8_t source_index = reverse ? (uint8_t)(8U - i) : i;
        pattern[i] = (scanner->run_counts[source_index] * 100U >=
                      wide_threshold) ?
                     'w' : 'n';
    }
    pattern[9] = '\0';

    for (j = 0U; j < (uint8_t)(sizeof(code39_patterns) /
                               sizeof(code39_patterns[0])); j++)
    {
        if (memcmp(pattern, code39_patterns[j], 9U) == 0)
        {
            return (j == 42U) ? '*' : code39_characters[j];
        }
    }

    return '\0';
}

static void accept_character(barcode_scanner_t *scanner, char character)
{
    if (character == '\0')
    {
        scanner->in_frame = false;
        scanner->payload_length = 0U;
        return;
    }

    if (character == '*')
    {
        if (!scanner->in_frame)
        {
            scanner->in_frame = true;
            scanner->payload_length = 0U;
        }
        else if (scanner->payload_length > 0U)
        {
            uint8_t i;
            uint8_t half_length = scanner->payload_length / 2U;

            if (scanner->scan_direction < 0)
            {
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
        }
        return;
    }

    if (scanner->in_frame)
    {
        if (scanner->payload_length < BARCODE_MAX_PAYLOAD)
        {
            scanner->payload[scanner->payload_length++] = (uint8_t)character;
        }
        else
        {
            scanner->in_frame = false;
            scanner->payload_length = 0U;
        }
    }
}

static void finish_character(barcode_scanner_t *scanner)
{
    if (scanner->run_count == 9U)
    {
        accept_character(scanner, decode_character(
            scanner, scanner->scan_direction < 0));
    }
    else
    {
        scanner->in_frame = false;
        scanner->payload_length = 0U;
    }
    reset_character(scanner);
}

bool barcode_scanner_init(
    barcode_scanner_t *scanner,
    const barcode_scanner_config_t *config,
    bool initial_black,
    int32_t initial_encoder_count)
{
    if ((scanner == NULL) || (config == NULL) ||
        (config->counts_per_module == 0U) ||
        (config->module_tolerance_percent > 50U))
    {
        return false;
    }

    memset(scanner, 0, sizeof(*scanner));
    scanner->config = *config;
    scanner->initialized = true;
    scanner->last_black = initial_black;
    scanner->last_encoder_count = initial_encoder_count;
    scanner->collecting = initial_black;
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

    count_delta = encoder_count - scanner->last_encoder_count;
    width_counts = absolute_count_delta(
        scanner->last_encoder_count, encoder_count);

    if (!scanner->collecting)
    {
        if (!scanner->last_black && black)
        {
            scanner->collecting = true;
            scanner->scan_direction = (count_delta < 0) ? -1 : 1;
            reset_character(scanner);
        }
    }
    else if (scanner->run_count < 9U)
    {
        scanner->run_counts[scanner->run_count++] = width_counts;
        if (scanner->last_black && !black &&
            scanner->run_count == 9U)
        {
            finish_character(scanner);
            scanner->collecting = false;
        }
    }
    else
    {
        scanner->collecting = false;
        reset_character(scanner);
    }

    scanner->last_black = black;
    scanner->last_encoder_count = encoder_count;
    return true;
}

bool barcode_scanner_take_result(
    barcode_scanner_t *scanner,
    uint8_t *payload,
    uint8_t *length)
{
    if ((scanner == NULL) || (payload == NULL) || (length == NULL) ||
        !scanner->result_ready)
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
}
