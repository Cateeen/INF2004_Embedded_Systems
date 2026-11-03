#include "buddy3.h"

#include <string.h>

#define BARCODE_SYNC       0xD5U
#define BARCODE_STOP       0x7EU
#define BARCODE_MAX_BITS   ((8U + 8U + (BARCODE_MAX_PAYLOAD * 8U) + 8U + 8U))

static uint32_t absolute_count_delta(int32_t first, int32_t second)
{
    int64_t delta = (int64_t)second - (int64_t)first;
    return (uint32_t)((delta < 0) ? -delta : delta);
}

static void clear_bits(barcode_scanner_t *scanner)
{
    memset(scanner->bit_buffer, 0, sizeof(scanner->bit_buffer));
    scanner->bit_count = 0U;
}

static void append_bit(barcode_scanner_t *scanner, bool black)
{
    uint16_t byte_index;
    uint8_t bit_index;

    if (scanner->bit_count >= BARCODE_MAX_BITS)
    {
        clear_bits(scanner);
    }

    byte_index = (uint16_t)(scanner->bit_count / 8U);
    bit_index = (uint8_t)(7U - (scanner->bit_count % 8U));
    if (black)
    {
        scanner->bit_buffer[byte_index] |= (uint8_t)(1U << bit_index);
    }
    scanner->bit_count++;
}

static uint8_t read_bits(
    const barcode_scanner_t *scanner,
    uint16_t offset,
    uint8_t width)
{
    uint8_t value = 0U;
    uint8_t bit;

    for (bit = 0U; bit < width; bit++)
    {
        uint16_t index = (uint16_t)(offset + bit);
        uint8_t byte = scanner->bit_buffer[index / 8U];
        uint8_t shift = (uint8_t)(7U - (index % 8U));
        value = (uint8_t)((value << 1U) | ((byte >> shift) & 1U));
    }
    return value;
}

static bool module_count(
    const barcode_scanner_t *scanner,
    uint32_t width_counts,
    uint32_t *modules)
{
    uint32_t count;
    uint32_t remainder;
    uint32_t tolerance;

    if ((scanner->config.counts_per_module == 0U) ||
        (width_counts == 0U))
    {
        return false;
    }

    count = (width_counts + scanner->config.counts_per_module / 2U) /
            scanner->config.counts_per_module;
    if ((count == 0U) || (count > 8U))
    {
        return false;
    }

    remainder = width_counts % scanner->config.counts_per_module;
    if (remainder > scanner->config.counts_per_module - remainder)
    {
        remainder = scanner->config.counts_per_module - remainder;
    }
    tolerance = (scanner->config.counts_per_module *
                 scanner->config.module_tolerance_percent) / 100U;
    if (remainder > tolerance)
    {
        return false;
    }

    *modules = count;
    return true;
}

static void parse_buffer(barcode_scanner_t *scanner)
{
    uint16_t offset;

    if (scanner->bit_count < 32U)
    {
        return;
    }

    for (offset = 0U; (uint32_t)offset + 32U <= scanner->bit_count; offset++)
    {
        uint8_t length;
        uint16_t frame_bits;
        uint8_t checksum;
        uint8_t i;

        if (read_bits(scanner, offset, 8U) != BARCODE_SYNC)
        {
            continue;
        }

        length = read_bits(scanner, (uint16_t)(offset + 8U), 8U);
        if ((length == 0U) || (length > BARCODE_MAX_PAYLOAD))
        {
            continue;
        }

        frame_bits = (uint16_t)(8U + 8U + (length * 8U) + 8U + 8U);
        if ((uint32_t)offset + frame_bits > scanner->bit_count)
        {
            continue;
        }
        if (read_bits(scanner, (uint16_t)(offset + frame_bits - 8U), 8U) !=
            BARCODE_STOP)
        {
            continue;
        }

        checksum = 0U;
        for (i = 0U; i < length; i++)
        {
            scanner->payload[i] = read_bits(
                scanner, (uint16_t)(offset + 16U + (i * 8U)), 8U);
            checksum ^= scanner->payload[i];
        }
        if (checksum != read_bits(
                scanner, (uint16_t)(offset + 16U + (length * 8U)), 8U))
        {
            continue;
        }

        scanner->payload_length = length;
        scanner->result_ready = true;
        clear_bits(scanner);
        return;
    }
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
    return true;
}

bool barcode_scanner_update(
    barcode_scanner_t *scanner,
    bool black,
    int32_t encoder_count)
{
    uint32_t width_counts;
    uint32_t modules;
    uint32_t i;

    if ((scanner == NULL) || !scanner->initialized)
    {
        return false;
    }
    if (black == scanner->last_black)
    {
        return true;
    }

    width_counts = absolute_count_delta(
        scanner->last_encoder_count, encoder_count);
    if (!module_count(scanner, width_counts, &modules))
    {
        clear_bits(scanner);
    }
    else
    {
        for (i = 0U; i < modules; i++)
        {
            append_bit(scanner, scanner->last_black);
        }
        parse_buffer(scanner);
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
    clear_bits(scanner);
    scanner->result_ready = false;
}
