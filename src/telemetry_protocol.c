#include "telemetry_protocol.h"

bool telemetry_pack(const telemetry_data_t *d,
                    uint8_t out[TELEMETRY_PACKET_SIZE])
{
    if ((d == NULL) || (out == NULL)) return false;
    if (d->message_type > 7U || d->vehicle_state > 7U ||
        d->line_state > 3U || d->obstacle_state > 3U ||
        d->terrain_state > 3U || d->fault_flags > FAULT_ALL ||
        d->sequence > 127U || d->battery_pct > 100U) return false;

    out[0] = (uint8_t)((d->message_type << 5U) |
                       (d->vehicle_state << 2U) |
                       d->line_state);
    out[1] = (uint8_t)((d->obstacle_state << 6U) |
                       (d->terrain_state << 4U) |
                       (d->fault_flags & 0x0FU));
    out[2] = (uint8_t)(((d->fault_flags >> 4U) & 0x01U) |
                       ((d->sequence & 0x7FU) << 1U));
    out[3] = (uint8_t)(d->battery_pct & 0x7FU);
    out[4] = 0U; /* reserved; must remain zero */
    return true;
}

bool telemetry_unpack(const uint8_t in[TELEMETRY_PACKET_SIZE],
                      telemetry_data_t *d)
{
    if ((in == NULL) || (d == NULL)) return false;
    if ((in[2] & 0x01U) > 1U || (in[3] & 0x80U) != 0U || in[4] != 0U)
        return false;

    d->message_type = (uint8_t)(in[0] >> 5U);
    d->vehicle_state = (uint8_t)((in[0] >> 2U) & 0x07U);
    d->line_state = (uint8_t)(in[0] & 0x03U);
    d->obstacle_state = (uint8_t)((in[1] >> 6U) & 0x03U);
    d->terrain_state = (uint8_t)((in[1] >> 4U) & 0x03U);
    d->fault_flags = (uint8_t)((in[1] & 0x0FU) | ((in[2] & 0x01U) << 4U));
    d->sequence = (uint8_t)((in[2] >> 1U) & 0x7FU);
    d->battery_pct = (uint8_t)(in[3] & 0x7FU);

    if (d->battery_pct > 100U || d->fault_flags > FAULT_ALL) return false;
    return true;
}

bool command_pack(const remote_command_t *c,
                  uint8_t out[COMMAND_PACKET_SIZE])
{
    if ((c == NULL) || (out == NULL)) return false;
    if (c->type > 7U || c->argument > 31U) return false;
    out[0] = (uint8_t)((c->type << 5U) | (c->argument & 0x1FU));
    out[1] = c->sequence;
    return true;
}

bool command_unpack(const uint8_t in[COMMAND_PACKET_SIZE],
                    remote_command_t *c)
{
    if ((in == NULL) || (c == NULL)) return false;
    c->type = (uint8_t)(in[0] >> 5U);
    c->argument = (uint8_t)(in[0] & 0x1FU);
    c->sequence = in[1];
    if (c->type > CMD_REQUEST_STATUS) return false;
    return true;
}
