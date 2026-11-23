/*
* obs_pack.h - Minimum-bit data representation,
* All inter-task messages are fixed 8 byte word with a 3 bit type in the top bits
* to keep queue sizing trival and the same word iss the MQTT payload.
* Wire layout is built with explicit shifts/masks, and little-endian.
*/

#ifndef OBS_PACK_H
#define OBS_PACK_H

#include <stdint.h>
#include <stdbool.h>

/* Enumerations (Each fit for the stated bits)*/

typedef enum {          /* 3 bits*/
    OBS_STATE_IDLE = 0, OBS_STATE_COARSE, OBS_STATE_FINE, OBS_STATE_PROFILING,
    OBS_STATE_BYPASSING, OBS_STATE_RECOVERY, OBS_STATE_FAULT
} obs_state_t;

typedef enum {          /* 3 bits: Obstacle task -> Vehicle Controller */
    OBS_MSG_NONE = 0, OBS_MSG_PROXIMITY, OBS_MSG_PROFILE, OBS_MSG_BYPASS_REQ,
    OBS_MSG_LINE_SEARCH_REQ, OBS_MSG_LINE_REACQUIRED, OBS_MSG_SCAN_CLEAR, OBS_MSG_FAULT_EVT
} obs_msg_type_t;

typedef enum {          /* 3 bits: Vehicle Controller -> Obstacle task */
    OBS_CMD_NONE = 0, OBS_CMD_SCAN_REQ, OBS_CMD_BYPASS_DONE, OBS_CMD_ABORT,
    OBS_CMD_RESET
} obs_cmd_t;
 
typedef enum {          /* 3 bits */
    OBS_ACT_STOP = 0, OBS_ACT_CONTINUE, OBS_ACT_LEFT, OBS_ACT_RIGHT, OBS_ACT_REVERSE
} obs_action_t;
 
typedef enum {          /* 2 bits */
    OBS_SIDE_NONE = 0, OBS_SIDE_LEFT, OBS_SIDE_RIGHT
} obs_side_t;

/* Fault bits (one bit each in the TEAM fault word) 
 * PROPOSAL: Buddy 5 owns bits 8..11. Agree the final positions with
 * Buddies 1, 2 and 4 and keep one shared header for everyone. */
#define OBS_FAULT_SHIFT                 8U
#define OBS_FAULT_NO_ECHO               (1UL << 0)
#define OBS_FAULT_NO_CLEAR_SIDE         (1UL << 1)
#define OBS_FAULT_BYPASS_BLOCKED        (1UL << 2)
#define OBS_FAULT_LINE_NOT_REACQUIRED   (1UL << 3)
#define OBS_FAULT_MASK                  0xFUL
 
/* Message word  */
typedef struct { uint32_t w[2]; } obs_msg_t;    /* 8 bytes */
 
typedef struct {
    uint16_t closest_mm;    /* 12 bits */
    uint16_t width_mm;      /* 10 bits, saturates at 1023 */
    uint16_t left_mm;       /* 11 bits, saturates at 2047 */
    uint16_t right_mm;      /* 11 bits */
    uint8_t  action;        /* 3 bits */
    bool     valid;         /* 1 bit  */
    uint16_t scan_ms;       /* 10 bits, saturates at 1023 */
    uint8_t  state;         /* 3 bits */
} obs_profile_msg_t;

/* Rebuild a message from 8 little-endian bytes. */
/* Store a width-bit value at bit position pos of a byte array (see obs_pack.c).*/

/* Generic packed bit-array helpers (used for 12-bit distance arrays). */
void obs_bits_put(uint8_t *buf, uint32_t pos, uint32_t width, uint32_t val);
/* Read a width-bit value from bit position pos of a byte array. */
uint32_t obs_bits_get(const uint8_t *buf, uint32_t pos, uint32_t width);
/* Return the 3-bit message type of a word. */
obs_msg_type_t obs_msg_type(const obs_msg_t *m);

/* Pack a PROFILE message (closest, width, clearances, action, valid, scan ms, state). Values saturate. */
void obs_msg_pack_profile(obs_msg_t *m, const obs_profile_msg_t *p);
/*Decode a PROFILE message into a struct.*/
void obs_msg_unpack_profile(const obs_msg_t *m, obs_profile_msg_t *p);

/* Pack BYPASS_REQ: side, lateral mm, advance mm (10-bit offsets saturate at 1023).*/
void obs_msg_pack_bypass(obs_msg_t *m, uint8_t side, uint16_t lateral_mm, uint16_t advance_mm);
/*Decode BYPASS_REQ. */
void obs_msg_unpack_bypass(const obs_msg_t *m, uint8_t *side, uint16_t *lateral_mm, uint16_t *advance_mm);

/* Pack LINE_SEARCH_REQ: direction to turn toward the line, max search mm.*/
void obs_msg_pack_line_search(obs_msg_t *m, uint8_t dir_side, uint16_t max_search_mm);
/* Decode LINE_SEARCH_REQ. */
void obs_msg_unpack_line_search(const obs_msg_t *m, uint8_t *dir_side, uint16_t *max_search_mm);

/* Pack PROXIMITY: closest distance in mm. */
void obs_msg_pack_proximity(obs_msg_t *m, uint16_t closest_mm);
/* Decode PROXIMITY: closest distance in mm. */
uint16_t obs_msg_unpack_proximity(const obs_msg_t *m);
/* Pack FAULT_EVT: the four OBS_FAULT_* bits.*/
void obs_msg_pack_fault(obs_msg_t *m, uint32_t fault_bits);

/* Return the four fault bits from a FAULT_EVT message. */
uint32_t obs_msg_unpack_fault(const obs_msg_t *m);

/* Pack a message with at most one 10-bit number (SCAN_CLEAR, LINE_REACQUIRED). */
void obs_msg_pack_simple(obs_msg_t *m, obs_msg_type_t type, uint16_t scan_ms);
/* Write the message as 8 little-endian bytes (wire / MQTT payload). */
void obs_msg_to_bytes(const obs_msg_t *m, uint8_t out[8]);   

/* wire/MQTT payload */
void obs_msg_from_bytes(obs_msg_t *m, const uint8_t in[8]);
 
#endif /* OBS_PACK_H */
 