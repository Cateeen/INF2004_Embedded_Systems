#include "obs_pack.h"

#define MASK64(bits)        ((1ULL << (bits)) - 1ULL)
#define MSG_TYPE_SH         61U
#define MSG_TYPE_BITS       3U

/* field_put - writes a value into one bit-field of a 64 bit word*/
static uint64_t field_put(uint64_t w, uint32_t v, uint32_t sh, uint32_t bits){
    uint64_t m = MASK64(bits);
    uint64_t x = (uint64_t)v;
    if (x > m){
        x = m;
    }
    return (w & ~(m << sh)) | (x << sh);
}

/* field_get - reads one bit field out of 64-bit word */
static uint32_t field_get(uint64_t w, uint32_t sh, uint32_t bits){
    return (uint32_t)((w >> sh) & MASK64(bits));
}

/* to64 - joins the two 32-bit halves of a message into one 64-bit value */
static uint64_t to64(const obs_msg_t *m){
    return ((uint64_t)m->w[1] << 32) | (uint64_t) m->w[0];
}

/* from64 - stores a 64-bit value back into a message */
static void from64(obs_msg_t *m, uint64_t w){
    m-> w[0] = (uint32_t)(w & 0xFFFFFFFFULL);
    m-> w[1] = (uint32_t)(w >> 32);
}

/* obs_bits_put - store a value at any bit position in a byte array */
void obs_bits_put(uint8_t *buf, uint32_t pos, uint32_t width, uint32_t val){
    uint32_t i;
    for (i = 0U; i < width; i++) {
        uint32_t bit = pos + i;
        uint8_t  mask = (uint8_t)(1U << (bit & 7U));
        if (((val >> i) & 1U) != 0U) {
            buf[bit >> 3] |= mask;
        } else {
            buf[bit >> 3] &= (uint8_t)~mask;
        }
    }
}

/* obs_bits_get - read a value from any bit position in a byte array*/
uint32_t obs_bits_get(const uint8_t *buf, uint32_t pos, uint32_t width){
    uint32_t v = 0U;
    uint32_t i;
    for(i=0U; i< width; i++){
        uint32_t bit = pos + i;
        v |= (uint32_t)((buf[bit>>3] >> (bit& 7U)) & 1U) << i;
    }
    return v;
}

/* obs_msg_type - read the message type*/
obs_msg_type_t obs_msg_type(const obs_msg_t *m){
    return (obs_msg_type_t)field_get(to64(m), MSG_TYPE_SH, MSG_TYPE_BITS);
}

/* start - begins a new message word*/
static uint64_t start(obs_msg_type_t t){
    return field_put(0ULL, (uint32_t)t, MSG_TYPE_SH, MSG_TYPE_BITS);
}

/* obs_msg_pack_profile - Building a profile message*/
/* PROFILE layout: [11:0] closest, [21:12] width, [32:22] left, [43:33] right, 
* [46:44] action, [47] valid, [57:48] scan_ms. [60:58] state */
void obs_msg_pack_profile(obs_msg_t *m, const obs_profile_msg_t *p){
    uint64_t w = start(OBS_MSG_PROFILE);
    w = field_put(w, p->closest_mm, 0U, 12U);
    w = field_put(w, p->width_mm, 12U, 10U);
    w = field_put(w, p->left_mm, 22U, 11U);
    w = field_put(w, p->right_mm, 33U, 11U);
    w = field_put(w, p->action, 44U, 3U);
    w = field_put(w, p->valid ? 1U : 0U, 47U, 1U);
    w = field_put(w, p->scan_ms, 48U, 10U);
    w = field_put(w, p->state, 58U, 3U);
    from64(m, w);
}

/* obs_msg_unpack_profile - decode a profile message*/
void obs_msg_unpack_profile(const obs_msg_t *m, obs_profile_msg_t *p){
    uint64_t w = to64(m);
    p->closest_mm = (uint16_t)field_get(w, 0U, 12U);
    p->width_mm   = (uint16_t)field_get(w, 12U, 10U);
    p->left_mm    = (uint16_t)field_get(w, 22U, 11U);
    p->right_mm   = (uint16_t)field_get(w, 33U, 11U);
    p->action     = (uint8_t)field_get(w, 44U, 3U);
    p->valid      = (field_get(w, 47U, 1U) != 0U);
    p->scan_ms    = (uint16_t)field_get(w, 48U, 10U);
    p->state      = (uint8_t)field_get(w, 58U, 3U);
}


/* obs_msg_bypass - build a bypass_REQ message */
/* BYPASS_REQ: [1:0] side, [11:2] lateral_mm, [22:12] advance_mm */
void obs_msg_pack_bypass(obs_msg_t *m, uint8_t side, uint16_t lateral_mm, uint16_t advance_mm){
    uint64_t w = start(OBS_MSG_BYPASS_REQ);
    w = field_put(w, side, 0U, 2U);
    w = field_put(w, lateral_mm, 2U, 10U);
    w = field_put(w, advance_mm, 12U, 10U);
    from64(m, w);
}

/* obs_msg_unpack_bypass - decode a BYPASS_REQ message */
void obs_msg_unpack_bypass(const obs_msg_t *m, uint8_t *side, uint16_t *lateral_mm, uint16_t *advance_mm){
    uint64_t w = to64(m);
    *side = (uint8_t)field_get(w, 0U, 2U);
    *lateral_mm = (uint16_t)field_get(w, 2U, 10U);
    *advance_mm = (uint16_t)field_get(w, 12U, 10U);
}

/* obs_msg_line_search - build a LINE_SEARCH_REQ message */
void obs_msg_pack_line_search(obs_msg_t *m, uint8_t dir_side, uint16_t max_search_mm){
    uint64_t w = start(OBS_MSG_LINE_SEARCH_REQ);
    w = field_put(w, dir_side, 0U, 2U);
    w = field_put(w, max_search_mm, 2U, 10U);
    from64(m,w);
}

/* obs_msg_unpack_line_search - decode a LINE_SEARCH_REQ message*/
void obs_msg_unpack_line_search(const obs_msg_t *m, uint8_t *dir_side, uint16_t *max_search_mm){
    uint64_t w = to64(m);
    *dir_side = (uint8_t)field_get(w, 0U, 2U);
    *max_search_mm = (uint16_t)field_get(w, 2U, 10U);
}

/* obs_msg_pack_proximity - build a proximity message */
void obs_msg_pack_proximity(obs_msg_t *m, uint16_t closest_mm){
    from64(m, field_put(start(OBS_MSG_PROXIMITY), closest_mm, 0U, 12U));
}

/* obs_msg_unpack_proximity - decode a proximity message. */
uint16_t obs_msg_unpack_proximity(const obs_msg_t *m){
    return (uint16_t)field_get(to64(m), 0U, 12U);
}

/* obs_msg_pack_simple - build a message with at most one 10-bit number.
 * Used for SCAN_CLEAR and LINE_REACQUIRED: [9:0] scan_ms, [63:61] type. */
void obs_msg_pack_simple(obs_msg_t *m, obs_msg_type_t type, uint16_t scan_ms){
    from64(m, field_put(start(type), scan_ms, 0U, 10U));
}

/* obs_msg_pack_fault - build a FAULT_EVT message */
void obs_msg_pack_fault(obs_msg_t *m, uint32_t fault_bits){
    from64(m, field_put(start(OBS_MSG_FAULT_EVT), fault_bits, 0U, 4U));
}

/* obs_msg_unpack_fault - decode a FAULT_EVT message. */
uint32_t obs_msg_unpack_fault(const obs_msg_t *m){
    return field_get(to64(m), 0U, 4U);
}

/* obs_msg_to_bytes - turn a message into 8 bytes for the wire. */
void obs_msg_to_bytes(const obs_msg_t *m, uint8_t out[8]){
    uint64_t w = to64(m);
    uint32_t i;
    for (i = 0U; i < 8U; i++) {
        out[i] = (uint8_t)(w >> (8U * i));
    }
}
 
/* obs_msg_from_bytes - rebuild a message from 8 received bytes. */
void obs_msg_from_bytes(obs_msg_t *m, const uint8_t in[8]){
    uint64_t w = 0ULL;
    uint32_t i;
    for (i = 0U; i < 8U; i++) {
        w |= ((uint64_t)in[i]) << (8U * i);
    }
    from64(m, w);
}

