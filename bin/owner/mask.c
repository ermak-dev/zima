/* Owner-side implementation of the change-device-state exchange.
 * The wire contract is documented in mask.h; everything here is transport-
 * agnostic so tests can drive it with in-memory fakes. */
#include <errno.h>

#include "mask.h"

static int bitmap_allowed(uint32_t bitmap) {
    if (bitmap == 0) return 0;
    if (bitmap & ~MASK_KNOWN_BITS) return 0;
    /* Bits 7 and 8 are two LDS modes and never appear together. */
    if ((bitmap & MASK_BIT_LDS) && (bitmap & MASK_BIT_LDS_FILTERED)) return 0;
    return 1;
}

size_t mask_build_change_device_state(uint8_t *out, size_t cap, uint32_t seq,
                                      uint32_t bitmap) {
    if (out == NULL || cap < MASK_PACKET_LEN) return 0;
    if (!bitmap_allowed(bitmap)) return 0;

    for (size_t i = 0; i < MASK_PACKET_LEN; i++)
        out[i] = 0;
    out[0] = (uint8_t)(MASK_EVENT_CHANGE_DEVICE_STATE & 0xff);
    out[1] = (uint8_t)(MASK_EVENT_CHANGE_DEVICE_STATE >> 8);
    out[2] = MASK_MODULE_RC;
    out[3] = MASK_MODULE_MC;
    out[4] = (uint8_t)(seq & 0xff);
    out[5] = (uint8_t)(seq >> 8 & 0xff);
    out[6] = (uint8_t)(seq >> 16 & 0xff);
    out[7] = (uint8_t)(seq >> 24 & 0xff);
    out[8] = 4; /* payload length */
    out[12] = (uint8_t)(bitmap & 0xff);
    out[13] = (uint8_t)(bitmap >> 8 & 0xff);
    out[14] = (uint8_t)(bitmap >> 16 & 0xff);
    out[15] = (uint8_t)(bitmap >> 24 & 0xff);
    return MASK_PACKET_LEN;
}

void mask_parse_header(const uint8_t *pkt, uint16_t *event, uint8_t *src,
                       uint8_t *dst) {
    if (event) *event = (uint16_t)(pkt[0] | (uint16_t)pkt[1] << 8);
    if (src) *src = pkt[2];
    if (dst) *dst = pkt[3];
}

int mask_change_device_state(const robot_ipc_transport_t *transport, uint32_t seq,
                             uint32_t bitmap, int timeout_ms) {
    if (transport == NULL || transport->send == NULL ||
        transport->recv == NULL || timeout_ms <= 0)
        return MASK_ERR_ARGS;
    if (!bitmap_allowed(bitmap)) return MASK_ERR_ARGS;

    uint8_t request[MASK_PACKET_LEN];
    if (mask_build_change_device_state(request, sizeof request, seq, bitmap) == 0)
        return MASK_ERR_ARGS;

    long sent = transport->send(transport->ctx, request, MASK_PACKET_LEN);
    if (sent < 0) return MASK_ERR_WRITE;
    if ((size_t)sent != MASK_PACKET_LEN) return MASK_ERR_SHORT_WRITE;

    unsigned foreign = 0;
    for (;;) {
        uint8_t reply[MASK_PACKET_LEN];
        long got = transport->recv(transport->ctx, reply, MASK_PACKET_LEN,
                                   timeout_ms);
        if (got == 0) return MASK_ERR_EOF;
        if (got < 0) {
            if (errno == ETIMEDOUT || errno == EAGAIN || errno == EWOULDBLOCK)
                return MASK_ERR_TIMEOUT;
            return MASK_ERR_READ;
        }
        if ((size_t)got != MASK_PACKET_LEN) return MASK_ERR_SHORT_READ;

        uint16_t event;
        uint8_t src, dst;
        mask_parse_header(reply, &event, &src, &dst);
        if (event == MASK_EVENT_DEVICE_STATE_DONE) {
            /* An ack-shaped packet from the wrong direction means the
             * channel is corrupt: fail instead of silently skipping. */
            if (src == MASK_MODULE_MC && dst == MASK_MODULE_RC)
                return MASK_OK;
            return MASK_ERR_UNEXPECTED;
        }
        /* Anything else is housekeeping traffic from the channel, not our
         * acknowledgement. Skip a bounded number, then give up: an
         * unbounded skip loop could spin forever on a chatty peer. */
        if (++foreign > MASK_MAX_FOREIGN) return MASK_ERR_UNEXPECTED;
    }
}
