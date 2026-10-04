// FEATURE: Owner-side codec and request loop for the change-device-state IPC.
// SPEC: 76-byte wire format, finite timeout, bounded foreign-packet skip,
// SPEC: every transport failure mode surfaces as a distinct status.
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mask.h"

static void die(const char *msg) {
    fprintf(stderr, "FAIL: %s\n", msg);
    exit(1);
}

typedef struct {
    uint8_t bytes[MASK_PACKET_LEN];
} scripted_reply;

typedef struct {
    scripted_reply replies[8];
    size_t nreplies;
    size_t next;

    uint8_t sent[MASK_PACKET_LEN];
    size_t nsent;
    long send_ret; /* 0 = normal */
    long recv_ret; /* 0 = normal; -1 = errno; >0 = short read */
    int recv_eof;  /* return 0 (peer closed) instead of a scripted reply */
    int recv_errno;

    int last_timeout_ms;
} fake_transport;

static long fake_send(void *ctx, const uint8_t *buf, size_t len) {
    fake_transport *t = ctx;
    if (t->nsent == 0 && len == MASK_PACKET_LEN)
        memcpy(t->sent, buf, len);
    t->nsent++;
    return t->send_ret ? t->send_ret : (long)len;
}

static long fake_recv(void *ctx, uint8_t *buf, size_t len, int timeout_ms) {
    fake_transport *t = ctx;
    t->last_timeout_ms = timeout_ms;
    if (t->recv_ret)
        return t->recv_ret == -1 ? (errno = t->recv_errno, -1) : t->recv_ret;
    if (t->recv_eof) return 0;
    if (t->next >= t->nreplies) {
        errno = ETIMEDOUT;
        return -1;
    }
    memcpy(buf, t->replies[t->next++].bytes,
           len < MASK_PACKET_LEN ? len : MASK_PACKET_LEN);
    return (long)MASK_PACKET_LEN;
}

static robot_ipc_transport_t make_transport(fake_transport *t) {
    robot_ipc_transport_t tr = {fake_send, fake_recv, t};
    return tr;
}

static void make_reply(scripted_reply *r, uint8_t src, uint8_t dst, uint16_t event) {
    memset(r->bytes, 0, sizeof r->bytes);
    r->bytes[0] = (uint8_t)(event & 0xff);
    r->bytes[1] = (uint8_t)(event >> 8);
    r->bytes[2] = src;
    r->bytes[3] = dst;
}

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
           (uint32_t)p[3] << 24;
}

/* ---- codec ---- */

static void test_build_known_masks(void) {
    static const struct { const char *name; uint32_t bitmap; } cases[] = {
        {"stock cleaning", MASK_STOCK_CLEANING},
        {"no brushes", MASK_CANDIDATE_NO_BRUSHES},
        {"no brushes no fan", MASK_CANDIDATE_NO_BRUSHES_NO_FAN},
    };
    uint8_t pkt[MASK_PACKET_LEN];
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        size_t n = mask_build_change_device_state(pkt, sizeof pkt, 0x36,
                                                  cases[i].bitmap);
        if (n != MASK_PACKET_LEN) die("build: wrong length");
        uint16_t event;
        uint8_t src, dst;
        mask_parse_header(pkt, &event, &src, &dst);
        if (event != MASK_EVENT_CHANGE_DEVICE_STATE) die("build: wrong event");
        if (src != MASK_MODULE_RC || dst != MASK_MODULE_MC)
            die("build: wrong routing");
        if (le32(pkt + 4) != 0x36) die("build: sequence not at +4");
        if (le32(pkt + 8) != 4) die("build: payload length not at +8");
        if (le32(pkt + 12) != cases[i].bitmap) die("build: bitmap not at +12");
    }
}

static void test_reject_bad_bitmaps(void) {
    uint8_t pkt[MASK_PACKET_LEN];
    if (mask_build_change_device_state(pkt, sizeof pkt, 1, 0) != 0)
        die("empty bitmap accepted");
    if (mask_build_change_device_state(pkt, sizeof pkt, 1, 1u << 19) != 0)
        die("unknown bit 19 accepted");
    if (mask_build_change_device_state(pkt, sizeof pkt, 1, 1u << 24) != 0)
        die("unknown bit 24 accepted");
    if (mask_build_change_device_state(pkt, sizeof pkt, 1,
                                        MASK_BIT_LDS | MASK_BIT_LDS_FILTERED) != 0)
        die("both LDS bits accepted");
}

static void test_reject_small_buffer(void) {
    uint8_t small[MASK_PACKET_LEN - 1];
    if (mask_build_change_device_state(small, sizeof small, 1,
                                       MASK_CANDIDATE_NO_BRUSHES) != 0)
        die("small buffer accepted");
}

/* ---- request loop ---- */

static void test_happy_path(void) {
    fake_transport t;
    memset(&t, 0, sizeof t);
    make_reply(&t.replies[0], MASK_MODULE_MC, MASK_MODULE_RC,
               MASK_EVENT_DEVICE_STATE_DONE);
    t.nreplies = 1;
    robot_ipc_transport_t tr = make_transport(&t);
    if (mask_change_device_state(&tr, 0x36, MASK_CANDIDATE_NO_BRUSHES, 250) != MASK_OK)
        die("happy path failed");
    if (t.nsent != 1) die("happy path resent the request");
    if (t.last_timeout_ms != 250) die("timeout not passed to recv");
}

static void test_timeout_is_finite(void) {
    fake_transport t;
    memset(&t, 0, sizeof t);
    robot_ipc_transport_t tr = make_transport(&t);
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_TIMEOUT)
        die("silent channel must time out");
}

static void test_skips_foreign_packets(void) {
    fake_transport t;
    memset(&t, 0, sizeof t);
    make_reply(&t.replies[0], MASK_MODULE_MC, MASK_MODULE_RC, 0x00f9);
    make_reply(&t.replies[1], MASK_MODULE_MC, MASK_MODULE_RC, 0x0111);
    make_reply(&t.replies[2], MASK_MODULE_MC, MASK_MODULE_RC,
               MASK_EVENT_DEVICE_STATE_DONE);
    t.nreplies = 3;
    robot_ipc_transport_t tr = make_transport(&t);
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) != MASK_OK)
        die("foreign packets before ack must be skipped");
}

static void test_foreign_flood_fails(void) {
    fake_transport t;
    memset(&t, 0, sizeof t);
    for (size_t i = 0; i <= MASK_MAX_FOREIGN; i++)
        make_reply(&t.replies[i], MASK_MODULE_MC, MASK_MODULE_RC, 0x00f9);
    t.nreplies = MASK_MAX_FOREIGN + 1;
    robot_ipc_transport_t tr = make_transport(&t);
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_UNEXPECTED)
        die("packet flood must fail as unexpected");
}

static void test_wrong_direction_ack(void) {
    fake_transport t;
    memset(&t, 0, sizeof t);
    make_reply(&t.replies[0], MASK_MODULE_RC, MASK_MODULE_MC,
               MASK_EVENT_DEVICE_STATE_DONE);
    t.nreplies = 1;
    robot_ipc_transport_t tr = make_transport(&t);
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_UNEXPECTED)
        die("ack from wrong direction must fail");
}

static void test_transport_failures(void) {
    fake_transport t;
    memset(&t, 0, sizeof t);
    make_reply(&t.replies[0], MASK_MODULE_MC, MASK_MODULE_RC,
               MASK_EVENT_DEVICE_STATE_DONE);
    t.nreplies = 1;
    robot_ipc_transport_t tr = make_transport(&t);

    t.send_ret = -1;
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_WRITE)
        die("failed write must fail as write error");
    t.send_ret = 40; /* short write */
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_SHORT_WRITE)
        die("short write must fail");
    t.send_ret = 0;

    t.recv_ret = 40; /* short read */
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_SHORT_READ)
        die("short read must fail as framing desync");
    t.recv_ret = 1; /* one-byte read: framing desync, not EOF */
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_SHORT_READ)
        die("one-byte read must fail as short read");
    t.recv_ret = 0;
    t.recv_eof = 1;
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_EOF)
        die("EOF must fail as eof");
    t.recv_eof = 0;
    t.recv_ret = -1;
    t.recv_errno = EIO;
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_READ)
        die("EIO must fail as read error");
    t.recv_ret = 0;
}

static void test_bad_args(void) {
    fake_transport t;
    memset(&t, 0, sizeof t);
    robot_ipc_transport_t tr = make_transport(&t);
    if (mask_change_device_state(NULL, 1, MASK_CANDIDATE_NO_BRUSHES, 250) !=
        MASK_ERR_ARGS)
        die("NULL transport must fail as args");
    if (mask_change_device_state(&tr, 1, MASK_CANDIDATE_NO_BRUSHES, 0) !=
        MASK_ERR_ARGS)
        die("zero timeout must fail as args");
    if (mask_change_device_state(&tr, 1, 1u << 19, 250) != MASK_ERR_ARGS)
        die("unknown bitmap must fail as args");
    if (t.nsent != 0)
        die("argument errors must not reach the transport");
}

int main(void) {
    test_build_known_masks();
    test_reject_bad_bitmaps();
    test_reject_small_buffer();
    test_happy_path();
    test_timeout_is_finite();
    test_skips_foreign_packets();
    test_foreign_flood_fails();
    test_wrong_direction_ack();
    test_transport_failures();
    test_bad_args();
    printf("owner mask tests passed\n");
    return 0;
}
