/* Device-mask owner: codec and request loop for the internal
 * RoboController<->McuProxy change-device-state exchange.
 *
 * Wire format (76-byte packet, little-endian words), established offline from
 * libroboipc.so, the RoboController binary and a saved FIFO trace; see
 * docs/player-platform-survey.md:
 *   +0  u16 event      0x0045 request, 0x00a9 acknowledgement
 *   +2  u8  src        1 = RoboController side, 4 = McuProxy side
 *   +3  u8  dst        counterpart of src
 *   +4  u32 sender sequence number (NOT echoed: the reply carries the
 *                      responder's own counter, correlation is one request
 *                      in flight plus the event id)
 *   +8  u32 payload length (always 4 here)
 *   +12 u32 device bitmap
 *
 * Device bitmap bits (BitmapToString, RoboController 0x2e284):
 *   0 Bumper, 1 UltraSonic, 2 Cliff, 3 Compass, 4 Wall, 5 Drop, 6 Bin,
 *   7 LDS, 8 LDS-filtered (mutually exclusive with 7), 9 MainBrush,
 *   10 SideBrush, 11 Fan, 12 Wheel, 13 Odometer, 14 Accelerator,
 *   15 Gyroscope, 16 Incline, 17 Dock, 18 Battery, 20 Slam.
 *
 * The stock RoboController waits for the acknowledgement forever; this module
 * adds the finite per-call timeout and the foreign-packet bound the stock code
 * lacks. Worst-case wall time is bounded by roughly
 * (1 + MASK_MAX_FOREIGN) * timeout_ms.
 */
#ifndef ROBOT_MASK_H
#define ROBOT_MASK_H

#include <stddef.h>
#include <stdint.h>

#define MASK_PACKET_LEN 76u

/* ModuleID_t values (GetProcIndex, libroboipc 0x50b4). */
#define MASK_MODULE_RC 1u
#define MASK_MODULE_MC 4u

#define MASK_EVENT_CHANGE_DEVICE_STATE 0x0045u
#define MASK_EVENT_DEVICE_STATE_DONE 0x00a9u

/* Bitmap bits used by the masks this owner may send. */
#define MASK_BIT_LDS (1u << 7)
#define MASK_BIT_LDS_FILTERED (1u << 8)
#define MASK_BIT_MAIN_BRUSH (1u << 9)
#define MASK_BIT_SIDE_BRUSH (1u << 10)
#define MASK_BIT_FAN (1u << 11)
#define MASK_BIT_WHEEL (1u << 12)
#define MASK_BIT_SLAM (1u << 20)
/* Bits with a known device behind them; anything else is rejected. */
#define MASK_KNOWN_BITS 0x0017ffffu

/* Masks observed on the wire or derived from them; candidates stay untested
 * on the physical robot until a separate authorized session. */
#define MASK_STOCK_CLEANING 0x0010feffu    /* both brushes + LDS + fan */
#define MASK_CANDIDATE_NO_BRUSHES 0x0010f8ffu /* LDS kept, brushes off */
#define MASK_CANDIDATE_NO_BRUSHES_NO_FAN 0x0010f0ffu

/* Housekeeping packets from Mc that are not our acknowledgement are skipped,
 * but only a bounded number of them. */
#define MASK_MAX_FOREIGN 4u

enum {
    MASK_OK = 0,
    MASK_ERR_ARGS = 1,       /* NULL hooks, non-positive timeout, bad bitmap */
    MASK_ERR_WRITE = 2,      /* transport write failed (e.g. EPIPE) */
    MASK_ERR_SHORT_WRITE = 3, /* fewer than 76 bytes written */
    MASK_ERR_READ = 4,       /* transport read failed for another reason */
    MASK_ERR_TIMEOUT = 5,    /* no acknowledgement within timeout_ms */
    MASK_ERR_EOF = 6,        /* peer closed the channel */
    MASK_ERR_SHORT_READ = 7, /* fewer than 76 bytes read: framing desync */
    MASK_ERR_UNEXPECTED = 8  /* ack from the wrong direction, or a flood of
                                foreign packets */
};

/* Transport the owner talks over. The robot build adapts this to the IPC
 * channel; tests inject an in-memory fake.
 * send: returns bytes written, or -1 with errno on error.
 * recv: returns bytes read (a full packet is MASK_PACKET_LEN), 0 on EOF, or
 *       -1 with errno (ETIMEDOUT when the per-call timeout expired). */
typedef struct robot_ipc_transport {
    long (*send)(void *ctx, const uint8_t *buf, size_t len);
    long (*recv)(void *ctx, uint8_t *buf, size_t len, int timeout_ms);
    void *ctx;
} robot_ipc_transport_t;

/* Build the 76-byte Rc->Mc change-device-state request. Returns
 * MASK_PACKET_LEN, or 0 when the buffer is too small or the bitmap is empty,
 * uses unknown bits, or sets both LDS bits at once. */
size_t mask_build_change_device_state(uint8_t *out, size_t cap, uint32_t seq,
                                      uint32_t bitmap);

/* Decode the routing header of a packet. */
void mask_parse_header(const uint8_t *pkt, uint16_t *event, uint8_t *src,
                       uint8_t *dst);

/* Send one change-device-state request and wait for its 0xa9 acknowledgement,
 * skipping up to MASK_MAX_FOREIGN foreign packets. Returns a MASK_* status.
 * The reply is matched by event id and direction; its sequence word is not
 * checked because the responder uses its own counter. */
int mask_change_device_state(const robot_ipc_transport_t *transport,
                             uint32_t seq, uint32_t bitmap, int timeout_ms);

#endif /* ROBOT_MASK_H */
