/**
 * @file bb_can.c
 * @brief FlexCAN1 — new to this project; the STM32 had no CAN at all.
 *
 * The NUCLEO carried no CAN transceiver, so the `sniffer` project's CAN decoder
 * has never had real traffic to decode. This board has FlexCAN1 wired to an
 * on-board transceiver (U13), so it can finally produce some.
 *
 * **Two board facts to know before expecting frames:**
 *
 *   - Jumpers **J44 and J46** must be in position 1-2. Position 2-3 routes
 *     those pads to SAI1 on J51 instead, and the CAN pins go nowhere.
 *   - Connector **J22 is DNP**. The transceiver is fitted and wired to it, but
 *     the 3-pin header is not populated - there is physically nothing to plug a
 *     bus into until one is soldered on. `CAN:CFG` says so in its reply rather
 *     than letting a silent lack of traffic look like a firmware fault.
 *
 * Listen-only mode is what makes CAN a *sniffer* rather than just a controller,
 * and it is a real hardware mode here: the receiver acknowledges nothing and
 * transmits nothing, so attaching to a live bus cannot disturb it. That is a
 * much stronger guarantee than the FlexIO capture gives on the two-wire buses,
 * where "passive" is a property of how the firmware is written.
 */
#include "bb_board.h"
#include "bb_can.h"
#include "bb_cmd.h"
#include "bb_image.h"
#include "bb_io.h"
#include "bb_status.h"

#include "fsl_clock.h"
#include "fsl_flexcan.h"

#include <stdio.h>
#include <string.h>

/* Message buffer 0 transmits, 1 receives. FlexCAN has many more; two is what a
 * request/response command protocol can actually use, and a deep filter bank
 * belongs to the capture engine rather than here. */
#define CAN_TX_MB   0u
#define CAN_RX_MB   1u

/* Frames buffered between CAN:RX? polls. A busy 500 kbit bus is around 4000
 * frames a second, so this is tens of milliseconds - enough that a host polling
 * at a sane rate loses nothing, and honest about the fact that continuous
 * capture is the sniffer's job, not this command's. */
#define CAN_RX_RING 32u

static const bb_pin_t s_tx = BB_CAN_TX;
static const bb_pin_t s_rx = BB_CAN_RX;

static struct {
    uint32_t bitrate;
    bool     listen_only;
    bool     configured;
    bool     ext_filter;      /**< accept extended IDs as well as standard */
    uint32_t rx_dropped;
    uint32_t rx_frames;
    uint32_t tx_frames;
} s = { .bitrate = 500000u, .listen_only = false };

static flexcan_frame_t s_ring[CAN_RX_RING];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;

/* ---- configuration ----------------------------------------------------- */

static bool can_apply(void)
{
    flexcan_config_t cfg;

    /* Push-pull on TX, and SION on RX so the receiver is enabled regardless of
     * mux mode - CAN RX reads nothing without it. */
    bb_pin_mux(&s_tx, BB_PAD_PUSHPULL, false);
    bb_pin_mux(&s_rx, BB_PAD_INPUT, true);

    FLEXCAN_GetDefaultConfig(&cfg);
    cfg.bitRate            = s.bitrate;
    cfg.enableListenOnlyMode = s.listen_only;
    /* Self-reception off: hearing our own frames back would double-count every
     * transmission in the RX ring and make `CAN:RX?` after a `CAN:TX` look like
     * the bus echoed. */
    cfg.disableSelfReception = true;

    FLEXCAN_Deinit(BB_CAN_INSTANCE);
    FLEXCAN_Init(BB_CAN_INSTANCE, &cfg, CLOCK_GetRootClockFreq(BB_CAN_CLKROOT));

    /* Accept everything: a sniffer that filters by default hides the frame you
     * were looking for. Masking is the host's business. */
    FLEXCAN_SetRxMbGlobalMask(BB_CAN_INSTANCE, 0u);

    flexcan_rx_mb_config_t rx;
    memset(&rx, 0, sizeof rx);
    rx.id     = 0u;
    rx.format = s.ext_filter ? kFLEXCAN_FrameFormatExtend
                             : kFLEXCAN_FrameFormatStandard;
    rx.type   = kFLEXCAN_FrameTypeData;
    FLEXCAN_SetRxMbConfig(BB_CAN_INSTANCE, CAN_RX_MB, &rx, true);

    s_head = s_tail = 0u;
    s.configured = true;
    return true;
}

void bb_can_init(void)
{
    /* Not brought up at boot. Configuring CAN claims GPIO_AON_06/07, and on
     * this board those pads are shared with SAI1 on J51 through jumpers J44/J46
     * - taking them before anyone asked would quietly change what J51 carries. */
    s.configured = false;
}

/* ---- polling ----------------------------------------------------------- */

void bb_can_poll(void)
{
    flexcan_frame_t f;

    if (!s.configured) {
        return;
    }
    /* Drained from the poll hook rather than an interrupt. A frame sits in the
     * message buffer until read and FlexCAN sets an overrun flag if a second
     * arrives first, so nothing is silently lost - it is counted. */
    while (FLEXCAN_ReadRxMb(BB_CAN_INSTANCE, CAN_RX_MB, &f) == kStatus_Success) {
        uint8_t next = (uint8_t)((s_head + 1u) % CAN_RX_RING);
        if (next == s_tail) {
            s.rx_dropped++;   /* newest dropped; the count is reported */
            break;
        }
        s_ring[s_head] = f;
        s_head         = next;
        s.rx_frames++;
    }
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_cfg(int argc, char **argv)
{
    uint32_t bitrate;

    /* 10 kbit to 1 Mbit: classical CAN's range. CAN FD would need a data-phase
     * rate as well and a transceiver rated for it, so it is refused rather than
     * half-supported. */
    if (!bb_parse_u32(argv[1], &bitrate) || bitrate < 10000u || bitrate > 1000000u) {
        bb_reply_err(BB_ERR_RANGE, "bitrate-10000-1000000");
        return;
    }
    if (argc > 2) {
        bb_reply_err(BB_ERR_UNSUPPORTED, "canfd-not-supported-classical-only");
        return;
    }
    s.bitrate = bitrate;
    if (!can_apply()) {
        bb_reply_err(BB_ERR_HAL, "can-init-failed");
        return;
    }
    /* The DNP connector is reported on every configure, not just once. It is
     * the single most likely reason for "configured fine, no traffic", and a
     * host that reconnects should not have to have seen the boot banner. */
#if BB_CAN_NEEDS_J22
    bb_reply_okf("bitrate=%lu listen=%u note=J22-is-DNP-fit-header-and-set-J44-J46-to-1-2",
                 (unsigned long)s.bitrate, s.listen_only ? 1u : 0u);
#else
    bb_reply_okf("bitrate=%lu listen=%u",
                 (unsigned long)s.bitrate, s.listen_only ? 1u : 0u);
#endif
}

static void cmd_cfg_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("bitrate=%lu listen=%u ext=%u state=%s rx=%lu tx=%lu dropped=%lu",
                 (unsigned long)s.bitrate, s.listen_only ? 1u : 0u,
                 s.ext_filter ? 1u : 0u,
                 s.configured ? "ready" : "idle",
                 (unsigned long)s.rx_frames, (unsigned long)s.tx_frames,
                 (unsigned long)s.rx_dropped);
}

static void cmd_listen(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1u, &on)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    s.listen_only = (on != 0u);
    if (s.configured && !can_apply()) {
        bb_reply_err(BB_ERR_HAL, "listen-apply-failed");
        return;
    }
    bb_reply_ok();
}

static void cmd_ext(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1u, &on)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    s.ext_filter = (on != 0u);
    if (s.configured && !can_apply()) {
        bb_reply_err(BB_ERR_HAL, "ext-apply-failed");
        return;
    }
    bb_reply_ok();
}

static void cmd_tx(int argc, char **argv)
{
    uint32_t        id;
    uint8_t         data[8];
    size_t          len;
    flexcan_frame_t f;

    if (!bb_parse_u32(argv[1], &id)) {
        bb_reply_err(BB_ERR_NUMBER, "id");
        return;
    }
    if (!bb_parse_bytes(argc, argv, 2, data, sizeof data, &len)) {
        /* Eight, not BB_PAYLOAD_MAX. Classical CAN's payload is eight bytes and
         * refusing a ninth is better than sending the first eight and calling
         * it a success. */
        bb_reply_err(BB_ERR_TOO_LONG, "classical-can-payload-max-8");
        return;
    }
    if (!s.configured) {
        bb_reply_err(BB_ERR_BUS_STATE, "not-configured-use-CAN:CFG");
        return;
    }
    if (s.listen_only) {
        bb_reply_err(BB_ERR_BUS_STATE, "listen-only-cannot-transmit");
        return;
    }

    bool ext = (id > 0x7FFu);
    if (ext && id > 0x1FFFFFFFu) {
        bb_reply_err(BB_ERR_RANGE, "id-max-0x1FFFFFFF");
        return;
    }

    memset(&f, 0, sizeof f);
    f.id     = ext ? FLEXCAN_ID_EXT(id) : FLEXCAN_ID_STD(id);
    f.format = ext ? (uint8_t)kFLEXCAN_FrameFormatExtend
                   : (uint8_t)kFLEXCAN_FrameFormatStandard;
    f.type   = (uint8_t)kFLEXCAN_FrameTypeData;
    f.length = (uint8_t)len;
    for (size_t i = 0; i < len; i++) {
        ((uint8_t *)&f.dataWord0)[i ^ 3u] = data[i];   /* FlexCAN is word/BE */
    }

    status_t st = FLEXCAN_TransferSendBlocking(BB_CAN_INSTANCE, CAN_TX_MB, &f);
    if (st != kStatus_Success) {
        /* A good CRC with no acknowledge is a bus with one node on it, which is
         * the commonest reason a CAN link will not come up. Saying that rather
         * than "error" is the whole point - the sniffer project's CAN decoder
         * makes the same distinction and for the same reason. */
        bb_reply_errf(BB_ERR_NACK, "tx-no-ack-bus-may-have-one-node err=%d", (int)st);
        return;
    }
    s.tx_frames++;
    bb_reply_ok();
}

static void cmd_rx(int argc, char **argv)
{
    uint32_t want = 8u;

    if (argc > 1 && (!bb_parse_u32_max(argv[1], CAN_RX_RING, &want) || want == 0u)) {
        bb_reply_errf(BB_ERR_RANGE, "max-1..%u", (unsigned)CAN_RX_RING);
        return;
    }
    if (!s.configured) {
        bb_reply_err(BB_ERR_BUS_STATE, "not-configured-use-CAN:CFG");
        return;
    }
    bb_can_poll();

    unsigned n = 0;
    bb_io_puts("OK");
    while (n < want && s_tail != s_head) {
        const flexcan_frame_t *f = &s_ring[s_tail];
        bool     ext = (f->format == (uint32_t)kFLEXCAN_FrameFormatExtend);
        uint32_t id  = ext ? (f->id & 0x1FFFFFFFu) : (f->id >> 18);

        bb_io_printf(" %s:%lX:", ext ? "x" : "s", (unsigned long)id);
        for (unsigned i = 0; i < f->length; i++) {
            bb_io_printf("%02X",
                         (unsigned)((const uint8_t *)&f->dataWord0)[i ^ 3u]);
        }
        s_tail = (uint8_t)((s_tail + 1u) % CAN_RX_RING);
        n++;
    }
    if (n == 0u) {
        bb_io_puts(" none");
    }
    bb_io_puts("\r\n");
}

static void cmd_rxn(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_can_poll();
    bb_reply_okf("%u dropped=%lu",
                 (unsigned)((s_head - s_tail) % CAN_RX_RING),
                 (unsigned long)s.rx_dropped);
}

static void cmd_flush(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    s_tail = s_head;
    bb_reply_ok();
}

/* ---- status ------------------------------------------------------------ */

void bb_can_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    bb_image_copy_str(out->pins, sizeof out->pins,
                      BB_CAN_PINS, strlen(BB_CAN_PINS));
    (void)snprintf(out->detail, sizeof out->detail, "%luk %s",
                   (unsigned long)(s.bitrate / 1000u),
                   s.listen_only ? "listen" : "active");
    out->state = !s.configured ? BB_STATE_ABSENT
                 : (s.rx_frames > 0u) ? BB_STATE_ACTIVE
                                      : BB_STATE_IDLE;
}

/* ---- table ------------------------------------------------------------- */

const bb_cmd_t bb_cmd_can[] = {
    { "CAN:CFG",    1, 2,    cmd_cfg,    "CAN:CFG <bitrate>" },
    { "CAN:CFG?",   0, 0,    cmd_cfg_q,  "CAN:CFG?" },
    { "CAN:LISTEN", 1, 1,    cmd_listen, "CAN:LISTEN <0|1>" },
    { "CAN:EXT",    1, 1,    cmd_ext,    "CAN:EXT <0=std|1=ext>" },
    { "CAN:TX",     1, 0xFF, cmd_tx,     "CAN:TX <id> [hex-bytes...]" },
    { "CAN:RX?",    0, 1,    cmd_rx,     "CAN:RX? [maxframes]" },
    { "CAN:RXN?",   0, 0,    cmd_rxn,    "CAN:RXN?" },
    { "CAN:FLUSH",  0, 0,    cmd_flush,  "CAN:FLUSH" },
    { NULL, 0, 0, NULL, NULL }
};
