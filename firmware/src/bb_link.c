/**
 * @file bb_link.c
 * @brief CM33 side of the CM33 <-> CM7 link, over shared memory.
 *
 * **Why not RPMsg-Lite.** The spec named RPMsg because it is what the SDK
 * ships and what the multicore examples use. Building it, a simpler thing was
 * enough and is easier to be sure of: this link carries strict
 * request/response, one outstanding request at a time, initiated only by the
 * CM33. RPMsg's virtqueues, buffer pools and name service all exist to support
 * multiple concurrent endpoints in both directions, none of which is wanted
 * here - and every one of them is state the two cores can disagree about.
 *
 * So the transport is two fixed buffers in shared memory and a pair of ready
 * flags. The CM33 writes a request, sets its flag, and polls for the reply.
 * That is about 80 lines instead of a middleware stack, and there is no state to
 * get out of step because there is only ever one message in flight.
 *
 * There is deliberately no doorbell interrupt either - see the note in
 * bb_link_request(). MU1 belongs to MCMGR, which is what starts the CM7, and
 * borrowing its general-purpose channels would make this depend on an
 * implementation detail of somebody else's middleware.
 *
 * The cost is that this cannot carry an unsolicited event from the CM7. It does
 * not need to: the capture core buffers records and the CM33 drains them with
 * `SNIFF:READ?`, exactly as the STM32 build's host drained its event queue.
 *
 * **Cache maintenance is the real difficulty, and it is designed out rather
 * than managed.** The CM7 has a data cache. If these buffers were cacheable,
 * every read of something the other core wrote would need an invalidate first
 * and every write a flush after - and a protocol that is only correct when both
 * sides remember that is a protocol that works until it does not. So the shared
 * region sits in OCRAM2 with the MPU marking it non-cacheable on both cores,
 * and the only barriers needed are the two `__DSB()`s around the ready flags,
 * which order the writes rather than maintaining a cache.
 */
#include "bb_link.h"
#include "bb_board.h"

#if BB_HAVE_CM7_IMAGE
#include "fsl_ele_base_api.h"
#endif

#include <stdio.h>
#include <string.h>

/* ---- shared layout ----------------------------------------------------- */
/*
 * Placed in the non-cacheable region both cores can see. The addresses are
 * fixed rather than negotiated because a negotiation needs a channel, and the
 * channel is what is being set up.
 *
 * OCRAM2 at 0x20500000 is used rather than the TCMs: a TCM is fast for its
 * owner and slow-to-impossible for the other core, which is the wrong trade for
 * a buffer that is by definition shared.
 */
#define BB_SHM_BASE       0x20500000u
#define BB_SHM_MAGIC      0x42425254u   /* "BBRT" */

typedef struct __attribute__((packed)) {
    uint32_t     magic;
    uint32_t     cm7_alive;     /**< the CM7 raises this whenever it is ready */
    /*
     * A free-running counter the capture core bumps every main-loop pass.
     *
     * "capture-core-not-running" was true but not useful: it cannot tell a core
     * that was never started from one that is running and failing to handshake,
     * and those need opposite fixes. A counter the controller can read settles
     * it without a debugger - and reading it needs no cooperation from the
     * capture core beyond being alive, which is the thing in question.
     */
    uint32_t     cm7_ticks;
    uint32_t     req_ready;     /**< CM33 -> CM7 */
    uint32_t     rep_ready;     /**< CM7 -> CM33 */
    bb_msg_hdr_t req_hdr;
    uint8_t      req[BB_MSG_MAX_PAYLOAD];
    bb_msg_hdr_t rep_hdr;
    uint8_t      rep[BB_MSG_MAX_PAYLOAD];
} bb_shm_t;

static volatile bb_shm_t *const shm = (volatile bb_shm_t *)BB_SHM_BASE;

static bool     s_up;
static uint32_t s_seq;

/* Whether the M7 has been given a clock. Gates every read of its memory - see
 * bb_link_beacon() for why that is a hang and not an error. */
static bool s_cm7_clocked;
static char     s_status[64];

/* A request that goes unanswered must fail, not hang. 50 ms is far longer than
 * any capture-core operation and short enough that a host does not think the
 * board has died. */
#define BB_LINK_TIMEOUT_MS  50u

/*
 * The self test and the bench are seconds of work, not milliseconds.
 *
 * 50 ms is right for every ordinary request - a configure, a start, a drain -
 * and badly wrong for two of them. selftest_run() drives thousands of edges and
 * then polls the intake ten thousand times; bench_run() calls it twelve times
 * over. Both far outrun a 50 ms deadline.
 *
 * What that looked like is worth recording, because the symptom pointed at the
 * wrong core entirely: SNIFF:BENCH timed out, the timeout marked the link down,
 * and every SNIFF:* command after it answered "capture-core-not-running" until
 * the 500 ms re-probe. The capture core was healthy throughout - its beacon
 * counter never stopped - and the fault was the controller giving up on it.
 */
#define BB_LINK_TIMEOUT_LONG_MS  8000u

static uint32_t link_timeout_ms(bb_msg_op_t op)
{
    switch (op) {
    case BB_MSG_SELFTEST:
    case BB_MSG_BENCH:
        return BB_LINK_TIMEOUT_LONG_MS;
    default:
        return BB_LINK_TIMEOUT_MS;
    }
}

/* ---- init -------------------------------------------------------------- */

void bb_link_init(void)
{
    s_up  = false;
    s_seq = 0u;
    (void)snprintf(s_status, sizeof s_status, "not-probed");

    /*
     * The CM33 owns the handshake because it is the core that releases the CM7.
     * The magic is written here, before the CM7 runs, so the capture core can
     * tell "the controller set this up" from "this is uninitialised OCRAM that
     * happens to contain the right word".
     */
    shm->magic     = BB_SHM_MAGIC;
    shm->cm7_alive = 0u;
    shm->req_ready = 0u;
    shm->rep_ready = 0u;

#if BB_HAVE_CM7_IMAGE
    /*
     * Start the capture core here, now that this image carries the one it is
     * starting.
     *
     * This used to be a deliberate command (SNIFF:CM7) and the reason was
     * sound at the time: the controller could not see what was in the CM7's
     * ITCM - it is not in this core's address map - so starting the core
     * unconditionally would have run whatever happened to be there.
     *
     * That reasoning no longer applies. This image contains the capture image,
     * copies it in, and reads the copy back before releasing the core, so it
     * knows exactly what it is starting. Leaving it manual just meant every
     * SNIFF:* command answered "capture-core-not-running" on a board that was
     * perfectly capable - which is how a host library and a GUI see it, and
     * fourteen of the host suite's failures were exactly that.
     *
     * The result is deliberately ignored. A controller that refused to finish
     * booting because the capture core did not come up would take USB, the
     * console and all five bus drivers down with it, for a subsystem that
     * reports its own absence perfectly well. SNIFF:CM7 remains, for retrying.
     */
    (void)bb_link_start_cm7();
#endif
}

bool bb_link_up(void)
{
    return s_up;
}

/* ---- releasing the capture core ---------------------------------------- */
/*
 * Nothing was releasing the CM7, and that was a hole rather than an omission:
 * the capture core could only ever be started by a debugger, so `SNIFF:*`
 * answered "capture-core-not-running" on every board that had not had a second
 * LinkServer session pointed at it. Worse, the two loads fight - the CM7 connect
 * script halts the M33 to wake the M7, so loading the capture image second kills
 * a running controller, and loading it first means the controller's own reset
 * re-asserts M7 reset and stops it again. Either order leaves one core dead.
 *
 * The sequence below is the RT1180 one, taken from the SDK's own MCMGR port
 * (middleware/multicore/mcmgr/src/mcmgr_internal_core_api_imxrt1180.c,
 * mcmgr_start_core_internal). Two things about it are not guessable and are the
 * reason it is copied rather than derived:
 *
 *   - The M7 cannot be released by clearing its WAIT bit alone. The EdgeLock
 *     Enclave has to be asked first, with the command word 0x17d20106 through
 *     the S3MU mailbox, and its two response words must be read or the mailbox
 *     stays full.
 *   - The M7 clock has to be *off* across the WAIT deassertion and switched back
 *     on afterwards. Deasserting WAIT with the clock running does not reliably
 *     start the core.
 *
 * MCMGR itself is not used, for two specific reasons rather than preference.
 * It adds around 3 KB to an image already at 96% of its 127 KB TCM, and
 * MCMGR_Init() claims MU1 and its interrupt - the very entanglement this link
 * was designed to avoid (see the note on the doorbell above). What is needed
 * here is one shot at core release, not a messaging framework.
 *
 * On this part MCMGR ignores the boot address it is given, and so does this: the
 * M7 boots from its own VTOR, which for the capture image is ITCM 0x00000000.
 * That is the same constraint the CM7 connect script encodes, and it is why the
 * capture profile links there.
 */

/* Last mailbox state seen by a release attempt, for SNIFF:CM7 to report. A
 * bare "the enclave did not answer" cannot distinguish a mailbox that is
 * unreachable from one that is busy, and those need different fixes. */
static uint32_t s_ele_tsr, s_ele_rsr;
#if BB_HAVE_CM7_IMAGE
/* Only the build that carries a capture image ever releases the core, so this
 * is the one piece of that state that has nothing to report otherwise. The
 * register captures above stay unconditional because bb_link_m7_state() and
 * bb_link_ele_state() are compiled either way - a diagnostic that vanishes in
 * half the builds is a diagnostic you cannot rely on. */
static bool     s_ele_ack;
#endif

/*
 * The M7's control state around a release attempt.
 *
 * The beacon settled that the capture core is not executing, and the copy into
 * its ITCM verified - so what is left is the release itself, and the release is
 * three registers. Reporting them beats reasoning about them: M7_CFG carries
 * both the initial VTOR and the CPU_WAIT bit that holds the core, and SRC's SCR
 * carries the boot release. Read before and after, they say whether WAIT was
 * ever set, whether clearing it took, and whether Prepare_CM7() ran at all.
 */
static uint32_t s_m7cfg_before, s_m7cfg_after, s_src_scr;
static uint32_t s_m7_root_hz;
static int      s_apc_status = -1;
static uint32_t s_itcm_w0, s_itcm_w1;

void bb_link_m7_clock(uint32_t *root_hz, int *apc_status)
{
    if (root_hz    != NULL) { *root_hz    = s_m7_root_hz; }
    if (apc_status != NULL) { *apc_status = s_apc_status; }
}

void bb_link_m7_state(uint32_t *cfg_before, uint32_t *cfg_after,
                      uint32_t *src_scr, uint32_t *itcm_w0, uint32_t *itcm_w1)
{
    if (cfg_before != NULL) { *cfg_before = s_m7cfg_before; }
    if (cfg_after  != NULL) { *cfg_after  = s_m7cfg_after; }
    if (src_scr    != NULL) { *src_scr    = s_src_scr; }
    if (itcm_w0    != NULL) { *itcm_w0    = s_itcm_w0; }
    if (itcm_w1    != NULL) { *itcm_w1    = s_itcm_w1; }
}

void bb_link_ele_state(uint32_t *tsr, uint32_t *rsr)
{
    if (tsr != NULL) { *tsr = s_ele_tsr; }
    if (rsr != NULL) { *rsr = s_ele_rsr; }
}

/*
 * The controller's window onto the CM7's ITCM.
 *
 * The CM7 boots from its own vector table at 0x00000000, which is its ITCM, and
 * that same memory appears at 0x303C0000 in *this* core's map. The value is not
 * invented: it is CORE1_BOOT_ADDRESS from the SDK's own board support for this
 * board (examples/_boards/frdmimxrt1186/multicore_examples/hello_world/cm33/
 * app.h), which pairs it with CORE1_KICKOFF_ADDRESS 0x0 - the two halves of the
 * same aliasing.
 *
 * It is checked rather than trusted. The copy is read back before the core is
 * released, so a wrong window is reported as a copy failure instead of starting
 * the CM7 on whatever else lives at that address.
 */
/*
 * The CM7's initial vector table, as this core programs it.
 *
 * 0x00000000 is the CM7's own view of its ITCM, which is where the capture
 * profile links. It is the SDK's CORE1_KICKOFF_ADDRESS for this board, and it
 * has to agree with three other things: the capture linker script, the
 * BB_CM7_ITCM_WINDOW copy below, and the CM7 connect script used when the
 * capture image is loaded by a debugger instead. On rev A0 the CM7's VTOR
 * cannot be changed once set, so this is the only chance to get it right.
 */
#define BB_CM7_KICKOFF_VTOR  0x00000000u

#define BB_CM7_ITCM_WINDOW   0x303C0000u

#if BB_HAVE_CM7_IMAGE
extern const uint8_t  bb_cm7_image[];
extern const uint32_t bb_cm7_image_size;
#endif

/*
 * Runs from SystemInit(), before main() and before anything in this project.
 *
 * Prepare_CM7() is the SDK's own routine in system_MIMXRT1186_cm33.c and it does
 * four things that nothing else does: powers up the PHY LDO, the ARM PLL and
 * DCDC that the CM7 runs from, programs the CM7's initial VTOR, releases it from
 * boot reset through SRC, and clears all four CM7 TCM windows by DMA.
 *
 * That last one is why this is not optional and why it has to be this early.
 * The TCM is ECC memory, and reading or writing it before it has been
 * initialised raises an ECC error - so the copy in bb_link_start_cm7() bus
 * faulted on its first write to 0x303C0000, taking the command loop down while
 * USB stayed enumerated. It looked like an access-permission problem. It was
 * uninitialised ECC.
 *
 * It also explains the silent EdgeLock Enclave: the release command was being
 * sent with the M7's power domain and PLL still down.
 *
 * Only in the build that carries a capture image. Powering the CM7 domain in a
 * controller that has nothing to run on it would be spending power for nothing.
 */
#if BB_HAVE_CM7_IMAGE
void SystemInitHook(void)
{
    Prepare_CM7(BB_CM7_KICKOFF_VTOR);
}
#endif

bb_cm7_start_t bb_link_start_cm7(void)
{
    if (shm->cm7_alive != 0u) {
        return BB_CM7_ALREADY_UP;
    }

#if !BB_HAVE_CM7_IMAGE
    /* Built without a capture image. Reported rather than attempted: releasing
     * the core with nothing loaded is how you get a second core executing
     * whatever the last run left behind. */
    return BB_CM7_NO_IMAGE;
#else
    if (bb_cm7_image_size == 0u) {
        return BB_CM7_NO_IMAGE;
    }

    /*
     * No cache maintenance. On this part the SDK's own multicore example leaves
     * APP_INVALIDATE_CACHE_FOR_SECONDARY_CORE_IMAGE_MEMORY undefined, so the
     * window is not cached from this side; the barrier is here to order the copy
     * against the release that follows, not to clean a cache.
     */
    /*
     * Give the M7 a clock before touching its TCM or letting it go.
     *
     * The LPCG first, because the TCM windows are that core's own memory and an
     * unclocked core does not answer for them.
     *
     * Then the root clock, because nothing else sets it: both places in the
     * board's clock_config.c that configure kCLOCK_Root_M7 sit inside
     * `#if (__CORTEX_M == 7)`, so a controller running BOARD_BootClockRUN()
     * configures its own root and leaves the M7's alone. The capture image would
     * set it in its own BOARD_InitBootClocks - but it has to be executing to do
     * that, and it cannot execute without a clock.
     *
     * OscRC400M because BOARD_BootClockRUN() has already enabled it and it needs
     * no PLL to be locked. It is only what the core starts on; the capture image
     * switches itself to the ARM PLL as soon as it runs, which is where its
     * 800 MHz comes from.
     */
    clock_root_config_t m7root;

    CLOCK_EnableClock(kCLOCK_M7);

    m7root.clockOff = false;
    m7root.mux      = (uint8_t)kCLOCK_M7_ClockRoot_MuxOscRc400M;
    m7root.div      = 1u;
    CLOCK_SetRootClock(kCLOCK_Root_M7, &m7root);
    s_m7_root_hz = CLOCK_GetRootClockFreq(kCLOCK_Root_M7);
    /*
     * From here the M7's memory windows answer, and not before. Recorded because
     * the beacon lives in that core's ITCM and reading it early does not fail -
     * it stalls. See bb_link_beacon().
     */
    s_cm7_clocked = true;
    __DSB();

    (void)memcpy((void *)BB_CM7_ITCM_WINDOW, bb_cm7_image, bb_cm7_image_size);
    __DSB();

    if (memcmp((const void *)BB_CM7_ITCM_WINDOW, bb_cm7_image,
               bb_cm7_image_size) != 0) {
        return BB_CM7_COPY_FAILED;
    }

    s_ele_tsr = MU_RT_S3MUA->TSR;
    s_ele_rsr = MU_RT_S3MUA->RSR;

    /*
     * Ask the enclave, but do not make the core's fate depend on it.
     *
     * SystemInitHook() has already run Prepare_CM7(), which powers the domain,
     * programs the VTOR and releases the M7 from boot reset through SRC. What is
     * left is the CPU_WAIT bit. MCMGR's port pings the enclave first and this
     * follows it, but an enclave that does not answer is not a reason to leave
     * the core held - the earlier version returned at this point and so never
     * cleared WAIT at all, which is why nothing ever started.
     *
     * The outcome says which path it took, because "started" and "started
     * without the enclave acknowledging" are different facts and the second one
     * is worth knowing about.
     */
    status_t apc    = ELE_BaseAPI_EnableAPC(MU_RT_S3MUA);
    bool     ele_ok = (apc == kStatus_Success);

    s_apc_status = (int)apc;

    s_ele_rsr = MU_RT_S3MUA->RSR;
    s_ele_ack = ele_ok;

    /* Recorded before the change, and the ITCM read back through the window the
     * CM7 will actually fetch from - the copy verify above compares against the
     * source, which proves the write landed but not that the window is ITCM. */
    s_m7cfg_before = BLK_CTRL_S_AONMIX->M7_CFG;
    s_itcm_w0      = ((const volatile uint32_t *)BB_CM7_ITCM_WINDOW)[0];
    s_itcm_w1      = ((const volatile uint32_t *)BB_CM7_ITCM_WINDOW)[1];

    CLOCK_DisableClock(kCLOCK_M7);
    BLK_CTRL_S_AONMIX->M7_CFG =
        (BLK_CTRL_S_AONMIX->M7_CFG & ~BLK_CTRL_S_AONMIX_M7_CFG_WAIT_MASK)
        | BLK_CTRL_S_AONMIX_M7_CFG_WAIT(0);
    CLOCK_EnableClock(kCLOCK_M7);
    __DSB();

    s_m7cfg_after = BLK_CTRL_S_AONMIX->M7_CFG;
    s_src_scr     = SRC_GENERAL_REG->SCR;

    /*
     * Give it a moment to reach its main loop and raise the flag. The capture
     * core's startup is a memset, a clock read and a spin on the mailbox magic,
     * so this is generous - and if it does not appear, saying so is more useful
     * than reporting success and leaving SNIFF:* to fail later with a vaguer
     * message.
     */
    for (uint32_t i = 0; i < 200000u; i++) {
        if (shm->cm7_alive != 0u) {
            return s_ele_ack ? BB_CM7_STARTED : BB_CM7_STARTED_NO_ELE;
        }
    }
    return BB_CM7_NO_RESPONSE;
#endif /* BB_HAVE_CM7_IMAGE */
}

/*
 * The capture core's beacon, read through the ITCM window.
 *
 * Deliberately not the mailbox. The mailbox is shared and cached differently on
 * each side, so when it fails every diagnostic built on it fails with it - and
 * "the core never started" then looks identical to "the core is running and
 * cannot see the mailbox". Those need opposite fixes.
 */
#define BB_BEACON_WINDOW  (BB_CM7_ITCM_WINDOW + 0x00010000u)
#define BB_BEACON_MAGIC   0x43374245u

/*
 * The beacon, or zeros if the core whose memory it lives in has no clock yet.
 *
 * The guard is not defensive tidiness. BB_BEACON_WINDOW is this core's window
 * onto the *M7's* ITCM, and an unclocked core does not answer for its own
 * memory: the read does not fault or return garbage, it stalls the AHB
 * indefinitely - no fault, no handler, the core simply stops. That is the same
 * trap i2c_apply() documents for an ungated LPI2C, and it is worth stating twice
 * because nothing about the expression `b[0]` suggests it can hang.
 *
 * It is reachable, which is how it was found. The RAM target is built without an
 * embedded capture image, so it never starts the M7 - and `SNIFF:LINK?`, whose
 * entire job is to report whether that core is alive, read the beacon to find
 * out and hung the controller instead of answering. A diagnostic that kills the
 * device when the thing it diagnoses is absent is worse than no diagnostic.
 *
 * Callers distinguish "not clocked" from "clocked and quiet" through
 * bb_link_cm7_clocked() rather than by guessing from a zero.
 */
bool bb_link_cm7_clocked(void)
{
    return s_cm7_clocked;
}

void bb_link_beacon(uint32_t *magic, uint32_t *ticks, uint32_t *saw_magic)
{
    const volatile uint32_t *b = (const volatile uint32_t *)BB_BEACON_WINDOW;

    if (magic     != NULL) { *magic     = 0u; }
    if (ticks     != NULL) { *ticks     = 0u; }
    if (saw_magic != NULL) { *saw_magic = 0u; }
    if (!s_cm7_clocked) {
        return;
    }
    if (magic     != NULL) { *magic     = b[0]; }
    if (ticks     != NULL) { *ticks     = b[1]; }
    if (saw_magic != NULL) { *saw_magic = b[2]; }
}

uint32_t bb_link_beacon_phase(void)
{
    if (!s_cm7_clocked) {
        return 0u;      /* see bb_link_beacon(): the read would stall, not fail */
    }
    return ((const volatile uint32_t *)BB_BEACON_WINDOW)[3];
}

void bb_link_raw(uint32_t *magic, uint32_t *alive, uint32_t *ticks)
{
    if (magic != NULL) { *magic = shm->magic; }
    if (alive != NULL) { *alive = shm->cm7_alive; }
    if (ticks != NULL) { *ticks = shm->cm7_ticks; }
}

const char *bb_link_status(void)
{
    return s_status;
}

/* ---- transport --------------------------------------------------------- */

bb_cap_err_t bb_link_request(bb_msg_op_t op,
                             const void *req, size_t req_len,
                             void *reply, size_t reply_cap, size_t *reply_len)
{
    if (reply_len != NULL) {
        *reply_len = 0u;
    }
    if (req_len > BB_MSG_MAX_PAYLOAD) {
        return BB_CAP_ERR_RANGE;
    }
    /* PING is allowed through while the link is down - it is how it comes up. */
    if (!s_up && op != BB_MSG_PING) {
        return BB_CAP_ERR_UNSUPPORTED;
    }
    if (shm->cm7_alive == 0u && op != BB_MSG_PING) {
        return BB_CAP_ERR_UNSUPPORTED;
    }

    shm->rep_ready = 0u;
    if (req != NULL && req_len > 0u) {
        memcpy((void *)shm->req, req, req_len);
    }
    shm->req_hdr.op     = (uint8_t)op;
    shm->req_hdr.status = 0u;
    shm->req_hdr.length = (uint16_t)req_len;
    shm->req_hdr.seq    = ++s_seq;

    __DSB();
    shm->req_ready = 1u;
    __DSB();

    /*
     * No doorbell interrupt. The obvious design is an MU general-purpose
     * interrupt, and the first version did that - but MU1 is MCMGR's, which is
     * what starts the CM7 in the first place, and sharing its interrupts means
     * depending on which GP channels MCMGR happens not to use. That is a
     * dependency on an implementation detail of somebody else's middleware.
     *
     * So the CM7 polls `req_ready` instead. The cost is real and small: its
     * idle loop reads one non-cacheable word. The capture path is FlexIO plus
     * eDMA and spends no CPU per edge, so there is nothing for that poll to
     * steal - which is exactly why this trade is affordable here and would not
     * have been on the STM32, where the capture *was* the CPU.
     */

    uint32_t budget   = link_timeout_ms(op);
    uint32_t deadline = HAL_GetTick() + budget;

    while (shm->rep_ready == 0u) {
        if ((int32_t)(HAL_GetTick() - deadline) >= 0) {
            shm->req_ready = 0u;
            s_up = false;
            (void)snprintf(s_status, sizeof s_status, "timeout-after-%ums",
                           (unsigned)budget);
            /*
             * A timeout, reported as one. This returned UNSUPPORTED, which is a
             * different claim entirely: a caller saw "ping-failed-unsupported"
             * and "load-the-cm7-image" for a core that was running, answering
             * its beacon, and merely slower than the budget allowed. Blaming a
             * missing image for a late answer sends the reader to the wrong
             * problem, which cost real time here.
             */
            return BB_CAP_ERR_TIMEOUT;
        }
    }
    __DSB();

    /* A reply whose sequence does not match is the answer to a question that
     * already timed out. Accepting it would attribute one command's result to
     * the next one, which is a class of bug that is very hard to see. */
    if (shm->rep_hdr.seq != s_seq) {
        shm->rep_ready = 0u;
        return BB_CAP_ERR_BUSY;
    }

    size_t n = shm->rep_hdr.length;
    if (n > BB_MSG_MAX_PAYLOAD) {
        n = BB_MSG_MAX_PAYLOAD;
    }
    if (reply != NULL && reply_cap > 0u) {
        size_t copy = (n < reply_cap) ? n : reply_cap;
        memcpy(reply, (const void *)shm->rep, copy);
        if (reply_len != NULL) {
            *reply_len = copy;
        }
    }
    bb_cap_err_t st = (bb_cap_err_t)shm->rep_hdr.status;
    shm->rep_ready = 0u;
    return st;
}

/* ---- liveness ---------------------------------------------------------- */

void bb_link_poll(void)
{
    static uint32_t next_probe_ms;

    if (s_up) {
        return;         /* a failed request marks it down; no need to probe */
    }
    /* Probed rather than assumed, and not on every pass: a down link probed at
     * command-loop rate would spend the whole timeout budget doing it. */
    if ((int32_t)(HAL_GetTick() - next_probe_ms) < 0) {
        return;
    }
    next_probe_ms = HAL_GetTick() + 500u;

    if (shm->cm7_alive == 0u) {
        (void)snprintf(s_status, sizeof s_status, "cm7-not-running");
        return;
    }
    uint8_t      rep[16];
    size_t       n  = 0u;
    bb_cap_err_t st = bb_link_request(BB_MSG_PING, NULL, 0u, rep, sizeof rep, &n);
    if (st == BB_CAP_OK) {
        s_up = true;
        (void)snprintf(s_status, sizeof s_status, "up");
    } else {
        (void)snprintf(s_status, sizeof s_status, "ping-failed-%s",
                       bb_cap_err_name(st));
    }
}
