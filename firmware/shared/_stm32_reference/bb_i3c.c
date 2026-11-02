/**
 * @file bb_i3c.c
 * @brief I3C in either role: as controller, dynamic address assignment, CCCs,
 *        private transfers, legacy I2C traffic on a mixed bus and in-band
 *        interrupt capture; as target, answering ENTDAA, serving private reads
 *        and writes, and raising IBIs and hot-join requests.
 *
 * The role is a mode switch (I3C:ROLE), not two things running at once: there
 * is one I3C1 and its register map means different things on each side of the
 * bus. Commands for the other role are refused rather than silently ignored.
 *
 * Bus timing comes from ST's I3C_CtrlTimingComputation utility rather than
 * hand-rolled register maths: it knows the open-drain/push-pull split and the
 * mixed-bus constraints that a pure ratio calculation gets wrong.
 *
 * The controller keeps its own table of assigned targets.  The peripheral has
 * only four DEVRx slots (so at most four targets can have IBIs acknowledged in
 * hardware), but the table can hold more for plain addressing - that limit is
 * silicon, not software, and I3C:TARGETS? reports which entries got a slot.
 *
 * The bus is on PD12/PD13 (D29/D28) with AF5.  I3C1 can be mapped to five
 * different pin pairs and the alternate function is not the same on all of
 * them, so never change the pins without checking the AF alongside.
 */
#include "bb_i3c.h"
#include "bb_board.h"
#include "bb_status.h"
#include "bb_cmd.h"
#include "bb_console.h"
#include "bb_io.h"
#include "stm32h5xx_util_i3c.h"
#include <stdio.h>
#include <string.h>

#define I3C_TIMEOUT_MS         1000u
#define I3C_MAX_TARGETS           8u   /* table size; only 4 get DEVRx slots */
#define I3C_HW_DEVICE_SLOTS       4u
#define I3C_IBI_QUEUE            16u
#define I3C_IBI_PAYLOAD_MAX       4u

/* The active controller needs a dynamic address of its own, and 0x00 is not
 * one: it is reserved, and leaving DEVR0 at DAVAL=1 with DA=0x00 is what the
 * default zero-initialised config produces. 0x08 is the first address that is
 * neither reserved nor special. Targets get 0x09 upwards so the controller's
 * own address is never handed out twice. */
#define I3C_CONTROLLER_ADDR    0x08u
#define I3C_FIRST_DYN_ADDR     0x09u

/* How long the arm path waits for an abort it started to finish. Generous
 * against the microseconds an abort actually takes, because the cost of
 * waiting is a slow command and the cost of not waiting was a target that
 * stopped answering. */
#define I3C_TGT_ABORT_WAIT_MS 25u

/* Broadcast CCC codes used by name here; everything else is passed by value. */
#define CCC_RSTDAA             0x06u
#define CCC_ENTDAA             0x07u

static I3C_HandleTypeDef hi3c;

/* GPDMA channel that feeds the target's replies into the transmit FIFO.
 *
 * Only the transmit direction is DMA. A reply has a length the host chose, so
 * it suits a block transfer exactly; a private write's length is chosen by the
 * controller and is not known until the bytes arrive, which is why the receive
 * side stays hand-fed. */
static DMA_HandleTypeDef hdma_i3c_tx;


/* Control/status FIFO scratch: the HAL builds register words in these. */
static uint32_t ctrl_words[8];
static uint32_t status_words[8];

typedef struct {
    uint8_t  dyn_addr;
    uint8_t  bcr;
    uint8_t  dcr;
    uint64_t payload;                   /* raw ENTDAA payload, as received */
    bool     ibi_capable;
    uint8_t  hw_slot;                   /* 1..4, or 0 if it did not get one */
} target_t;

static target_t targets[I3C_MAX_TARGETS];
static uint8_t  target_count;

typedef struct {
    uint8_t addr;
    uint8_t len;
    uint8_t data[I3C_IBI_PAYLOAD_MAX];
} ibi_event_t;

static volatile ibi_event_t ibi_queue[I3C_IBI_QUEUE];
static volatile uint8_t     ibi_head;
static volatile uint8_t     ibi_tail;
static volatile uint32_t    ibi_dropped;

/* Which pin pair the one I3C controller is using. There is a single I3C
 * peripheral on both supported parts, so this selects where the bus comes out,
 * not which of several controllers to talk to. */
static uint8_t  i3c_port   = 1u;

/* Controller or target. The peripheral is one or the other - there is a
 * single I3C1 and the register map means different things in each role - so
 * this is a mode switch, not a capability that can be held open both ways. */
typedef enum { BB_I3C_CONTROLLER = 0, BB_I3C_TARGET = 1 } i3c_role_t;

static i3c_role_t i3c_role = BB_I3C_CONTROLLER;

/* ---- target-mode state ---------------------------------------------------
 *
 * A target is passive: the controller decides when to read and when to write,
 * so both directions have to be standing ready rather than started by a
 * command. Receives are re-armed from the completion callback and land in a
 * ring buffer; transmits are armed by the host and consumed when the
 * controller next reads.
 */
#define I3C_TGT_RX_RING         256u
#define I3C_TGT_TX_MAX           64u

static volatile uint8_t  tgt_rx_ring[I3C_TGT_RX_RING];
static volatile uint16_t tgt_rx_head;
static volatile uint16_t tgt_rx_tail;
static volatile uint32_t tgt_rx_lost;     /* bytes dropped: host too slow */
static volatile uint32_t tgt_errors;      /* bus errors seen as a target      */
/* Reads the controller asked for when nothing was armed to answer with.
 *
 * Not a fault, and worth keeping out of the error count: the controller clocks
 * a read, this target has no reply loaded, and the peripheral reports a
 * transmit underrun. The controller is told properly - it sees the address go
 * unacknowledged - and a device that has not been given anything to say is
 * behaving correctly by saying nothing. Counting it as an error made a working
 * bus look broken every time a script read before it armed. */
static volatile uint32_t tgt_underruns;
/* Times the arm path had to finish an abort the HAL left hanging. Reported so
 * a target that keeps needing it does not look like one that never did. */
static volatile uint32_t tgt_forced_idle;
static volatile uint32_t tgt_last_ser;   /* SER behind the last error     */
static volatile uint32_t tgt_last_inframe;/* bytes taken in that frame     */
static void tgt_note_error(uint32_t ser);
/* Defined with the target table, used by i3c_apply() above it. */
static bool program_device_slots(void);
static void tgt_listen_rx(void);
/* Event flags dropped because nothing else cleared them. Should stay at zero;
 * a rising count means an event this firmware does not understand is arriving. */
static volatile uint32_t tgt_stuck_events;
static volatile uint32_t tgt_writes;      /* completed private writes seen */
static volatile uint32_t tgt_reads;       /* completed private reads served */

/* Bytes taken from RDR since the last frame completed. A private write is
 * whatever arrived between two frame-complete events, which is how a write is
 * counted without knowing its length beforehand. */
static volatile uint16_t tgt_rx_in_frame;
/* Whether the receive interrupts are enabled. Not "armed" - nothing is armed
 * any more - just whether this target is listening. */
static volatile bool     tgt_rx_live;
/* Whether a controller on this bus has actually assigned us an address.
 *
 * Not the same as DEVR0's validity bit, which survives a role change and
 * cannot reliably be cleared here: writes to it are ignored unless the
 * peripheral is disabled, and zeroing the whole register stops the target
 * acknowledging the bus at all. So the peripheral keeps whatever it kept, and
 * this records what actually happened - a target that has not been discovered
 * reports no address, which is the truth a caller needs. */
static volatile bool     tgt_addr_assigned;

static uint8_t           tgt_tx_buf[I3C_TGT_TX_MAX];
static I3C_XferTypeDef   tgt_tx_xfer;
static volatile bool     tgt_tx_armed;
static uint8_t           tgt_tx_len;
/* How much of tgt_tx_buf has been pushed into the FIFO. */
static volatile uint8_t  tgt_tx_pos;

/* Identity. The vendor half of the PID is ST's and fixed in silicon; these are
 * the parts a device gets to choose. 0xC6 is "sensor" in the MIPI DCR list,
 * picked because it is a plausible thing to find on a bus rather than 0x00. */
static uint8_t tgt_dcr      = 0xC6u;
static uint8_t tgt_instance = 0x01u;

/* Whether the target arms a hot-join request. Off by default, and the reason
 * is worth stating: a target with HJEN set announces itself by hot-joining
 * rather than by answering ENTDAA, and the two are exclusive here. Armed by
 * default, the target is invisible to ENTDAA - which shows up as a discovery
 * that completes cleanly and finds nothing, with no error at either end. */
static bool    tgt_hotjoin;

static uint32_t i3c_pp_hz  = 12500000UL;  /* push-pull, I3C SDR */
static uint32_t i2c_od_hz  =   400000UL;  /* open-drain, legacy I2C on the bus */
static uint32_t bus_type   = I3C_MIXED_BUS;
/* What i3c_pick_kernel() settled on, for the reply.
 *
 * The frequency is cached rather than asked for on demand.
 * HAL_RCCEx_GetPeriphCLKFreq() walks the RCC tree, and how long that takes
 * depends on which PLL the clock came from and what state that PLL is in - it
 * was measured at 7 ms in a healthy state and long enough to blow a 2 s
 * transport timeout after a run had left the peripheral half-configured. A
 * status query has no business being unbounded, and the answer is already
 * known at the moment the clock is selected. */
static const char *i3c_kernel_name = "hsi-64M";
/* Seeded with the boot default, which bb_board sets, so a query before the
 * peripheral has ever been configured reports what is true rather than zero. */
static uint32_t    i3c_kernel_hz    = HSI_VALUE;
/* True when the only clock that could express the requested rates leaves one
 * half of the push-pull bit a single cycle long. Such a bus works or does not
 * depending on the pull-ups, so it is named as what it is. */
static bool        i3c_kernel_marginal = false;
static bool     configured;
static bool     notifications_on;

/* Buffer the IBI notification machinery hands back to the HAL. */
static uint8_t     ibi_rx_buf[I3C_IBI_PAYLOAD_MAX];
static I3C_XferTypeDef notify_xfer;

/* ---- setup ------------------------------------------------------------- */

/* Which step of i3c_apply() failed, for the error reply. */
static const char *last_init_error = "none";

/**
 * @brief Start listening for private writes.
 *
 * Enables the receive and frame-complete interrupts directly rather than
 * arming a HAL transfer, because a HAL target receive insists on being told
 * the exact byte count in advance and errors on anything shorter. A device
 * does not get to know that in advance.
 *
 * Not before the bus has assigned an address: a private write is addressed by
 * the dynamic address, so nothing can arrive until there is one.
 */
static void tgt_listen_rx(void)
{
    if (LL_I3C_IsEnabledOwnDynAddress(hi3c.Instance) != 1u) {
        tgt_rx_live = false;
        return;
    }
    tgt_rx_in_frame = 0u;
    LL_I3C_ClearFlag_FC(hi3c.Instance);
    LL_I3C_ClearFlag_ERR(hi3c.Instance);
    LL_I3C_EnableIT_RXFNE(hi3c.Instance);
    LL_I3C_EnableIT_FC(hi3c.Instance);
    /* Errors too, or the target cannot notice its own.
     *
     * Nothing here used to enable this: it was on only when the HAL had left
     * it on from some earlier transfer, so whether a bad frame was reported at
     * all depended on what the target had been doing beforehand. Fresh from
     * discovery an unserved read was reported and recovered from; after a
     * DMA-fed read the same frame raised no interrupt, the recovery never ran,
     * and the target sat wedged with a full FIFO - refusing writes and IBIs -
     * while its error count stayed reassuringly at zero. */
    LL_I3C_EnableIT_ERR(hi3c.Instance);
    tgt_rx_live = true;
}

/**
 * @brief Push as much of the armed reply as the Tx FIFO will take.
 *
 * Interrupt context, byte moves only. Stops at the armed length rather than
 * padding: if the controller asks for more than was armed it should see a short
 * transfer, not invented data.
 */
static void tgt_fill_tx(void)
{
    /* Word at a time while a whole word is left. The FIFO is a few words
     * deep, so byte writes put a quarter as much in it before the controller
     * starts clocking - and at 12.5 MHz push-pull, what is not already in the
     * FIFO when the frame starts does not make it. */
    while ((uint8_t)(tgt_tx_len - tgt_tx_pos) >= 4u &&
           LL_I3C_IsActiveFlag_TXFNF(I3C1) != 0u) {
        uint32_t word = (uint32_t)tgt_tx_buf[tgt_tx_pos]
                      | ((uint32_t)tgt_tx_buf[tgt_tx_pos + 1u] << 8)
                      | ((uint32_t)tgt_tx_buf[tgt_tx_pos + 2u] << 16)
                      | ((uint32_t)tgt_tx_buf[tgt_tx_pos + 3u] << 24);
        LL_I3C_TransmitData32(I3C1, word);
        tgt_tx_pos = (uint8_t)(tgt_tx_pos + 4u);
    }
    while (tgt_tx_pos < tgt_tx_len && LL_I3C_IsActiveFlag_TXFNF(I3C1) != 0u) {
        LL_I3C_TransmitData8(I3C1, tgt_tx_buf[tgt_tx_pos]);
        tgt_tx_pos++;
    }
    if (tgt_tx_pos >= tgt_tx_len) {
        /* TXFNF is a level: it stays asserted while the FIFO has room, so
         * leaving its interrupt enabled after the last byte is pushed means an
         * interrupt that re-fires with nothing to do, for ever. Off until the
         * next reply is armed. */
        LL_I3C_DisableIT_TXFNF(I3C1);
    }
}

/**
 * @brief Move everything the Rx FIFO holds into the ring.
 *
 * Interrupt context, and deliberately nothing but byte moves: no HAL calls and
 * no interrupt reconfiguration, so there is nothing here that can re-enter.
 * Overflow is counted rather than allowed to overwrite - a ring that silently
 * drops its oldest bytes turns "the host was slow" into "the bus corrupted
 * data".
 */
static void tgt_drain_rx(void)
{
    while (LL_I3C_IsActiveFlag_RXFNE(I3C1) != 0u) {
        uint8_t  byte = LL_I3C_ReceiveData8(I3C1);
        uint16_t next = (uint16_t)((tgt_rx_head + 1u) % I3C_TGT_RX_RING);

        if (next == tgt_rx_tail) {
            tgt_rx_lost++;
            continue;
        }
        tgt_rx_ring[tgt_rx_head] = byte;
        tgt_rx_head = next;
        if (tgt_rx_in_frame < 0xFFFFu) {
            tgt_rx_in_frame++;
        }
    }
}

/**
 * @brief Set up the GPDMA channel that feeds target replies.
 *
 * Byte-wide into TDR, matching the byte-wide FIFO threshold this driver uses.
 * Priority above the console so a reply in flight is never starved by a host
 * command, and below nothing else that matters.
 */
static bool tgt_dma_init(void)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    hdma_i3c_tx.Instance                 = GPDMA1_Channel0;
    hdma_i3c_tx.Init.Request             = GPDMA1_REQUEST_I3C1_TX;
    hdma_i3c_tx.Init.BlkHWRequest        = DMA_BREQ_SINGLE_BURST;
    hdma_i3c_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i3c_tx.Init.SrcInc              = DMA_SINC_INCREMENTED;
    hdma_i3c_tx.Init.DestInc             = DMA_DINC_FIXED;
    hdma_i3c_tx.Init.SrcDataWidth        = DMA_SRC_DATAWIDTH_BYTE;
    hdma_i3c_tx.Init.DestDataWidth       = DMA_DEST_DATAWIDTH_BYTE;
    hdma_i3c_tx.Init.Priority            = DMA_HIGH_PRIORITY;
    hdma_i3c_tx.Init.SrcBurstLength      = 1;
    hdma_i3c_tx.Init.DestBurstLength     = 1;
    hdma_i3c_tx.Init.TransferAllocatedPort =
        DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    hdma_i3c_tx.Init.TransferEventMode   = DMA_TCEM_BLOCK_TRANSFER;
    hdma_i3c_tx.Init.Mode                = DMA_NORMAL;

    if (HAL_DMA_Init(&hdma_i3c_tx) != HAL_OK) {
        return false;
    }
    if (HAL_DMA_ConfigChannelAttributes(&hdma_i3c_tx, DMA_CHANNEL_NPRIV) != HAL_OK) {
        return false;
    }
    __HAL_LINKDMA(&hi3c, hdmatx, hdma_i3c_tx);

    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

    return true;
}

void bb_i3c_dma_irq(void)
{
    HAL_DMA_IRQHandler(&hdma_i3c_tx);
}

static bool i3c_apply_target(void)
{
    I3C_TgtTimingTypeDef timing = {0};
    I3C_FifoConfTypeDef  fifo   = {0};
    I3C_TgtConfTypeDef   tgt    = {0};

    timing.clockSrcFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I3C1);

    hi3c.Instance = I3C1;
    hi3c.Mode     = HAL_I3C_MODE_TARGET;
    last_init_error = "tgt-timing-computation";
    if (I3C_TgtTimingComputation(&timing, &hi3c.Init.TgtBusCharacteristic) != SUCCESS) {
        return false;
    }
    last_init_error = "hal-init";
    if (HAL_I3C_Init(&hi3c) != HAL_OK) {
        return false;
    }

    /* No control or status FIFO here: those describe a queue of frames the
     * controller is going to issue, and a target does not issue frames. */
    fifo.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_1_4;
    fifo.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_1_4;
    fifo.ControlFifo     = HAL_I3C_CONTROLFIFO_DISABLE;
    fifo.StatusFifo      = HAL_I3C_STATUSFIFO_DISABLE;
    last_init_error = "fifo-config";
    if (HAL_I3C_SetConfigFifo(&hi3c, &fifo) != HAL_OK) {
        return false;
    }

    tgt.Identifier            = tgt_dcr;
    tgt.MIPIIdentifier        = tgt_instance;
    tgt.CtrlRoleRequest       = DISABLE;   /* not a secondary controller */
    tgt.HotJoinRequest        = tgt_hotjoin ? ENABLE : DISABLE;
    tgt.IBIRequest            = ENABLE;
    tgt.IBIPayload            = ENABLE;
    tgt.IBIPayloadSize        = HAL_I3C_PAYLOAD_4_BYTES;
    tgt.MaxReadDataSize       = I3C_TGT_TX_MAX;
    tgt.MaxWriteDataSize      = I3C_TGT_RX_RING;
    tgt.CtrlCapability        = DISABLE;
    tgt.GroupAddrCapability   = DISABLE;
    tgt.DataTurnAroundDuration = HAL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS;
    tgt.MaxReadTurnAround     = 0x00u;
    tgt.MaxDataSpeed          = HAL_I3C_GETMXDS_FORMAT_1;
    tgt.MaxSpeedLimitation    = DISABLE;
    tgt.HandOffActivityState  = HAL_I3C_HANDOFF_ACTIVITY_STATE_0;
    tgt.HandOffDelay          = DISABLE;
    tgt.PendingReadMDB        = DISABLE;
    last_init_error = "tgt-dma";
    if (!tgt_dma_init()) {
        return false;
    }

    last_init_error = "tgt-config";
    if (HAL_I3C_Tgt_Config(&hi3c, &tgt) != HAL_OK) {
        return false;
    }

    HAL_NVIC_SetPriority(I3C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I3C1_EV_IRQn);
    HAL_NVIC_SetPriority(I3C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I3C1_ER_IRQn);

    tgt_rx_head = 0u;
    tgt_rx_tail = 0u;
    tgt_rx_lost = 0u;
    tgt_errors  = 0u;
    tgt_underruns = 0u;
    tgt_forced_idle = 0u;
    tgt_last_ser = 0u;
    tgt_last_inframe = 0u;
    tgt_stuck_events = 0u;
    tgt_writes  = 0u;
    tgt_reads   = 0u;
    tgt_tx_armed = false;
    tgt_tx_len   = 0u;
    tgt_tx_pos   = 0u;
    tgt_rx_live  = false;
    tgt_rx_in_frame = 0u;
    tgt_addr_assigned = false;


    /* Listen for the events a target cannot ask for: the dynamic address
     * being assigned or reset, the read/write limits being changed by a CCC,
     * and bus errors. DEFIE and GRPIE are left out deliberately - they need a
     * receive buffer handed over up front, and this target has no defining-byte
     * or group-address role to play.
     *
     * The receive is not armed here. It cannot be: that needs an address, and
     * the address is what the controller is about to give us. It is armed when
     * the DAUPD event says we have one. */
    last_init_error = "activate-notification";
    if (HAL_I3C_ActivateNotification(&hi3c, NULL,
                                     HAL_I3C_IT_DAUPDIE | HAL_I3C_IT_MWLUPDIE |
                                     HAL_I3C_IT_MRLUPDIE | HAL_I3C_IT_GETIE |
                                     HAL_I3C_IT_STAIE | HAL_I3C_IT_INTUPDIE |
                                     HAL_I3C_IT_ASUPDIE | HAL_I3C_IT_RSTIE |
                                     HAL_I3C_IT_ERRIE) != HAL_OK) {
        return false;
    }

    last_init_error  = "none";
    configured       = true;
    notifications_on = false;
    return true;
}

/**
 * @brief Point I3C1 at whichever kernel clock can express the wanted rates.
 *
 * ST's timing computation divides this clock to build the SCL waveform, so it
 * is the resolution available - and the two ends of the bus want opposite
 * things from it. A fast push-pull rate needs cycles per bit; a slow
 * open-drain rate needs a low-period count that fits in eight bits, so the
 * slowest open-drain achievable is kernel/255.
 *
 * @return false when no available clock can do both, which is a real
 *         constraint and not a failure to try: 12.5 MHz push-pull alongside
 *         100 kHz open-drain cannot be expressed by any single divider.
 */
static bool i3c_pick_kernel(LL_I3C_CtrlBusConfTypeDef *out)
{
    RCC_PeriphCLKInitTypeDef pclk = {0};

    /* Two passes over the taps below. The first requires room in both halves of
     * the push-pull bit; the second takes whatever computes, for a rate pair
     * that nothing else can express. See the guard for what that buys. */
    for (unsigned pass = 0; pass < 2u; pass++) {

    /* Slowest first: a slower kernel keeps more of the open-drain range
     * reachable, and only the fast push-pull rates need the finer one. */
    /* Slowest first. A slower kernel reaches lower open-drain rates - the low
     * period is an eight-bit count of kernel cycles - while a faster one is
     * needed to shape a fast push-pull bit. PLL3R appears twice because its
     * divider is ours to choose: one slow tap for legacy I2C speeds on a mixed
     * bus, one fast tap for full-rate SDR. */
    static const struct {
        uint32_t sel;
        uint32_t pll3r_div;      /* 0 when the source is not PLL3 */
        const char *name;
    } sources[] = {
        { RCC_I3C1CLKSOURCE_PLL3R, 16u, "pll3r-25M"  },
        { RCC_I3C1CLKSOURCE_HSI,    0u, "hsi-64M"    },
        { RCC_I3C1CLKSOURCE_PLL3R,  4u, "pll3r-100M" },
    };

    for (unsigned i = 0; i < (sizeof sources / sizeof sources[0]); i++) {
        I3C_CtrlTimingTypeDef      timing = {0};
        LL_I3C_CtrlBusConfTypeDef  conf   = {0};

        pclk.PeriphClockSelection = RCC_PERIPHCLK_I3C1;
        pclk.I3c1ClockSelection   = sources[i].sel;
        if (sources[i].sel == RCC_I3C1CLKSOURCE_PLL3R) {
            /* Selecting PLL3R makes the HAL reconfigure PLL3 from this same
             * struct, so the PLL parameters have to travel with the choice -
             * a zero-filled PLL3 block is simply rejected, which is what the
             * first attempt at this did.
             *
             * Sourced from the HSI, not the HSE, so it is identical on both
             * boards here: they run different crystals. 64 MHz / 8 = 8 MHz
             * into the VCO, x50 = 400 MHz, / 4 = 100 MHz. */
            pclk.PLL3.PLL3Source   = RCC_PLL3_SOURCE_HSI;
            pclk.PLL3.PLL3M        = 8;
            pclk.PLL3.PLL3N        = 50;
            pclk.PLL3.PLL3P        = 2;
            pclk.PLL3.PLL3Q        = 2;
            pclk.PLL3.PLL3R        = sources[i].pll3r_div;
            pclk.PLL3.PLL3RGE      = RCC_PLL3_VCIRANGE_2;
            pclk.PLL3.PLL3VCOSEL   = RCC_PLL3_VCORANGE_WIDE;
            pclk.PLL3.PLL3FRACN    = 0;
            pclk.PLL3.PLL3ClockOut = RCC_PLL3_DIVR;
        }
        if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
            continue;
        }

        timing.clockSrcFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I3C1);
        timing.i3cPPFreq    = i3c_pp_hz;
        timing.i2cODFreq    = (bus_type == I3C_MIXED_BUS) ? i2c_od_hz : 0u;
        timing.dutyCycle    = 50u;
        timing.busType      = bus_type;
        if (I3C_CtrlTimingComputation(&timing, &conf) != SUCCESS) {
            continue;               /* this clock cannot express both rates */
        }
        /* A one-cycle push-pull low is the degenerate result: the computation
         * reports success and a target cannot follow the waveform. Ask for the
         * next clock up instead.
         *
         * The high duration needs the same guard, and did not have it. A slow
         * push-pull rate on a slow kernel came back with SCLI3CHighDuration
         * zero - one kernel cycle high against several low, a duty cycle of
         * about one in six where 50% was asked for - and the computation called
         * that success. The symptom was not a distorted waveform but a target
         * that stopped acknowledging: 4 MHz and 1 MHz push-pull both failed
         * every transfer with address-nack while 12.5 MHz, on a faster kernel,
         * was perfect. It also hid well, because switching the internal
         * pull-ups in was enough to make it work.
         *
         * The cost of this guard is that a very low open-drain rate can become
         * unreachable: the open-drain low period is an eight-bit count of
         * kernel cycles, so only a slow kernel reaches 100 kHz, and that is
         * exactly the kernel being refused here. Saying "no kernel clock fits
         * both rates" is the better answer - it is true, and it is checkable,
         * which a waveform that works until the pull-ups change is not. */
        if (pass == 0u &&
            (conf.SCLPPLowDuration < 2u || conf.SCLI3CHighDuration < 2u)) {
            continue;
        }

        i3c_kernel_name     = sources[i].name;
        i3c_kernel_hz       = timing.clockSrcFreq;
        /* Reported, not hidden: on this clock the waveform is lopsided enough
         * that whether a target follows it depends on the pull-ups. */
        i3c_kernel_marginal = (conf.SCLPPLowDuration < 2u ||
                               conf.SCLI3CHighDuration < 2u);
        if (out != NULL) {
            *out = conf;
        }
        return true;
        }
    }
    return false;
}

static bool i3c_apply(void)
{
    I3C_CtrlTimingTypeDef timing = {0};
    I3C_FifoConfTypeDef   fifo   = {0};
    I3C_CtrlConfTypeDef   ctrl   = {0};

    bb_pins_i3c(i3c_port);
    if (configured) {
        (void)HAL_I3C_DeInit(&hi3c);
        configured = false;
    }

    __HAL_RCC_I3C1_CLK_ENABLE();

    /* Hold the peripheral in reset for a moment, so every register starts from
     * its default rather than from whatever the last role left.
     *
     * This is here because of a failure that took a long time to corner: after
     * enough role and rate changes a target stopped acknowledging the broadcast
     * address and stayed that way. Discovery reported no-ack-for-7E, which
     * reads as a wiring fault, and every register this module knows about
     * looked correct - CFGR enabled, DEVR0 clean, the state machine listening.
     * Resetting the whole board fixed it; nothing short of that did, which is
     * the shape of a register nobody here is thinking about.
     *
     * HAL_I3C_DeInit() does not do this: it tears down the handle and the
     * interrupts, and leaves the peripheral's registers as they were. Asking
     * RCC to reset it does, and costs two register writes on a path that
     * already reconfigures everything. */
    __HAL_RCC_I3C1_FORCE_RESET();
    __HAL_RCC_I3C1_RELEASE_RESET();

    /* Wipe the other role's state before anything is enabled, and in this
     * order, because the order is the whole trick.
     *
     * CFGR first: it carries the enable bit, and DEVR0 is write-protected
     * while the peripheral is enabled. Clearing DEVR0 first looked right, was
     * silently ignored, and left the previous role's dynamic address in place
     * - so a board switched to the target role came up already believing it
     * had an address and took no part in discovery. The register read back the
     * old value the whole time; only reading it back proved the write had not
     * landed.
     *
     * DEVR0 second: the board's own dynamic address. In the controller role
     * this same register holds the controller's address with DAVAL set, so
     * inheriting it across a role change is exactly the trap above.
     *
     * CFGR also carries the hot-join acknowledge and the FIFO enables, which
     * the HAL only rewrites when configuring a controller - a target
     * configured after a controller inherited them and stopped acknowledging
     * the bus at all. HAL_I3C_Init() sets what it needs afterwards, so
     * starting from zero costs nothing. */
    I3C1->CFGR = 0u;
    /* The address as well as its valid bit.
     *
     * Clearing DAVAL alone left the address itself sitting in DA, and in the
     * controller role that address is 0x08 - this peripheral's own. A board
     * that had been a controller therefore entered the target role carrying
     * 0x08 with DAVAL clear, which is not a state a target should ever be in:
     * discovery got as far as the broadcast address and stopped, reporting
     * nothing acknowledged, with SER showing a protocol error on the target
     * side. It looked like missing pull-ups.
     *
     * Only these two bits. Zeroing the whole register also clears IBIEN, and a
     * target without it stops acknowledging the bus altogether - which is what
     * made a full clear look like the wrong fix when it was tried before. */
    I3C1->DEVR0 &= ~(uint32_t)(I3C_DEVR0_DAVAL | I3C_DEVR0_DA);

    if (i3c_role == BB_I3C_CONTROLLER) {
        /* And say what the controller's own address is, here, while the
         * peripheral is disabled and the register will take it.
         *
         * Clearing was not enough. A board found holding its old target address
         * with the valid bit clear - own=0x09, DAVAL=0 - drove the bus with no
         * address of its own, and every frame it started ran to the full
         * timeout instead of completing: an I3C:PROBE that should answer
         * instantly took the whole second, twice over for two trials. It looked
         * like one board being slower than the other.
         *
         * Writing it rather than clearing it and trusting the configuration
         * that follows means the register is right whatever it held before, and
         * whatever route the caller took to get here. */
        I3C1->DEVR0 |= ((uint32_t)I3C_CONTROLLER_ADDR << I3C_DEVR0_DA_Pos)
                     | (uint32_t)I3C_DEVR0_DAVAL;
    }

    if (i3c_role == BB_I3C_TARGET) {
        /* A target computes nothing from its kernel clock but the bus-available
         * time, which any of these can express, so it keeps whatever the
         * controller role last chose. */
        return i3c_apply_target();
    }

    /* No kernel clock can express both rates. The two ends pull in opposite
     * directions - a fast push-pull bit needs cycles to shape, a slow
     * open-drain bit needs a low-period count that fits in eight bits - so far
     * enough apart and nothing satisfies both. */
    last_init_error = "no-kernel-clock-fits-both-rates";
    if (!i3c_pick_kernel(&hi3c.Init.CtrlBusCharacteristic)) {
        return false;
    }

    /* Timing already computed by i3c_pick_kernel(), which had to run it to
     * find out whether a given kernel clock could express the wanted rates at
     * all. Running it twice would just risk the two disagreeing. */
    (void)timing;

    hi3c.Instance = I3C1;
    hi3c.Mode     = HAL_I3C_MODE_CONTROLLER;
    last_init_error = "hal-init";
    if (HAL_I3C_Init(&hi3c) != HAL_OK) {
        return false;
    }

    /* Word-wide FIFO access plus the control and status FIFOs, which is what
     * makes the descriptor-driven transfer API usable. */
    fifo.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_1_4;
    fifo.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_1_4;
    /* Control FIFO on, status FIFO off.
     *
     * The control FIFO is what makes the descriptor-driven API work: it is
     * where the frame's control words are queued. The status FIFO is a
     * different thing - a per-frame status word the application is expected to
     * read back - and the HAL's polling transfer functions do not read it.
     * They wait on the frame-complete flag while the peripheral waits for the
     * status word to be consumed, and neither moves. On the wire the frame
     * looks perfect and then nothing happens, which is exactly what was seen
     * as "the CCC reached the bus and timed out anyway".
     *
     * It was never noticed before because no I3C frame in this project had
     * ever completed: with no target on the bus every transfer failed at the
     * broadcast address first, which sets the error flag and breaks the same
     * wait for a different reason. */
    fifo.ControlFifo     = HAL_I3C_CONTROLFIFO_ENABLE;
    fifo.StatusFifo      = HAL_I3C_STATUSFIFO_DISABLE;
    last_init_error = "fifo-config";
    if (HAL_I3C_SetConfigFifo(&hi3c, &fifo) != HAL_OK) {
        return false;
    }

    ctrl.DynamicAddr    = I3C_CONTROLLER_ADDR;
    ctrl.StallTime      = 0;
    ctrl.HotJoinAllowed = ENABLE;
    ctrl.ACKStallState  = DISABLE;
    ctrl.CCCStallState  = DISABLE;
    ctrl.TxStallState   = DISABLE;
    ctrl.RxStallState   = DISABLE;
    ctrl.HighKeeperSDA  = DISABLE;
#if defined(I3C_TIMINGR2_STALLL)
    ctrl.ACKI2CAddrState  = DISABLE;
    ctrl.ACKI2CWriteState = DISABLE;
    ctrl.ACKI2CReadState  = DISABLE;
#endif
    last_init_error = "ctrl-config";
    if (HAL_I3C_Ctrl_Config(&hi3c, &ctrl) != HAL_OK) {
        return false;
    }

    HAL_NVIC_SetPriority(I3C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I3C1_EV_IRQn);
    /* The error interrupt stays OFF in the controller role, and that is not an
     * oversight.
     *
     * Every controller transfer here uses the HAL's polling API, whose loops
     * look like
     *
     *     I3C_ControlDataTreatment(hi3c);
     *     hi3c->ptrTxFunc(hi3c);
     *
     * while the error interrupt's handler runs I3C_ErrorTreatment(), which
     * sets ptrTxFunc and ptrRxFunc to NULL. An error arriving mid-loop makes
     * the very next iteration jump to zero: a hard fault with pc=0 in the
     * middle of an ordinary private write. That is exactly what a 32-byte
     * write at 12.5 MHz produced - a faster bus means more chance of an error
     * landing inside the loop.
     *
     * Nothing is lost by leaving it off: the polling functions read the error
     * flag themselves when they finish, which is where i3c_reply() gets the
     * code it reports. The target role does enable it, because a target has no
     * polling loop to corrupt and does need the flag cleared. */
    HAL_NVIC_DisableIRQ(I3C1_ER_IRQn);

    last_init_error  = "none";
    configured       = true;
    notifications_on = false;

    /* Hand the discovered targets back to the peripheral.
     *
     * The DEVRx slots carry which addresses may raise an acknowledged IBI, and
     * resetting the peripheral above clears them - so a rate change quietly
     * revoked the permission that discovery had granted, and the next IBI a
     * target raised went unacknowledged and timed out. The targets themselves
     * are still known; only the peripheral forgot, and it is cheap to remind
     * it. Before the reset was added this survived by accident, which is a
     * thinner guarantee than doing it on purpose. */
    if (target_count > 0u) {
        (void)program_device_slots();
    }
    return true;
}

/**
 * @brief Bring the peripheral up in @p want, or explain why it cannot be.
 *
 * Refusing rather than silently switching: a controller command that flipped
 * the peripheral out of target mode would drop the dynamic address the bus had
 * assigned, and the bus would not be told.
 */
static bool i3c_ready_as(i3c_role_t want)
{
    if (i3c_role != want) {
        bb_reply_errf(BB_ERR_BUS_STATE, "peripheral-is-%s-use-I3C:ROLE",
                      (i3c_role == BB_I3C_TARGET) ? "target" : "controller");
        return false;
    }
    if (!configured) {
        target_count = 0;
        if (!i3c_apply()) {
            bb_reply_errf(BB_ERR_HAL, "i3c-init-failed-at-%s", last_init_error);
            return false;
        }
    }
    return true;
}

static bool i3c_ready(void)
{
    return i3c_ready_as(BB_I3C_CONTROLLER);
}

void bb_i3c_init(void)
{
    /* The peripheral comes up on first use.  Nothing is shared with the I2C
     * controllers any more - I3C1 has PD12/PD13 to itself - so this is purely
     * about not driving pins nobody asked for. */
}

bool bb_i3c_configured(void)
{
    return configured;
}

void bb_i3c_release(void)
{
    if (configured) {
        (void)HAL_I3C_DeInit(&hi3c);
        configured   = false;
        target_count = 0;
    }
    __HAL_RCC_I3C1_CLK_DISABLE();
}

void bb_i3c_ev_irq(void)
{
    /* As a target, this module owns the receive path: take the bytes out of
     * the FIFO before anything else looks at the peripheral, close the frame
     * on frame-complete, and clear the error flag so it cannot re-assert the
     * interrupt for ever. The HAL handler still runs afterwards for the events
     * it does own - the dynamic address being assigned, CCCs changing our
     * limits, transmit completion. */
    /* Fast path: bytes arriving and nothing else pending.
     *
     * A private write's data phase runs at the push-pull rate, so at 12.5 MHz
     * a byte lands every 640 ns. The full path below reads several registers
     * and then hands over to the HAL's event handler, which is too much per
     * FIFO-full: a 16-byte write arrived truncated to 8 without this. When
     * RXFNE is the only thing pending there is provably nothing else to do. */
    if (i3c_role == BB_I3C_TARGET && (I3C1->EVR & I3C1->IER) == I3C_EVR_RXFNEF) {
        tgt_drain_rx();
        return;
    }

    HAL_I3C_StateTypeDef state = HAL_I3C_GetState(&hi3c);

    /* Both data directions belong to this module in the target role, because
     * the HAL's target transfers insist on knowing the byte count in advance
     * and treat any other count as an error - which a device cannot promise in
     * either direction.
     *
     * Stand aside whenever the HAL says it owns something: the IBI and
     * hot-join requests are polling calls that watch the same frame-complete
     * flag, and clearing it underneath them would hang them. */
    if (i3c_role == BB_I3C_TARGET &&
        (state == HAL_I3C_STATE_LISTEN || state == HAL_I3C_STATE_READY)) {
        uint32_t evr = I3C1->EVR;

        /* Drain unconditionally: the HAL's target event handler starts with
         * `if (RXFNEF) hi3c->ptrRxFunc(hi3c);` and that pointer is only set
         * when a HAL receive is armed, which this module never does. A byte
         * left in the FIFO for it to find is a call through a null pointer. */
        tgt_drain_rx();

        if ((evr & I3C_EVR_TXFNFF) != 0u) {
            tgt_fill_tx();
        }
        if ((evr & I3C_EVR_FCF) != 0u) {
            /* A frame ended. Whatever the DMA collected during it was one
             * write - the count is measured from the channel, since nobody
             * told us how long the write would be. */
            tgt_drain_rx();
            if (tgt_rx_in_frame > 0u) {
                tgt_writes++;
                tgt_rx_in_frame = 0u;
            }
            LL_I3C_ClearFlag_FC(I3C1);
        }
        if ((evr & I3C_EVR_ERRF) != 0u) {
            /* Collect first: a frame can error after carrying good bytes, and
             * discarding those would lose data the bus really did deliver. */
            tgt_drain_rx();
            /* Draining adds to the frame's byte count rather than resetting
             * it, which is what lets tgt_note_error() tell a write that outran
             * the receive path from a read nobody armed. It does the counting,
             * the clearing and the recovery. */
            tgt_note_error(I3C1->SER);
            LL_I3C_ClearFlag_ERR(I3C1);
            hi3c.ErrorCode = HAL_I3C_ERROR_NONE;
        }
    }

    /* Mask the receive source across the HAL's handler.
     *
     * Its target event ISR starts with `if (RXFNEF) hi3c->ptrRxFunc(hi3c);`
     * and no null check, and that pointer is only ever set when a HAL receive
     * is armed - which this module never does, because a HAL receive needs a
     * byte count nobody can give it. Draining before the call is not enough:
     * at 10 MHz a byte can land in the gap between the drain and the check,
     * and then the flag is set again and the core jumps to zero. Observed
     * exactly that way, as FAULT HardFault pc=0x00000000.
     *
     * With the source masked the flag cannot be in the handler's view at all,
     * which is a guarantee rather than a narrower race. */
    uint32_t rx_unmask = (i3c_role == BB_I3C_TARGET)
                       ? (I3C1->IER & I3C_IER_RXFNEIE) : 0u;

    if (rx_unmask != 0u) {
        LL_I3C_DisableIT_RXFNE(I3C1);
    }
    HAL_I3C_EV_IRQHandler(&hi3c);
    if (rx_unmask != 0u) {
        /* Anything that arrived while it was masked is still in the FIFO. */
        tgt_drain_rx();
        LL_I3C_EnableIT_RXFNE(I3C1);
    }

    /* Last resort, in either role: an enabled event flag that nobody cleared
     * keeps this interrupt asserted, and the core never returns to the main
     * loop. No fault, no output, the board simply stops answering - four
     * separate flags have done it during this work, and each one looked like a
     * different bug. The last was a controller sharing a bus with another
     * controller, where a stray error flag wedged the board that was not even
     * being used.
     *
     * Anything still pending once everyone has had a look at it is dropped and
     * counted. A lost event is a far smaller problem than a lost board, and
     * the count makes the loss visible rather than silent. */
    {
        uint32_t stuck = I3C1->EVR & I3C1->IER;

        if (stuck != 0u) {
            I3C1->CEVR = stuck;
            tgt_stuck_events++;
        }
    }
}

void bb_i3c_er_irq(void)
{
    HAL_I3C_ER_IRQHandler(&hi3c);
}

/* ---- IBI plumbing ------------------------------------------------------ */

/**
 * @brief HAL notification callback: an IBI, hot-join or controller-role event.
 *
 * Runs in interrupt context, so it only queues; the reply is formatted later
 * when the host asks with I3C:IBI?.
 */
void HAL_I3C_NotifyCallback(I3C_HandleTypeDef *phi3c, uint32_t eventId)
{
    I3C_CCCInfoTypeDef info = {0};

    if (i3c_role == BB_I3C_TARGET) {
        /* Getting an address is the moment a private write becomes possible,
         * so that is when this target starts listening. Enabling two interrupt
         * bits is safe here; arming a HAL transfer would not have been. A
         * RSTDAA takes the address away again and tgt_listen_rx() checks for
         * that, so the same path covers both directions. */
        if ((eventId & EVENT_ID_DAU) != 0u) {
            /* The bus has just set or cleared our address, so from here the
             * peripheral's validity bit means what it says. */
            tgt_addr_assigned = (LL_I3C_IsEnabledOwnDynAddress(hi3c.Instance) != 0u);
            tgt_listen_rx();
        }
        return;
    }

    if ((eventId & EVENT_ID_IBI) == 0u) {
        return;
    }
    if (HAL_I3C_GetCCCInfo(phi3c, EVENT_ID_IBI, &info) != HAL_OK) {
        return;
    }

    uint8_t next = (uint8_t)((ibi_head + 1u) % I3C_IBI_QUEUE);
    if (next == ibi_tail) {
        ibi_dropped++;
        return;
    }
    ibi_queue[ibi_head].addr = (uint8_t)info.IBICRTgtAddr;
    uint8_t len = (uint8_t)info.IBITgtNbPayload;
    if (len > I3C_IBI_PAYLOAD_MAX) {
        len = I3C_IBI_PAYLOAD_MAX;
    }
    ibi_queue[ibi_head].len = len;
    for (uint8_t i = 0; i < len; i++) {
        ibi_queue[ibi_head].data[i] = (uint8_t)(info.IBITgtPayload >> (8u * i));
    }
    ibi_head = next;
}

/* ---- target-mode callbacks --------------------------------------------- */

/**
 * @brief The controller finished reading the armed reply.
 *
 * Consumed, not reused: answering a second read from the same bytes would look
 * like a working device quietly serving stale data.
 */
void HAL_I3C_TgtTxCpltCallback(I3C_HandleTypeDef *phi3c)
{
    tgt_tx_armed = false;
    tgt_tx_len   = 0u;
    tgt_tx_pos   = 0u;
    tgt_reads++;

    /* Put back the two interrupts finishing a read takes away.
     *
     * The HAL's completion path leaves IER holding RXFNEIE alone: frame
     * complete and error are both switched off. A target that has served one
     * read is then deaf to the end of any later frame and to its own errors -
     * so private writes stop being counted, an unserved read raises nothing to
     * recover from, and the peripheral sits wedged with a full FIFO refusing
     * writes and IBIs while its error count stays at zero. Restored here
     * rather than in the arm path, because it has to hold whether or not
     * anything arms another reply. */
    LL_I3C_EnableIT_FC(phi3c->Instance);
    LL_I3C_EnableIT_ERR(phi3c->Instance);
}

/**
 * @brief Record a target-side error, telling the expected kind from the rest.
 *
 * A controller reading from a target that has nothing armed is not a fault:
 * the peripheral runs out of bytes to send, reports a FIFO underrun, and the
 * controller is told properly - it sees the address go unacknowledged. A
 * device that has not been given anything to say is behaving correctly by
 * saying nothing, and counting that as a bus error made a working link look
 * broken every time a script read before it armed.
 *
 * @param ser  SER as it read before anything cleared it.
 */
static void tgt_note_error(uint32_t ser)
{
    tgt_last_ser = ser;
    tgt_last_inframe = tgt_rx_in_frame;
    /* Told apart by whether the frame delivered anything.
     *
     * DOVR covers both directions - it is set when the receive FIFO overflows
     * and when the transmit FIFO runs dry - and SER and SR read identically
     * for the two cases, so the peripheral is no help here. What separates
     * them is that a write which outran the receive path has bytes to show for
     * itself, and a read nobody armed does not: at 12.5 MHz a failing 16- or
     * 32-byte write reports eight bytes taken, an unserved read reports none.
     *
     * So: nothing armed, nothing received, and the FIFO complaining means the
     * controller asked this target for something it had not been given. A
     * write that ran past the receive path is a real fault and stays one. */
    if ((ser & I3C_SER_DOVR) != 0u && !tgt_tx_armed && tgt_rx_in_frame == 0u) {
        tgt_underruns++;
    } else {
        tgt_errors++;
    }

    /* Put the target back in service.
     *
     * Any error leaves the peripheral mid-frame with whatever the failed
     * transfer left in its FIFOs, and nothing else clears that: the next
     * private write is accepted and silently dropped, and the one after is not
     * acknowledged at all. Re-arming a reply happened to clean it up, which is
     * why a scripted target looked like it recovered on its own and a target
     * left idle after one unserved read looked dead.
     *
     * The same three steps the arm path uses, minus arming anything: finish
     * the frame the peripheral still thinks it is in, empty both FIFOs, and
     * enable the receive interrupts again. */
    tgt_drain_rx();
    if (HAL_I3C_GetState(&hi3c) != HAL_I3C_STATE_LISTEN &&
        HAL_I3C_GetState(&hi3c) != HAL_I3C_STATE_READY) {
        (void)HAL_I3C_Abort_IT(&hi3c);
    }
    LL_I3C_DisableIT_TXFNF(I3C1);
    LL_I3C_RequestTxFIFOFlush(I3C1);
    LL_I3C_RequestRxFIFOFlush(I3C1);
    tgt_tx_armed = false;
    tgt_tx_len   = 0u;
    tgt_tx_pos   = 0u;
    tgt_rx_in_frame = 0u;
    tgt_listen_rx();
}

/**
 * @brief Bus error while acting as a target.
 *
 * Reached through the error interrupt rather than the event one, and its only
 * job is to keep an error from repeating for ever: while the error flag is set
 * the event interrupt stays asserted, and the HAL's target event handler does
 * not clear it. Also cleared in bb_i3c_ev_irq(), because either interrupt can
 * see it first.
 */
void HAL_I3C_ErrorCallback(I3C_HandleTypeDef *phi3c)
{
    if (i3c_role != BB_I3C_TARGET) {
        return;
    }
    /* Before the clears: they are what makes the code unreadable afterwards. */
    tgt_note_error(phi3c->Instance->SER);
    LL_I3C_ClearFlag_ERR(phi3c->Instance);
    LL_I3C_ClearFlag_FC(phi3c->Instance);
    phi3c->ErrorCode = HAL_I3C_ERROR_NONE;
}

/* ---- target table ------------------------------------------------------ */

/** Push the IBI-acknowledge policy for the known targets into DEVRx. */
static bool program_device_slots(void)
{
    I3C_DeviceConfTypeDef desc[I3C_HW_DEVICE_SLOTS];
    uint8_t               n = 0;

    for (uint8_t i = 0; i < target_count && n < I3C_HW_DEVICE_SLOTS; i++) {
        desc[n].DeviceIndex       = (uint8_t)(n + 1u);
        desc[n].TargetDynamicAddr = targets[i].dyn_addr;
        desc[n].IBIAck            = targets[i].ibi_capable ? ENABLE : DISABLE;
        desc[n].IBIPayload        = targets[i].ibi_capable ? ENABLE : DISABLE;
        desc[n].CtrlRoleReqAck    = DISABLE;
        desc[n].CtrlStopTransfer  = DISABLE;
        targets[i].hw_slot        = (uint8_t)(n + 1u);
        n++;
    }
    for (uint8_t i = n; i < target_count; i++) {
        targets[i].hw_slot = 0;
    }
    if (n == 0u) {
        return true;
    }
    return HAL_I3C_Ctrl_ConfigBusDevices(&hi3c, desc, n) == HAL_OK;
}

/**
 * @brief Turn an I3C error code into something a person can act on.
 *
 * The low nibble of SER is a CE0..CE3 / TE0..TE6 code qualified by PERR; the
 * upper bits are separate NACK and overrun flags.  CE2 in particular is the
 * one you see constantly during bring-up: it means nothing on the bus answered
 * the 0x7E broadcast, i.e. no target, or no pull-ups.
 */
static const char *i3c_error_text(uint32_t err)
{
    if (err == HAL_I3C_ERROR_NONE) {
        return "none";
    }
    if ((err & I3C_SER_ANACK) != 0u) {
        return "address-nack";
    }
    if ((err & I3C_SER_DNACK) != 0u) {
        return "data-nack";
    }
    if ((err & I3C_SER_COVR) != 0u) {
        return "fifo-overrun";
    }
    if ((err & I3C_SER_STALL) != 0u) {
        return "scl-stall";
    }
    if ((err & I3C_SER_PERR) != 0u) {
        switch (err & 0x0Fu) {
        case 0u:  return "protocol-ce0-illegal-broadcast";
        case 1u:  return "protocol-ce1-bad-tx-data";
        case 2u:  return "protocol-ce2-no-ack-for-7E";
        case 3u:  return "protocol-ce3-bad-controller-handoff";
        default:  return "protocol-error";
        }
    }
    /* HAL-level flags, checked last: they are usually a consequence of one of
     * the bus errors above rather than the root cause. */
    if ((err & HAL_I3C_ERROR_SIZE) != 0u) {
        return "short-transfer";
    }
    if ((err & HAL_I3C_ERROR_NOT_ALLOWED) != 0u) {
        return "not-allowed-in-this-state";
    }
    if ((err & HAL_I3C_ERROR_INVALID_PARAM) != 0u) {
        return "invalid-parameter";
    }
    return "unknown";
}

/** Reply for an I3C transfer, decoding the peripheral error on failure. */
/**
 * @brief Put the peripheral back into a usable state after a failed frame.
 *
 * A frame that failed part-way leaves bytes in the control, status, Tx and Rx
 * FIFOs and the state machine somewhere in the middle. The next command then
 * fails for a reason that belongs to this one - and the failure after that for
 * a reason belonging to neither - which makes a bus impossible to reason
 * about. Seen on this project as a controller that stopped emitting anything
 * at all: zero edges on the wire, while the host was still being told
 * "timeout" as though the bus were at fault.
 *
 * The I2C side already does this in seq_wait(). I3C had nothing.
 *
 * Flush first, and re-initialise only when the state machine did not come back
 * on its own: a full re-init also re-applies the pins, which is a bigger
 * hammer than a NACK deserves.
 */
static void i3c_recover(void)
{
    if (!configured) {
        return;
    }
    (void)HAL_I3C_FlushAllFifo(&hi3c);
    if (HAL_I3C_GetState(&hi3c) != HAL_I3C_STATE_READY) {
        /* Force the next command to rebuild it from scratch. */
        configured   = false;
        target_count = 0;
    }
}

static bool i3c_reply(HAL_StatusTypeDef st, const char *what)
{
    if (st == HAL_OK) {
        return true;
    }
    if (st == HAL_TIMEOUT) {
        bb_reply_errf(BB_ERR_TIMEOUT, "%s-timeout", what);
        i3c_recover();
        return false;
    }
    uint32_t err = HAL_I3C_GetError(&hi3c);
    int code = ((err & (I3C_SER_ANACK | I3C_SER_DNACK)) != 0u ||
                (err & 0x0Fu) == 2u) ? BB_ERR_NACK : BB_ERR_HAL;
    bb_reply_errf(code, "%s %s err=0x%lX", what, i3c_error_text(err), (unsigned long)err);
    i3c_recover();
    return false;
}

/**
 * @brief Fill in an I3C transfer descriptor.
 *
 * Two HAL requirements are easy to get wrong and both fail as a bare
 * "invalid parameter":
 *   - a directed CCC consumes two control words per frame, so CtrlBuf must be
 *     sized for 2 x nbFrame, not nbFrame;
 *   - TxBuf.pBuffer is rejected when NULL even for a zero-length broadcast CCC
 *     such as RSTDAA, so it always points at real storage here.
 */
static void xfer_init(I3C_XferTypeDef *xfer, uint8_t *buf, uint32_t tx_len, uint32_t rx_len)
{
    xfer->CtrlBuf.pBuffer   = ctrl_words;
    xfer->CtrlBuf.Size      = (uint32_t)(sizeof ctrl_words / sizeof ctrl_words[0]);
    xfer->StatusBuf.pBuffer = status_words;
    xfer->StatusBuf.Size    = (uint32_t)(sizeof status_words / sizeof status_words[0]);
    xfer->TxBuf.pBuffer     = buf;
    xfer->TxBuf.Size        = tx_len;
    xfer->RxBuf.pBuffer     = buf;
    xfer->RxBuf.Size        = rx_len;
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_freq(int argc, char **argv)
{
    uint32_t pp;
    uint32_t od = i2c_od_hz;

    if (!bb_parse_u32(argv[1], &pp) || pp < 100000u || pp > 12900000u) {
        bb_reply_err(BB_ERR_RANGE, "pp-hz-100k..12.9M");
        return;
    }
    if (argc > 2) {
        if (!bb_parse_u32(argv[2], &od) || od < 10000u || od > 1000000u) {
            bb_reply_err(BB_ERR_RANGE, "od-hz-10k..1M");
            return;
        }
    }
    i3c_pp_hz = pp;
    i2c_od_hz = od;
    if (!i3c_apply()) {
        bb_reply_errf(BB_ERR_HAL, "i3c-init-failed-at-%s", last_init_error);
        return;
    }
    bb_reply_okf("pp=%lu od=%lu", (unsigned long)i3c_pp_hz, (unsigned long)i2c_od_hz);
}

static void cmd_freq_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* The kernel clock is named, and named as marginal when it is: a clock that
     * leaves one half of the push-pull bit a single cycle long drives a bus
     * that works or does not depending on the pull-ups, and a caller deserves
     * to see that here rather than infer it from a target that stopped
     * acknowledging. */
    bb_reply_okf("pp=%lu od=%lu bus=%s kernel=%lu(%s%s) port=%u",
                 (unsigned long)i3c_pp_hz, (unsigned long)i2c_od_hz,
                 (bus_type == I3C_MIXED_BUS) ? "mixed" : "pure",
                 (unsigned long)i3c_kernel_hz,
                 i3c_kernel_name,
                 i3c_kernel_marginal ? ",marginal" : "",
                 (unsigned)i3c_port);
}

/**
 * @brief Choose which pin pair the I3C controller drives.
 *
 * Both supported parts have exactly one I3C peripheral, so this does not select
 * between controllers - it selects where the one bus appears. Boards with a
 * single pair accept only port 1.
 *
 * Changing port re-initialises the controller, which discards the target table:
 * the devices that were addressed are on the other pins now, and reporting
 * stale dynamic addresses for them would be worse than forgetting them.
 */
/**
 * @brief Hold the I3C pins at fixed levels, for checking a wire.
 *
 * Releases the I3C peripheral first: the pins cannot be an alternate function
 * and a GPIO at once, and leaving the controller thinking it still owns them
 * would make the next transfer fail in a way unrelated to the cause.
 */
static void cmd_drive(int argc, char **argv)
{
    uint32_t scl;
    uint32_t sda;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1, &scl) || !bb_parse_u32_max(argv[2], 1, &sda)) {
        bb_reply_err(BB_ERR_RANGE, "levels-0-or-1");
        return;
    }
    bb_i3c_release();
    target_count = 0;
    bb_pins_i3c_drive(i3c_port, scl != 0u, sda != 0u);
    bb_reply_okf("driving scl=%lu sda=%lu on %s", (unsigned long)scl,
                 (unsigned long)sda, bb_i3c_port_pins(i3c_port));
}

/**
 * @brief Stop taking part in the bus: deinit the controller, release the pins.
 *
 * Needed whenever this board is meant to watch rather than drive. Without it a
 * board that has ever run an I3C command keeps SCL and SDA as push-pull
 * outputs, so it silently fights the board that is actually driving - and the
 * symptom is a line that reads stuck at one level, which looks like a wiring
 * fault rather than like two boards arguing.
 */
static void cmd_free(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_i3c_release();
    target_count = 0;
    bb_pins_i3c_free(i3c_port);
    bb_reply_okf("released %s", bb_i3c_port_pins(i3c_port));
}

/**
 * @brief Keep or drop the internal pull-ups on the I3C pins.
 *
 * Re-applies the pins immediately when the controller is already up, so the
 * change is visible without a reconfigure - a setting that needs an unrelated
 * command before it takes effect is a setting people get wrong.
 */
static void cmd_pull(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1, &on)) {
        bb_reply_err(BB_ERR_RANGE, "expected-0-or-1");
        return;
    }
    bb_pins_i3c_set_pull(on != 0u);
    if (configured) {
        bb_pins_i3c(i3c_port);
    }
    bb_reply_okf("pull=%s on %s (internal, about 40k - not a substitute for "
                 "bus resistors)", (on != 0u) ? "on" : "off",
                 bb_i3c_port_pins(i3c_port));
}

/** The dynamic address the bus gave us, or 0 when it has not been assigned. */
static uint8_t tgt_dyn_addr(void)
{
    if (!configured || i3c_role != BB_I3C_TARGET || !tgt_addr_assigned) {
        return 0u;
    }
    if (LL_I3C_IsEnabledOwnDynAddress(hi3c.Instance) == 0u) {
        return 0u;
    }
    return LL_I3C_GetOwnDynamicAddress(hi3c.Instance);
}

/** I3C:ROLE <controller|target> - which side of the bus this board is. */
static void cmd_role(int argc, char **argv)
{
    i3c_role_t want;

    (void)argc;
    if (bb_streq_ci(argv[1], "controller") || bb_streq_ci(argv[1], "ctrl")) {
        want = BB_I3C_CONTROLLER;
    } else if (bb_streq_ci(argv[1], "target") || bb_streq_ci(argv[1], "tgt")) {
        want = BB_I3C_TARGET;
    } else {
        bb_reply_err(BB_ERR_ARGC, "expected-controller-or-target");
        return;
    }

    /* Always rebuild, even when the role is not changing.
     *
     * A caller asking for a role is asking for a peripheral in that role, not
     * for a no-op if it happens to be there already. Skipping the rebuild left
     * whatever the last run had done in place - a dynamic address, an armed
     * reply, a half-finished transfer - so a script that began by setting its
     * role got a target that was already addressed and would not take part in
     * discovery. Rebuilding costs a few register writes. */
    {
        bb_i3c_release();
        i3c_role     = want;
        target_count = 0;
        /* Back to the discoverable default. Hot-join is a deliberate mode and
         * should not survive a role switch, or a target set up fresh would
         * still be invisible to ENTDAA for reasons set several commands ago. */
        tgt_hotjoin  = false;
    }
    if (!i3c_ready_as(i3c_role)) {
        return;
    }
    if (i3c_role == BB_I3C_TARGET) {
        bb_reply_okf("role=target dcr=0x%02X instance=%u addr=%s %s",
                     (unsigned)tgt_dcr, (unsigned)tgt_instance,
                     (tgt_dyn_addr() != 0u) ? "assigned" : "none-yet",
                     bb_i3c_port_pins(i3c_port));
    } else {
        bb_reply_okf("role=controller %s", bb_i3c_port_pins(i3c_port));
    }
}

static void cmd_role_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("role=%s up=%u",
                 (i3c_role == BB_I3C_TARGET) ? "target" : "controller",
                 configured ? 1u : 0u);
}

/** I3C:TGT? - everything a target knows about its own situation. */
static void cmd_tgt_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (i3c_role != BB_I3C_TARGET) {
        bb_reply_err(BB_ERR_BUS_STATE, "not-in-target-role");
        return;
    }

    uint8_t  addr    = tgt_dyn_addr();
    uint16_t pending = (uint16_t)((tgt_rx_head + I3C_TGT_RX_RING - tgt_rx_tail)
                                 % I3C_TGT_RX_RING);

    char addr_text[8];

    /* "none" rather than 0x00: zero is a legal-looking address and this field
     * is read by eye as often as by a script. */
    if (addr != 0u) {
        snprintf(addr_text, sizeof addr_text, "0x%02X", (unsigned)addr);
    } else {
        snprintf(addr_text, sizeof addr_text, "none");
    }

    /* EPIDR and BCR only. DEVR0, SER, the HAL state and its error code all used
     * to be here as well, and the reply outgrew the 256-byte payload: it came
     * back truncated, losing EPIDR's last digit and BCR entirely, which is a
     * poor way for a diagnostic to behave. They are all in I3C:REG? already, so
     * the duplicates went rather than the fields that are only here. */
    bb_reply_okf("addr=%s dcr=0x%02X instance=%u hotjoin=%u rx_pending=%u "
                 "rx_lost=%lu "
                 "writes=%lu reads=%lu errors=%lu underruns=%lu "
                 "last_ser=0x%08lX last_inframe=%lu forced_idle=%lu "
                 "stuck=%lu rx_live=%u tx_armed=%u tx_len=%u "
                 "epidr=0x%08lX bcr=0x%08lX",
                 addr_text, (unsigned)tgt_dcr,
                 (unsigned)tgt_instance, tgt_hotjoin ? 1u : 0u, (unsigned)pending,
                 (unsigned long)tgt_rx_lost, (unsigned long)tgt_writes,
                 (unsigned long)tgt_reads, (unsigned long)tgt_errors,
                 (unsigned long)tgt_underruns,
                 (unsigned long)tgt_last_ser,
                 (unsigned long)tgt_last_inframe,
                 (unsigned long)tgt_forced_idle,
                 (unsigned long)tgt_stuck_events,
                 tgt_rx_live ? 1u : 0u, tgt_tx_armed ? 1u : 0u,
                 (unsigned)tgt_tx_len,
                 (unsigned long)I3C1->EPIDR, (unsigned long)I3C1->BCR);
}

/** I3C:TGT:ADDR? - just the dynamic address, for scripts. 0 means unassigned. */
static void cmd_tgt_addr(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (i3c_role != BB_I3C_TARGET) {
        bb_reply_err(BB_ERR_BUS_STATE, "not-in-target-role");
        return;
    }
    bb_reply_okf("0x%02X", (unsigned)tgt_dyn_addr());
}

/** I3C:TGT:ID <dcr> [instance] - what this target claims to be. */
static void cmd_tgt_id(int argc, char **argv)
{
    uint32_t dcr;
    uint32_t inst = tgt_instance;

    if (!bb_parse_u32_max(argv[1], 0xFFu, &dcr)) {
        bb_reply_err(BB_ERR_RANGE, "dcr-0..255");
        return;
    }
    if (argc > 2 && !bb_parse_u32_max(argv[2], 0x0Fu, &inst)) {
        bb_reply_err(BB_ERR_RANGE, "instance-0..15");
        return;
    }
    tgt_dcr      = (uint8_t)dcr;
    tgt_instance = (uint8_t)inst;

    /* The identity is part of the ENTDAA payload, so it has to be in the
     * registers before the controller asks - which means reconfiguring now
     * rather than at the next transfer. */
    if (i3c_role == BB_I3C_TARGET && configured) {
        bb_i3c_release();
        if (!i3c_apply()) {
            bb_reply_errf(BB_ERR_HAL, "i3c-init-failed-at-%s", last_init_error);
            return;
        }
    }
    bb_reply_okf("dcr=0x%02X instance=%u", (unsigned)tgt_dcr,
                 (unsigned)tgt_instance);
}

/**
 * @brief Load a reply for the controller's next private read.
 *
 * Split out of the I3C:TGT:TX command so that the peripheral profile can arm
 * replies of its own without a host in the loop - an EEPROM has to answer a read
 * that nobody asked it to prepare for. Everything hard about this function was
 * learned from the bugs described inside it; see bb_i3c.h.
 *
 * @return false when the peripheral refused the transfer, in which case
 *         HAL_I3C_GetError() says why.
 */
bool bb_i3c_tgt_arm(const uint8_t *data, size_t len)
{
    if (len == 0u || len > sizeof tgt_tx_buf) {
        return false;
    }
    if (i3c_role != BB_I3C_TARGET || !configured) {
        return false;
    }
    memcpy(tgt_tx_buf, data, len);

    /* Preload the count, then let the interrupt feed the bytes. The count has
     * to be in TGTTDR before the controller starts clocking, which is why it
     * is armed by a command rather than produced on demand. */
    /* Throw away anything left from a previous reply. A read that consumed
     * fewer bytes than were armed leaves the rest in the FIFO, and they come
     * out in front of the next reply - which shows up as reads that succeed
     * and return the previous answer, shifted. */
    LL_I3C_DisableIT_TXFNF(hi3c.Instance);
    LL_I3C_RequestTxFIFOFlush(hi3c.Instance);

    /* Also finish anything the last frame left behind. Arming a reply while a
     * previous transfer is still nominally in flight, or while a flag from the
     * frame before is still set, means the DMA is started into a peripheral
     * that is not ready - and the symptom is not a failed arm but a later read
     * that returns the wrong length. Interleaving writes and reads was enough
     * to trip it; doing all the writes and then all the reads was not, which
     * is why it hid for so long. */
    if (HAL_I3C_GetState(&hi3c) != HAL_I3C_STATE_LISTEN &&
        HAL_I3C_GetState(&hi3c) != HAL_I3C_STATE_READY) {
        (void)HAL_I3C_Abort_IT(&hi3c);
    }
    /* And wait for it, which is the part that was missing.
     *
     * HAL_I3C_Abort_IT() only starts an abort - it finishes in an interrupt,
     * some microseconds later. Transmitting straight afterwards was therefore
     * asking a peripheral in HAL_I3C_STATE_ABORT to start a transfer, which it
     * refuses: the arm failed, and the target was left mid-abort, no longer
     * acknowledging its own address. One write that overran the receive FIFO
     * was enough, and nothing recovered afterwards - every later frame came
     * back address-nack. An autonomous target has nobody to re-arm it, so it
     * simply went deaf.
     *
     * The wait is bounded because a bus being hammered can delay the abort,
     * and hanging here would be worse than reporting it. */
    for (uint32_t spent = 0u; spent < I3C_TGT_ABORT_WAIT_MS; spent++) {
        HAL_I3C_StateTypeDef state = HAL_I3C_GetState(&hi3c);
        if (state == HAL_I3C_STATE_LISTEN || state == HAL_I3C_STATE_READY) {
            break;
        }
        HAL_Delay(1u);
    }
    if (HAL_I3C_GetState(&hi3c) != HAL_I3C_STATE_LISTEN &&
        HAL_I3C_GetState(&hi3c) != HAL_I3C_STATE_READY) {
        /* The abort did not finish, so finish it here.
         *
         * It needs the bus to settle, and the bus is not always going to: a
         * controller measuring a write ceiling keeps clocking frames at a
         * target that cannot answer, and the abort waits behind them. Refusing
         * to arm was worse than the problem - the target then had nothing to
         * say for the rest of that rate, so every read came back
         * address-nack and one overrun took a whole row of a sweep with it.
         *
         * Nothing here is the HAL's to lose: this module drives the target's
         * receive path from the event interrupt and its transmit path through
         * DMA, so the HAL's own view of the transfer is bookkeeping. Stop the
         * channel, empty the FIFOs, and say what is actually true - the
         * peripheral is listening and nothing is in flight. */
        if (hi3c.hdmatx != NULL) {
            (void)HAL_DMA_Abort(hi3c.hdmatx);
        }
        LL_I3C_RequestTxFIFOFlush(hi3c.Instance);
        LL_I3C_RequestRxFIFOFlush(hi3c.Instance);
        LL_I3C_ClearFlag_FC(hi3c.Instance);
        LL_I3C_ClearFlag_ERR(hi3c.Instance);
        hi3c.State     = HAL_I3C_STATE_LISTEN;
        hi3c.ErrorCode = HAL_I3C_ERROR_NONE;
        tgt_forced_idle++;
    }
    LL_I3C_ClearFlag_FC(hi3c.Instance);
    LL_I3C_ClearFlag_ERR(hi3c.Instance);
    hi3c.ErrorCode = HAL_I3C_ERROR_NONE;

    tgt_tx_len = (uint8_t)len;
    tgt_tx_pos = 0u;

    /* DMA feeds the FIFO from here on. Software cannot: a private read runs at
     * the push-pull rate, so at 12.5 MHz a byte leaves every 640 ns while an
     * interrupt costs microseconds. Hand-feeding delivered whatever happened
     * to be preloaded and nothing after it. */
    tgt_tx_xfer.TxBuf.pBuffer = tgt_tx_buf;
    tgt_tx_xfer.TxBuf.Size    = (uint32_t)len;
    if (HAL_I3C_Tgt_Transmit_DMA(&hi3c, &tgt_tx_xfer) != HAL_OK) {
        return false;
    }
    tgt_tx_armed = true;
    return true;
}

/** Bytes the controller actually clocked out of the last armed reply.
 *
 * The GPDMA channel counts down as it feeds the FIFO, so what is left in CBR1
 * is what the controller did not take. A pointer that auto-increments has to
 * advance by the difference and not by the length that was armed - a controller
 * reading four bytes of a thirty-two byte arm would otherwise leave the pointer
 * twenty-eight bytes past where it should be. */
uint16_t bb_i3c_tgt_tx_consumed(void)
{
    uint32_t left = 0;

    if (hi3c.hdmatx != NULL) {
        left = hi3c.hdmatx->Instance->CBR1 & DMA_CBR1_BNDT;
    }
    if (left > (uint32_t)tgt_tx_len) {
        return 0;
    }
    return (uint16_t)((uint32_t)tgt_tx_len - left);
}

/** True while a reply is loaded and unread. */
bool bb_i3c_tgt_armed(void)
{
    return tgt_tx_armed;
}

/** Completed private writes, for a caller that needs to know a frame ended. */
uint32_t bb_i3c_tgt_write_frames(void)
{
    return tgt_writes;
}

/** Drain received bytes without a host command. Returns how many were taken. */
size_t bb_i3c_tgt_take(uint8_t *out, size_t max)
{
    size_t n = 0;

    while (n < max && tgt_rx_tail != tgt_rx_head) {
        out[n++] = tgt_rx_ring[tgt_rx_tail];
        tgt_rx_tail = (uint16_t)((tgt_rx_tail + 1u) % I3C_TGT_RX_RING);
    }
    return n;
}

/** The dynamic address the bus gave us, or 0. */
uint8_t bb_i3c_tgt_address(void)
{
    return (i3c_role == BB_I3C_TARGET) ? tgt_dyn_addr() : 0u;
}

/**
 * @brief Put the peripheral into the target role, without a host asking.
 *
 * Same rebuild the I3C:ROLE command performs, and for the same reason: a caller
 * asking for the target role wants a peripheral freshly in it, not whatever the
 * last run left behind.
 */
bool bb_i3c_tgt_begin(void)
{
    bb_i3c_release();
    i3c_role     = BB_I3C_TARGET;
    target_count = 0;
    tgt_hotjoin  = false;
    return i3c_apply();
}

/** I3C:TGT:TX <hex> - arm bytes for the controller's next private read. */
static void cmd_tgt_tx(int argc, char **argv)
{
    static uint8_t staged[I3C_TGT_TX_MAX];
    size_t len;

    if (!i3c_ready_as(BB_I3C_TARGET)) {
        return;
    }
    /* Parsed into a local, not straight into tgt_tx_buf: the arming function
     * copies into that buffer, and handing it its own destination as a source
     * would be a self-overlapping memcpy. */
    if (!bb_parse_bytes(argc, argv, 1, staged, sizeof staged, &len)
        || len == 0u) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }

    if (LL_I3C_IsEnabledOwnDynAddress(hi3c.Instance) != 1u) {
        bb_reply_err(BB_ERR_BUS_STATE, "no-dynamic-address-run-ENTDAA-first");
        return;
    }

    if (!bb_i3c_tgt_arm(staged, len)) {
        bb_reply_errf(BB_ERR_HAL, "arm-failed err=0x%lX",
                      (unsigned long)HAL_I3C_GetError(&hi3c));
        return;
    }
    bb_reply_okf("armed=%u dma=1", (unsigned)len);
}

/** I3C:TGT:RX? [max] - drain what the controller has written to us. */
static void cmd_tgt_rx(int argc, char **argv)
{
    uint8_t  out[BB_PAYLOAD_MAX];
    uint32_t want = sizeof out;
    size_t   n    = 0;

    if (i3c_role != BB_I3C_TARGET) {
        bb_reply_err(BB_ERR_BUS_STATE, "not-in-target-role");
        return;
    }
    if (argc > 1 && !bb_parse_u32_max(argv[1], sizeof out, &want)) {
        bb_reply_errf(BB_ERR_RANGE, "max-1..%u", (unsigned)sizeof out);
        return;
    }
    while (n < want && tgt_rx_tail != tgt_rx_head) {
        out[n++] = tgt_rx_ring[tgt_rx_tail];
        tgt_rx_tail = (uint16_t)((tgt_rx_tail + 1u) % I3C_TGT_RX_RING);
    }
    if (n == 0u) {
        bb_reply_okf("empty");
        return;
    }
    bb_reply_ok_bytes(out, n);
}

/** I3C:TGT:IBI [hex] - ask the controller for attention. */
static void cmd_tgt_ibi(int argc, char **argv)
{
    uint8_t payload[I3C_IBI_PAYLOAD_MAX];
    size_t  len = 0;

    if (!i3c_ready_as(BB_I3C_TARGET)) {
        return;
    }
    if (tgt_dyn_addr() == 0u) {
        /* An IBI is addressed by the target's dynamic address. Without one
         * there is nothing to put on the bus. */
        bb_reply_err(BB_ERR_BUS_STATE, "no-dynamic-address-run-ENTDAA-first");
        return;
    }
    if (argc > 1 && !bb_parse_bytes(argc, argv, 1, payload, sizeof payload,
                                    &len)) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }

    HAL_StatusTypeDef st = HAL_I3C_Tgt_IBIReq(&hi3c,
                                              (len > 0u) ? payload : NULL,
                                              (uint8_t)len, I3C_TIMEOUT_MS);
    if (st != HAL_OK) {
        bb_reply_errf(BB_ERR_HAL, "ibi-refused err=0x%lX",
                      (unsigned long)HAL_I3C_GetError(&hi3c));
        return;
    }
    bb_reply_okf("ibi-sent addr=0x%02X payload=%u",
                 (unsigned)tgt_dyn_addr(), (unsigned)len);
}

/**
 * @brief I3C:TGT:HJ - ask to be given an address without a full ENTDAA.
 *
 * Arms hot-join first and reconfigures, because HJEN has to be set before the
 * request and cannot be set while a frame is in flight. Doing that takes the
 * target out of ENTDAA's view until the role is switched again, so the reply
 * says so rather than leaving it to be discovered by a discovery that finds
 * nothing.
 */
static void cmd_tgt_hj(int argc, char **argv)
{
    uint8_t addr = 0;

    (void)argc;
    (void)argv;
    if (!i3c_ready_as(BB_I3C_TARGET)) {
        return;
    }
    if (!tgt_hotjoin) {
        tgt_hotjoin = true;
        bb_i3c_release();
        if (!i3c_apply()) {
            tgt_hotjoin = false;
            bb_reply_errf(BB_ERR_HAL, "i3c-init-failed-at-%s", last_init_error);
            return;
        }
    }
    if (HAL_I3C_Tgt_HotJoinReq(&hi3c, &addr, I3C_TIMEOUT_MS) != HAL_OK) {
        bb_reply_errf(BB_ERR_HAL, "hotjoin-refused err=0x%lX",
                      (unsigned long)HAL_I3C_GetError(&hi3c));
        return;
    }
    bb_reply_okf("joined addr=0x%02X (hot-join now armed, so ENTDAA will not "
                 "see this target until I3C:ROLE is set again)", (unsigned)addr);
}

/**
 * @brief Raw peripheral registers, for when the reply is not enough.
 *
 * Added while chasing a CCC that reached the wire complete - START, address,
 * command byte, STOP, all sniffed - and still timed out. The polling loop
 * waits on the frame-complete flag, so the only way to tell "the frame did not
 * finish" from "the frame finished and the flag went missing" is to look.
 */
static void cmd_regs(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!configured) {
        bb_reply_errf(BB_ERR_BUS_STATE, "not-configured-role=%s",
                      (i3c_role == BB_I3C_TARGET) ? "target" : "controller");
        return;
    }
    bb_reply_okf("role=%s state=%u halerr=0x%lX evr=0x%08lX ser=0x%08lX "
                 "cfgr=0x%08lX sr=0x%08lX devr0=0x%08lX ier=0x%08lX "
                 "timingr0=0x%08lX timingr1=0x%08lX",
                 (i3c_role == BB_I3C_TARGET) ? "target" : "controller",
                 (unsigned)HAL_I3C_GetState(&hi3c),
                 (unsigned long)HAL_I3C_GetError(&hi3c),
                 (unsigned long)I3C1->EVR, (unsigned long)I3C1->SER,
                 (unsigned long)I3C1->CFGR, (unsigned long)I3C1->SR,
                 (unsigned long)I3C1->DEVR0, (unsigned long)I3C1->IER,
                 (unsigned long)I3C1->TIMINGR0, (unsigned long)I3C1->TIMINGR1);
}

static void cmd_port(int argc, char **argv)
{
    uint32_t n;

    (void)argc;
    if (!bb_parse_u32(argv[1], &n) || n < 1u || n > BB_I3C_PORT_COUNT) {
        bb_reply_errf(BB_ERR_RANGE, "port-1..%u", (unsigned)BB_I3C_PORT_COUNT);
        return;
    }
    if ((uint8_t)n != i3c_port) {
        i3c_port = (uint8_t)n;
        /* Force a re-apply on the next use, on the new pins. */
        bb_i3c_release();
        target_count = 0;
    }
    bb_reply_okf("port=%u pins=%s of=%u", (unsigned)i3c_port,
                 bb_i3c_port_pins(i3c_port), (unsigned)BB_I3C_PORT_COUNT);
}

static void cmd_port_q(int argc, char **argv)
{
    char out[220];
    size_t used = 0;

    (void)argc;
    (void)argv;
    for (unsigned i = 1u; i <= BB_I3C_PORT_COUNT; i++) {
        used += (size_t)snprintf(out + used, sizeof out - used, "%s%u:%s",
                                 (i > 1u) ? ";" : "", i, bb_i3c_port_pins(i));
    }
    bb_reply_okf("port=%u count=%u %s", (unsigned)i3c_port,
                 (unsigned)BB_I3C_PORT_COUNT, out);
}

static void cmd_bus(int argc, char **argv)
{
    (void)argc;
    if (bb_streq_ci(argv[1], "PURE")) {
        bus_type = I3C_PURE_I3C_BUS;
    } else if (bb_streq_ci(argv[1], "MIXED")) {
        bus_type = I3C_MIXED_BUS;
    } else {
        bb_reply_err(BB_ERR_RANGE, "pure-or-mixed");
        return;
    }
    if (!i3c_apply()) {
        bb_reply_errf(BB_ERR_HAL, "i3c-init-failed-at-%s", last_init_error);
        return;
    }
    bb_reply_ok();
}

/**
 * @brief Run ENTDAA and hand out dynamic addresses.
 *
 * The HAL signals "a target answered, its payload is ready" with HAL_BUSY and
 * "the frame is finished" with HAL_OK, so the loop assigns an address on every
 * HAL_BUSY and stops on HAL_OK.
 */
static void cmd_entdaa(int argc, char **argv)
{
    uint64_t payload = 0;
    uint8_t  next_addr = I3C_FIRST_DYN_ADDR;

    (void)argc;
    (void)argv;
    if (!i3c_ready()) {
        return;
    }

    target_count = 0;
    HAL_StatusTypeDef st = HAL_I3C_Ctrl_DynAddrAssign(&hi3c, &payload,
                                                      I3C_RSTDAA_THEN_ENTDAA,
                                                      I3C_TIMEOUT_MS);
    while (st == HAL_BUSY) {
        if (target_count >= I3C_MAX_TARGETS) {
            bb_reply_errf(BB_ERR_RANGE, "too-many-targets-max-%u", (unsigned)I3C_MAX_TARGETS);
            return;
        }

        I3C_ENTDAAPayloadTypeDef decoded = {0};
        (void)HAL_I3C_Get_ENTDAA_Payload_Info(&hi3c, payload, &decoded);

        target_t *t   = &targets[target_count];
        t->dyn_addr   = next_addr;
        t->payload    = payload;
        t->dcr        = (uint8_t)decoded.DCR;
        t->ibi_capable = (decoded.BCR.IBIRequestCapable == ENABLE);
        /* Rebuild the BCR byte from the decoded flags for reporting. */
        t->bcr = (uint8_t)((decoded.BCR.MaxDataSpeedLimitation == ENABLE ? 0x01u : 0u) |
                           (decoded.BCR.IBIRequestCapable     == ENABLE ? 0x02u : 0u) |
                           (decoded.BCR.IBIPayload            == ENABLE ? 0x04u : 0u) |
                           (decoded.BCR.OfflineCapable        == ENABLE ? 0x08u : 0u) |
                           (decoded.BCR.VirtualTargetSupport  == ENABLE ? 0x10u : 0u) |
                           (decoded.BCR.AdvancedCapabilities  == ENABLE ? 0x20u : 0u) |
                           (decoded.BCR.DeviceRole            == ENABLE ? 0x40u : 0u));
        target_count++;

        if (HAL_I3C_Ctrl_SetDynAddr(&hi3c, next_addr) != HAL_OK) {
            bb_reply_err(BB_ERR_HAL, "setdynaddr-failed");
            return;
        }
        next_addr++;
        st = HAL_I3C_Ctrl_DynAddrAssign(&hi3c, &payload, I3C_ONLY_ENTDAA, I3C_TIMEOUT_MS);
    }

    if (st != HAL_OK) {
        /* An empty bus fails here too: nothing acknowledges the 0x7E broadcast,
         * so report the HAL error code rather than a bare "error". */
        bb_reply_errf(BB_ERR_NACK, "entdaa %s err=0x%lX targets=%u",
                      i3c_error_text(HAL_I3C_GetError(&hi3c)),
                      (unsigned long)HAL_I3C_GetError(&hi3c), target_count);
        return;
    }
    if (!program_device_slots()) {
        bb_reply_err(BB_ERR_HAL, "configbusdevices-failed");
        return;
    }
    bb_reply_okf("targets=%u", target_count);
}

static void cmd_rstdaa(int argc, char **argv)
{
    I3C_XferTypeDef xfer       = {0};
    I3C_CCCTypeDef  ccc        = {0};
    static uint8_t  scratch[1] = {0};

    (void)argc;
    (void)argv;
    if (!i3c_ready()) {
        return;
    }
    ccc.TargetAddr     = 0;
    ccc.CCC            = CCC_RSTDAA;
    ccc.CCCBuf.pBuffer = NULL;
    ccc.CCCBuf.Size    = 0;
    ccc.Direction      = HAL_I3C_DIRECTION_WRITE;

    xfer_init(&xfer, scratch, 0, 0);

    if (HAL_I3C_AddDescToFrame(&hi3c, &ccc, NULL, &xfer, 1,
                               I3C_BROADCAST_WITHOUT_DEFBYTE_STOP) != HAL_OK) {
        bb_reply_err(BB_ERR_HAL, "adddesc-failed");
        return;
    }
    HAL_StatusTypeDef st = HAL_I3C_Ctrl_TransmitCCC(&hi3c, &xfer, I3C_TIMEOUT_MS);
    target_count = 0;
    if (i3c_reply(st, "rstdaa")) {
        bb_reply_ok();
    }
}

static void cmd_targets(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (target_count == 0u) {
        bb_reply_okf("targets=0");
        return;
    }
    for (uint8_t i = 0; i < target_count; i++) {
        const target_t *t = &targets[i];
        bb_io_printf("# addr=0x%02X bcr=0x%02X dcr=0x%02X ibi=%d slot=%u "
                          "payload=%08lX%08lX\r\n",
                          t->dyn_addr, t->bcr, t->dcr, t->ibi_capable ? 1 : 0,
                          t->hw_slot,
                          (unsigned long)(t->payload >> 32),
                          (unsigned long)(t->payload & 0xFFFFFFFFu));
    }
    bb_reply_okf("targets=%u", target_count);
}

/** Broadcast CCC: I3C:CCC:B <ccc> [hex-payload...] */
static void cmd_ccc_broadcast(int argc, char **argv)
{
    uint8_t         data[BB_PAYLOAD_MAX];
    size_t          len = 0;
    uint32_t        code;
    I3C_XferTypeDef xfer = {0};
    I3C_CCCTypeDef  ccc  = {0};

    if (!i3c_ready()) {
        return;
    }
    if (!bb_parse_u32_max(argv[1], 0xFFu, &code)) {
        bb_reply_err(BB_ERR_RANGE, "ccc-0..255");
        return;
    }
    if (argc > 2 && !bb_parse_bytes(argc, argv, 2, data, sizeof data, &len)) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }

    ccc.TargetAddr     = 0;
    ccc.CCC            = (uint8_t)code;
    ccc.CCCBuf.pBuffer = (len > 0) ? data : NULL;
    ccc.CCCBuf.Size    = (uint32_t)len;
    ccc.Direction      = HAL_I3C_DIRECTION_WRITE;

    xfer_init(&xfer, data, (uint32_t)len, 0);

    uint32_t option = (len > 0) ? I3C_BROADCAST_WITH_DEFBYTE_STOP
                                : I3C_BROADCAST_WITHOUT_DEFBYTE_STOP;
    if (HAL_I3C_AddDescToFrame(&hi3c, &ccc, NULL, &xfer, 1, option) != HAL_OK) {
        bb_reply_err(BB_ERR_HAL, "adddesc-failed");
        return;
    }
    if (i3c_reply(HAL_I3C_Ctrl_TransmitCCC(&hi3c, &xfer, I3C_TIMEOUT_MS), "ccc-broadcast")) {
        bb_reply_ok();
    }
}

/** Directed CCC write: I3C:CCC:W <addr> <ccc> [hex-payload...] */
static void cmd_ccc_write(int argc, char **argv)
{
    uint8_t         data[BB_PAYLOAD_MAX];
    size_t          len = 0;
    uint32_t        addr;
    uint32_t        code;
    I3C_XferTypeDef xfer = {0};
    I3C_CCCTypeDef  ccc  = {0};

    if (!i3c_ready()) {
        return;
    }
    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr) || !bb_parse_u32_max(argv[2], 0xFFu, &code)) {
        bb_reply_err(BB_ERR_RANGE, "addr-7bit-ccc-0..255");
        return;
    }
    if (argc > 3 && !bb_parse_bytes(argc, argv, 3, data, sizeof data, &len)) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }

    ccc.TargetAddr     = (uint8_t)addr;
    ccc.CCC            = (uint8_t)code;
    ccc.CCCBuf.pBuffer = (len > 0) ? data : NULL;
    ccc.CCCBuf.Size    = (uint32_t)len;
    ccc.Direction      = HAL_I3C_DIRECTION_WRITE;

    xfer_init(&xfer, data, (uint32_t)len, 0);

    if (HAL_I3C_AddDescToFrame(&hi3c, &ccc, NULL, &xfer, 1,
                               I3C_DIRECT_WITHOUT_DEFBYTE_STOP) != HAL_OK) {
        bb_reply_err(BB_ERR_HAL, "adddesc-failed");
        return;
    }
    if (i3c_reply(HAL_I3C_Ctrl_TransmitCCC(&hi3c, &xfer, I3C_TIMEOUT_MS), "ccc-write")) {
        bb_reply_ok();
    }
}

/** Directed CCC read: I3C:CCC:R <addr> <ccc> <count> */
static void cmd_ccc_read(int argc, char **argv)
{
    uint8_t         data[BB_PAYLOAD_MAX];
    uint32_t        addr;
    uint32_t        code;
    uint32_t        count;
    I3C_XferTypeDef xfer = {0};
    I3C_CCCTypeDef  ccc  = {0};

    (void)argc;
    if (!i3c_ready()) {
        return;
    }
    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr) ||
        !bb_parse_u32_max(argv[2], 0xFFu, &code) ||
        !bb_parse_u32_max(argv[3], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_err(BB_ERR_RANGE, "addr-ccc-count");
        return;
    }

    ccc.TargetAddr     = (uint8_t)addr;
    ccc.CCC            = (uint8_t)code;
    ccc.CCCBuf.pBuffer = data;
    ccc.CCCBuf.Size    = count;
    ccc.Direction      = HAL_I3C_DIRECTION_READ;

    xfer_init(&xfer, data, 0, count);

    if (HAL_I3C_AddDescToFrame(&hi3c, &ccc, NULL, &xfer, 1,
                               I3C_DIRECT_WITHOUT_DEFBYTE_STOP) != HAL_OK) {
        bb_reply_err(BB_ERR_HAL, "adddesc-failed");
        return;
    }
    if (i3c_reply(HAL_I3C_Ctrl_ReceiveCCC(&hi3c, &xfer, I3C_TIMEOUT_MS), "ccc-read")) {
        bb_reply_ok_bytes(data, count);
    }
}

/**
 * @brief Private write. I3C:W <addr> <hex-bytes...>
 *
 * Works for both I3C targets (SDR private write) and legacy I2C devices on a
 * mixed bus - the peripheral picks the right signalling from the address and
 * the arbitration option.
 */
static void cmd_private_write(int argc, char **argv)
{
    uint8_t            data[BB_PAYLOAD_MAX];
    size_t             len;
    uint32_t           addr;
    I3C_XferTypeDef    xfer = {0};
    I3C_PrivateTypeDef priv = {0};

    if (!i3c_ready()) {
        return;
    }
    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr)) {
        bb_reply_err(BB_ERR_RANGE, "addr-7bit");
        return;
    }
    if (!bb_parse_bytes(argc, argv, 2, data, sizeof data, &len) || len == 0) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }

    priv.TargetAddr    = (uint8_t)addr;
    priv.TxBuf.pBuffer = data;
    priv.TxBuf.Size    = (uint32_t)len;
    priv.Direction     = HAL_I3C_DIRECTION_WRITE;

    xfer_init(&xfer, data, (uint32_t)len, 0);

    if (HAL_I3C_AddDescToFrame(&hi3c, NULL, &priv, &xfer, 1,
                               I3C_PRIVATE_WITH_ARB_STOP) != HAL_OK) {
        bb_reply_err(BB_ERR_HAL, "adddesc-failed");
        return;
    }
    if (i3c_reply(HAL_I3C_Ctrl_Transmit(&hi3c, &xfer, I3C_TIMEOUT_MS), "i3c-write")) {
        bb_reply_okf("wrote=%u", (unsigned)len);
    }
}

/** Private read. I3C:R <addr> <count> */
static void cmd_private_read(int argc, char **argv)
{
    uint8_t            data[BB_PAYLOAD_MAX];
    uint32_t           addr;
    uint32_t           count;
    I3C_XferTypeDef    xfer = {0};
    I3C_PrivateTypeDef priv = {0};

    (void)argc;
    if (!i3c_ready()) {
        return;
    }
    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr) ||
        !bb_parse_u32_max(argv[2], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_err(BB_ERR_RANGE, "addr-7bit-count-1..256");
        return;
    }

    priv.TargetAddr    = (uint8_t)addr;
    priv.RxBuf.pBuffer = data;
    priv.RxBuf.Size    = count;
    priv.Direction     = HAL_I3C_DIRECTION_READ;

    xfer_init(&xfer, data, 0, count);

    if (HAL_I3C_AddDescToFrame(&hi3c, NULL, &priv, &xfer, 1,
                               I3C_PRIVATE_WITH_ARB_STOP) != HAL_OK) {
        bb_reply_err(BB_ERR_HAL, "adddesc-failed");
        return;
    }
    if (i3c_reply(HAL_I3C_Ctrl_Receive(&hi3c, &xfer, I3C_TIMEOUT_MS), "i3c-read")) {
        bb_reply_ok_bytes(data, count);
    }
}

/** Probe: is anything at this address, as an I3C or as a legacy I2C device? */
static void cmd_probe(int argc, char **argv)
{
    uint32_t addr;
    bool     as_i2c = false;

    if (!i3c_ready()) {
        return;
    }
    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr)) {
        bb_reply_err(BB_ERR_RANGE, "addr-7bit");
        return;
    }
    if (argc > 2 && bb_streq_ci(argv[2], "I2C")) {
        as_i2c = true;
    }
    HAL_StatusTypeDef st = as_i2c
        ? HAL_I3C_Ctrl_IsDeviceI2C_Ready(&hi3c, (uint8_t)addr, 2, I3C_TIMEOUT_MS)
        : HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c, (uint8_t)addr, 2, I3C_TIMEOUT_MS);
    bb_reply_okf("%d", (st == HAL_OK) ? 1 : 0);
}

/** Arm IBI/hot-join reception so events land in the queue. */
static void cmd_listen(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    if (!i3c_ready()) {
        return;
    }
    if (!bb_parse_u32_max(argv[1], 1, &on)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }

    /* No ERRIE here either: its handler nulls the HAL's transfer function
     * pointers and every controller transfer in this firmware is a polling one
     * that calls them. See the note in i3c_apply(). */
    uint32_t mask = HAL_I3C_IT_IBIIE | HAL_I3C_IT_HJIE;
    if (on != 0u) {
        notify_xfer.RxBuf.pBuffer = ibi_rx_buf;
        notify_xfer.RxBuf.Size    = sizeof ibi_rx_buf;
        if (HAL_I3C_ActivateNotification(&hi3c, &notify_xfer, mask) != HAL_OK) {
            bb_reply_err(BB_ERR_HAL, "activate-notification-failed");
            return;
        }
        notifications_on = true;
    } else {
        (void)HAL_I3C_DeactivateNotification(&hi3c, mask);
        notifications_on = false;
    }
    bb_reply_ok();
}

/** Drain one queued IBI, or report that the queue is empty. */
static void cmd_ibi(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (ibi_head == ibi_tail) {
        bb_reply_okf("none dropped=%lu listen=%d",
                     (unsigned long)ibi_dropped, notifications_on ? 1 : 0);
        return;
    }
    ibi_event_t event;
    event = ibi_queue[ibi_tail];
    ibi_tail = (uint8_t)((ibi_tail + 1u) % I3C_IBI_QUEUE);

    char hex[3u * I3C_IBI_PAYLOAD_MAX + 1u];
    size_t pos = 0;
    static const char digits[] = "0123456789ABCDEF";
    for (uint8_t i = 0; i < event.len; i++) {
        hex[pos++] = (i == 0u) ? ' ' : ' ';
        hex[pos++] = digits[event.data[i] >> 4];
        hex[pos++] = digits[event.data[i] & 0x0Fu];
    }
    hex[pos] = '\0';
    bb_reply_okf("addr=0x%02X len=%u payload%s", event.addr, event.len,
                 (event.len > 0u) ? hex : "=");
}

void bb_i3c_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    const char *pins = bb_i3c_port_pins(i3c_port);

    snprintf(out->pins, sizeof out->pins, "p%u %s", (unsigned)i3c_port,
             (pins != NULL) ? pins : "-");
    if (i3c_role == BB_I3C_TARGET) {
        uint8_t addr = tgt_dyn_addr();
        snprintf(out->detail, sizeof out->detail, "target %s",
                 (addr != 0u) ? "joined" : "no addr");
        out->state = (addr != 0u) ? BB_STATE_ACTIVE : BB_STATE_IDLE;
    } else {
        snprintf(out->detail, sizeof out->detail, "%luM %u tgt",
                 (unsigned long)(i3c_pp_hz / 1000000u),
                 (unsigned)target_count);
        out->state = (target_count > 0u) ? BB_STATE_ACTIVE : BB_STATE_IDLE;
    }
}

const bb_cmd_t bb_cmd_i3c[] = {
    { "I3C:FREQ",   1, 2,    cmd_freq,          "I3C:FREQ <pp-hz> [od-hz]" },
    { "I3C:FREQ?",  0, 0,    cmd_freq_q,        "I3C:FREQ?" },
    { "I3C:BUS",    1, 1,    cmd_bus,           "I3C:BUS <pure|mixed>" },
    { "I3C:PORT",   1, 1,    cmd_port,          "I3C:PORT <n>" },
    { "I3C:DRIVE",  2, 2,    cmd_drive,         "I3C:DRIVE <scl 0|1> <sda 0|1>" },
    { "I3C:FREE",   0, 0,    cmd_free,          "I3C:FREE" },
    { "I3C:PULL",   1, 1,    cmd_pull,          "I3C:PULL <0|1>" },
    { "I3C:REGS?",  0, 0,    cmd_regs,          "I3C:REGS?" },
    { "I3C:ROLE",   1, 1,    cmd_role,          "I3C:ROLE <controller|target>" },
    { "I3C:ROLE?",  0, 0,    cmd_role_q,        "I3C:ROLE?" },
    { "I3C:TGT?",   0, 0,    cmd_tgt_q,         "I3C:TGT?" },
    { "I3C:TGT:ADDR?", 0, 0, cmd_tgt_addr,      "I3C:TGT:ADDR?" },
    { "I3C:TGT:ID", 1, 2,    cmd_tgt_id,        "I3C:TGT:ID <dcr> [instance]" },
    { "I3C:TGT:TX", 1, 0xFF, cmd_tgt_tx,        "I3C:TGT:TX <hex-bytes...>" },
    { "I3C:TGT:RX?", 0, 1,   cmd_tgt_rx,        "I3C:TGT:RX? [max]" },
    { "I3C:TGT:IBI", 0, 4,   cmd_tgt_ibi,       "I3C:TGT:IBI [hex-bytes...]" },
    { "I3C:TGT:HJ", 0, 0,    cmd_tgt_hj,        "I3C:TGT:HJ" },
    { "I3C:PORT?",  0, 0,    cmd_port_q,        "I3C:PORT?" },
    { "I3C:ENTDAA", 0, 0,    cmd_entdaa,        "I3C:ENTDAA" },
    { "I3C:RSTDAA", 0, 0,    cmd_rstdaa,        "I3C:RSTDAA" },
    { "I3C:TARGETS?", 0, 0,  cmd_targets,       "I3C:TARGETS?" },
    { "I3C:CCC:B",  1, 0xFF, cmd_ccc_broadcast, "I3C:CCC:B <ccc> [hex...]" },
    { "I3C:CCC:W",  2, 0xFF, cmd_ccc_write,     "I3C:CCC:W <addr> <ccc> [hex...]" },
    { "I3C:CCC:R",  3, 3,    cmd_ccc_read,      "I3C:CCC:R <addr> <ccc> <count>" },
    { "I3C:W",      2, 0xFF, cmd_private_write, "I3C:W <addr> <hex-bytes...>" },
    { "I3C:R",      2, 2,    cmd_private_read,  "I3C:R <addr> <count>" },
    { "I3C:PROBE",  1, 2,    cmd_probe,         "I3C:PROBE <addr> [i2c]" },
    { "I3C:LISTEN", 1, 1,    cmd_listen,        "I3C:LISTEN <0|1>" },
    { "I3C:IBI?",   0, 0,    cmd_ibi,           "I3C:IBI?" },
    { NULL, 0, 0, NULL, NULL }
};
