/**
 * @file bb_usb.c
 * @brief Native USB device transport — high speed, WinUSB, no driver to install.
 *
 * Replaces the Phase 0 stub. Two bulk endpoints on a vendor-specific interface,
 * bound to Windows' own signed WinUSB by Microsoft OS 2.0 descriptors, so there
 * is no INF and nothing to sign.
 *
 * ## Where the speed comes from, and where it does not
 *
 * The STM32 build measured 5409 command round trips a second and 110 kB/s of
 * payload. Both numbers were full-speed limits, and they were limited by
 * different things - which matters, because only one of them is fixed by faster
 * silicon:
 *
 *   - **Payload** was capped by the link. Full-speed bulk is 12 Mbit/s shared
 *     with everything else on the bus, and its bulk endpoints are *fixed at 64
 *     bytes* per packet. High speed is 480 Mbit/s and bulk packets are 512
 *     bytes. That is the change this file makes.
 *
 *   - **Round trips** were capped by frame timing, not bandwidth. A full-speed
 *     frame is 1 ms; a high-speed microframe is 125 us. So the latency floor
 *     improves 8x, not 40x, and a host that issues one transfer and waits for
 *     it will see roughly that - no more. Reaching the link rate needs several
 *     transfers in flight at once, which is a *host* change, not a firmware
 *     one. See host/windows/bb_winusb_hs.c.
 *
 * Saying that plainly here because the tempting summary - "high speed is 40x
 * faster" - is wrong in the way that leads to measuring the wrong thing.
 *
 * ## The board detail that catches people
 *
 * This is **USB OTG1 on connector J63**, not the MCU-Link debug USB on J23.
 * They are different connectors and different silicon: J23 goes to the LPC55S69
 * that provides the VCOM console, J63 goes to the RT1186's own USB. A second
 * cable is needed, and plugging into J23 and waiting for a WinUSB device to
 * appear is a very easy hour to lose.
 */
#include "bb_usb.h"
#include "bb_board.h"

/* usb_device_config.h is generated into the build directory by Kconfig and must
 * come first: usb_device_dci.h reads USB_DEVICE_CONFIG_ENDPOINTS from it to size
 * its endpoint table, and without it the driver header fails to compile in a way
 * that points at the SDK rather than at this include order.
 *
 * usb_device_class.h is included for the descriptor request structs only. The
 * class framework itself is not used - this device has one vendor interface and
 * no class to speak of - but the ch9 handler passes those structs to the device
 * callback, so their definitions have to come from somewhere. */
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_ch9.h"
#include "usb_device_class.h"
#include "usb_device_dci.h"
#include "usb_phy.h"

#include "board.h"
#include "clock_config.h"

#include <string.h>

/* ---- endpoints --------------------------------------------------------- */

#define EP0_MAX_PACKET      64u
#define EP_BULK_OUT         0x01u
#define EP_BULK_IN          0x81u

/*
 * Bulk packet size is a property of the enumerated speed, not a choice:
 *
 *   full speed  64 bytes, fixed by the spec
 *   high speed  512 bytes, fixed by the spec
 *
 * Both are declared - the config descriptor is patched to match whatever the
 * host enumerated us at, and the endpoint is opened with the same figure. Get
 * these out of step and the device enumerates, answers a few packets, and then
 * stalls in a way that looks like a firmware hang.
 */
#define BULK_MAX_PACKET_FS  64u
#define BULK_MAX_PACKET_HS  512u

/* One high-speed packet per buffer. Larger buffers here would not help: the
 * host's transfer size is what sets throughput, and the device just has to keep
 * a packet available every microframe. */
#define BULK_BUF_SIZE       BULK_MAX_PACKET_HS

/* ---- line assembly ----------------------------------------------------- */
/*
 * The command protocol is one line per command, so the transport has to
 * reassemble lines from packets. A 1100-byte command line is three high-speed
 * packets, and the host library sends a 256-byte payload as one contiguous hex
 * token precisely so it lands in as few packets as possible.
 */
#define RX_RING             4096u
#define RX_MASK             (RX_RING - 1u)

static uint8_t  s_rx_ring[RX_RING];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

static uint16_t s_line_len;
static bool     s_line_overflow;

/* ---- state ------------------------------------------------------------- */

static usb_device_handle s_handle;

/* Zero classes: this device has one vendor interface and no class to speak of.
 * The list exists because it is how the stack finds deviceCallback. Must be a
 * global - USB_DeviceClassInit stores the pointer. */
static usb_status_t device_cb(usb_device_handle handle, uint32_t event, void *param);

static usb_device_class_config_list_struct_t s_class_config = {
    .config         = NULL,
    .deviceCallback = device_cb,
    .count          = 0u,
};
static volatile bool     s_configured;
static uint8_t           s_cur_config;
static uint8_t           s_speed = USB_SPEED_FULL;
static uint16_t          s_bulk_mps = BULK_MAX_PACKET_FS;

static uint32_t s_rx_bytes;
static uint32_t s_tx_bytes;
static uint32_t s_resets;
static uint32_t s_bos_reqs;
static uint32_t s_msos_reqs;
static uint32_t s_stalls;
static uint16_t s_last_setup;

/*
 * Enumeration progress counters.
 *
 * Added because debugging without them was guesswork: the device appeared in
 * Device Manager, the host reset it four times, and `bos=0` was the only clue -
 * which says nothing about whether GET_DESCRIPTOR(DEVICE) ever arrived. These
 * make the failure point visible from the console, which is the whole reason
 * the console transport is kept.
 */
static uint8_t  s_init_ok;      /**< USB_DeviceClassInit returned success */
static uint8_t  s_phy_done;     /**< the PHY init path ran */
static uint32_t s_desc_reqs;    /**< device + config + qualifier descriptors */
/* Split out per type. Lumping them together said the failure was after the
 * descriptors and before configuration, but not which descriptor the host
 * stopped at - which is the one thing needed to know where to look. */
static uint32_t s_dev_desc;
/*
 * Event trace: a shift register of 4-bit event codes, newest in the low nibble.
 *
 * The counters said enumeration stopped somewhere between the configuration
 * descriptor and SET_CONFIGURATION, but not in what order the host asked for
 * things - and the order is the whole question, because the device state the
 * SDK requires for SET_CONFIGURATION depends on SET_ADDRESS having landed
 * first. Guessing at the host's sequence from aggregate counts was costing more
 * attempts than recording it.
 *
 * 16 events fit. That is more than one enumeration attempt, which is all that
 * is needed - each attempt starts with a reset, so the boundaries are visible.
 */
#define TRC_RESET   0x1u
#define TRC_DEVDESC 0x2u
#define TRC_CFGDESC 0x3u
#define TRC_QUALDSC 0x4u
#define TRC_STRDESC 0x5u
#define TRC_BOS     0x6u
#define TRC_SETCFG  0x7u
#define TRC_SETIFC  0x8u
#define TRC_VENDOR  0x9u
#define TRC_ATTACH  0xAu
#define TRC_DETACH  0xBu
#define TRC_SUSPEND 0xCu
#define TRC_RESUME  0xDu
#define TRC_OTHER   0xEu
#define TRC_ERROR   0xFu

static uint64_t s_trace;

/* The MS OS 2.0 request as the host actually sent it. `msos=1` proved a vendor
 * request with the right bRequest arrived, but not that it was the descriptor
 * -set request (wIndex 7) nor how many bytes the host wanted - and a host that
 * asks for fewer bytes than the set contains gets a truncated set, which it
 * then rejects silently. Worth four bytes of RAM to not have to guess. */
static uint8_t  s_vnd_reqtype;
static uint16_t s_vnd_windex;
static uint16_t s_vnd_wlength;
static uint16_t s_vnd_sent;

static void trc(uint32_t code)
{
    s_trace = (s_trace << 4) | (uint64_t)(code & 0xFu);
}
static uint32_t s_cfg_desc;
static uint32_t s_qual_desc;
static uint32_t s_str_reqs;
static uint32_t s_setcfg;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_ep_out[BULK_BUF_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_ep_in[BULK_BUF_SIZE];

/* ---- descriptors ------------------------------------------------------- */

#define DESC_DEVICE         0x01u
#define DESC_CONFIG         0x02u
#define DESC_STRING         0x03u
#define DESC_INTERFACE      0x04u
#define DESC_ENDPOINT       0x05u
#define DESC_DEV_QUALIFIER  0x06u
#define DESC_OTHER_SPEED    0x07u
#define DESC_BOS            0x0Fu

#define MS_OS_20_VENDOR_CODE  0x42u

static uint8_t s_device_desc[] = {
    18, DESC_DEVICE,
    /* bcdUSB 2.01, not 2.00. A BOS descriptor - which is how MS OS 2.0
     * descriptors are advertised - is only requested by Windows from a device
     * claiming 2.01 or later. Declaring 2.00 makes the whole WinUSB binding
     * path silently not happen. */
    0x01, 0x02,
    0xFFu,                  /* bDeviceClass: vendor specific                 */
    0x00u, 0x00u,
    EP0_MAX_PACKET,
    /* pid.codes 0x1209, as the STM32 build used - not NXP's 0x1FC9, which is
     * not ours to hand out product IDs from. */
    0x09u, 0x12u,           /* idVendor  0x1209 - pid.codes                  */
    /* 0x0002, distinct from the STM32 board's 0x0001, and that distinctness is
     * not cosmetic: Windows caches a device's MS OS 2.0 descriptor set against
     * VID/PID/bcdDevice. Reusing 0x0001 with a different set gets the *cached
     * STM32 one* served instead - a device that binds WinUSB with the wrong
     * properties, explicable by reading neither firmware. The STM32 build hit
     * the same trap from the other side and bumped bcdDevice to 1.01. */
    0x03u, 0x00u,           /* idProduct 0x0003 - RT1186                     */
    /* bcdDevice 1.02, bumped deliberately. Windows caches a device's
     * descriptors and its MS OS 2.0 set against VID/PID/bcdDevice, and it
     * caches *failures* too - a device node that failed to start stays
     * failed and is not re-enumerated from scratch. The first attempts here
     * failed for a real reason (no PHY init); after fixing that, Windows was
     * still reporting Code 43 from the cached node. Bumping this makes it a
     * device instance Windows has never seen. The STM32 build bumped its own
     * to 1.01 for exactly this reason. */
    0x03u, 0x01u,           /* bcdDevice 1.03                                */
    1, 2, 3,                /* iManufacturer, iProduct, iSerialNumber        */
    1                       /* bNumConfigurations                            */
};

/*
 * A high-speed-capable device must answer GET_DESCRIPTOR(DEVICE_QUALIFIER).
 * Omitting it is legal only for a full-speed-only device, and Windows uses its
 * absence to decide the device cannot do high speed - so a device that is
 * physically capable enumerates at full speed and every measurement afterwards
 * is of the wrong thing.
 */
static const uint8_t s_qualifier_desc[] = {
    10, DESC_DEV_QUALIFIER,
    0x01, 0x02,
    0xFFu, 0x00u, 0x00u,
    EP0_MAX_PACKET,
    1,                      /* bNumConfigurations                            */
    0
};

#define CONFIG_DESC_SIZE  32u

/* wMaxPacketSize at offsets 22 and 29 is patched to match the enumerated
 * speed - see usb_patch_config_for_speed(). */
static uint8_t s_config_desc[CONFIG_DESC_SIZE] = {
    9, DESC_CONFIG,
    CONFIG_DESC_SIZE, 0x00,
    1,                      /* bNumInterfaces                                */
    1,                      /* bConfigurationValue                           */
    0,                      /* iConfiguration                                */
    0x80u,                  /* bmAttributes: bus powered                     */
    250,                    /* bMaxPower: 500 mA                             */

    9, DESC_INTERFACE,
    0,                      /* bInterfaceNumber                              */
    0,                      /* bAlternateSetting                             */
    2,                      /* bNumEndpoints                                 */
    /*
     * 0xFF/0x00/0x00, not 0xFF/0xFF/0xFF. Vendor-specific class with a zero
     * subclass and protocol is what the STM32 build advertises, and its device
     * nodes on this machine carry USB\MS_COMP_WINUSB and winusb.inf. Keeping
     * the ported image byte-compatible with the one known to bind removes a
     * difference with no upside.
     */
    0xFFu, 0x00u, 0x00u,    /* bInterfaceClass/SubClass/Protocol             */
    0,                      /* iInterface                                    */

    7, DESC_ENDPOINT,
    EP_BULK_OUT,
    0x02u,                  /* bmAttributes: bulk                            */
    BULK_MAX_PACKET_FS, 0x00,
    0,                      /* bInterval: ignored for bulk                   */

    7, DESC_ENDPOINT,
    EP_BULK_IN,
    0x02u,
    BULK_MAX_PACKET_FS, 0x00,
    0
};

#define CFG_OFF_OUT_MPS  22u
#define CFG_OFF_IN_MPS   29u

static void usb_patch_config_for_speed(uint8_t speed)
{
    uint16_t mps = (speed == USB_SPEED_HIGH) ? BULK_MAX_PACKET_HS
                                             : BULK_MAX_PACKET_FS;
    s_bulk_mps = mps;
    s_config_desc[CFG_OFF_OUT_MPS]      = (uint8_t)(mps & 0xFFu);
    s_config_desc[CFG_OFF_OUT_MPS + 1u] = (uint8_t)(mps >> 8);
    s_config_desc[CFG_OFF_IN_MPS]       = (uint8_t)(mps & 0xFFu);
    s_config_desc[CFG_OFF_IN_MPS + 1u]  = (uint8_t)(mps >> 8);
}

#define MSOS20_SET_HEADER_LEN  10u
#define MSOS20_COMPAT_ID_LEN   20u
#define MSOS20_PROP_NAME_LEN   42u   /* "DeviceInterfaceGUIDs" + NUL, UTF-16 */
#define MSOS20_PROP_DATA_LEN   80u   /* the GUID + NUL + list terminator     */
#define MSOS20_REG_PROP_LEN    (10u + MSOS20_PROP_NAME_LEN + MSOS20_PROP_DATA_LEN)
#define MSOS20_TOTAL_LEN       (MSOS20_SET_HEADER_LEN + MSOS20_COMPAT_ID_LEN +                                 MSOS20_REG_PROP_LEN)

/* BOS with the Microsoft OS 2.0 platform capability. This is the whole reason
 * Windows binds WinUSB with nothing installed. */
static const uint8_t s_bos_desc[] = {
    5, DESC_BOS,
    33, 0x00,               /* wTotalLength                                  */
    1,                      /* bNumDeviceCaps                                */

    28, 0x10,               /* DEVICE CAPABILITY, PLATFORM                   */
    0x05u, 0x00u,
    /* GUID {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}, little-endian fields */
    0xDFu, 0x60u, 0xDDu, 0xD8u, 0x89u, 0x45u, 0xC7u, 0x4Cu,
    0x9Cu, 0xD2u, 0x65u, 0x9Du, 0x9Eu, 0x64u, 0x8Au, 0x9Fu,
    0x00u, 0x00u, 0x03u, 0x06u,   /* dwWindowsVersion: 8.1                   */
    /* Derived, not written twice: a BOS that advertises a length the set does
     * not have is a host-side parse failure with no symptom on this side. */
    (uint8_t)(MSOS20_TOTAL_LEN & 0xFFu), (uint8_t)(MSOS20_TOTAL_LEN >> 8),
    MS_OS_20_VENDOR_CODE,
    0x00u
};

/*
 * The MS OS 2.0 descriptor set. Two things in it do the work:
 *
 *   CompatibleID "WINUSB"     tells Windows which driver to load
 *   DeviceInterfaceGUID       gives the device a stable interface GUID so a
 *                             host can find it with SetupDi* without an INF
 *
 * The GUID below must match the one the host library looks for. If they drift
 * apart the device binds WinUSB correctly and the host then cannot find it,
 * which presents as "the driver installed but the tool sees nothing".
 */
/*
 * Microsoft OS 2.0 descriptor set - FLAT.
 *
 * The feature descriptors sit directly under the set header, with no
 * configuration subset and no function subset, so they apply to the whole
 * device. That is what the STM32 build in E:/STMicro does, and its device
 * nodes on this machine carry USB\MS_COMP_WINUSB with winusb.inf bound.
 *
 * The first version here used the nested form - set header, configuration
 * subset, function subset, then the features. Every length and type field in
 * it was correct, verified byte by byte out of the linked image against the
 * spec, and the host fetched all 178 bytes of it (bmRequestType 0xC0, wIndex 7,
 * wLength 178, answered in full). Windows then silently declined to act on it:
 * no MS_COMP_WINUSB compatible id, no DeviceInterfaceGUIDs registry value, and
 * CM_PROB_FAILED_INSTALL with no log entry naming a reason.
 *
 * So the nesting is the difference, not the contents. Worth recording because
 * the failure mode teaches nothing on its own: a structurally valid set that
 * the host reads completely and ignores looks identical to a caching problem,
 * which is where the time went.
 */
static const uint8_t s_msos20_desc[MSOS20_TOTAL_LEN] = {
    /* Descriptor set header */
    MSOS20_SET_HEADER_LEN, 0x00u,
    0x00u, 0x00u,                       /* wDescriptorType: SET_HEADER       */
    0x00u, 0x00u, 0x03u, 0x06u,         /* dwWindowsVersion: 8.1             */
    (uint8_t)(MSOS20_TOTAL_LEN & 0xFFu), (uint8_t)(MSOS20_TOTAL_LEN >> 8),

    /* Compatible ID: the line that makes Windows load WinUSB */
    MSOS20_COMPAT_ID_LEN, 0x00u,
    0x03u, 0x00u,                       /* FEATURE_COMPATIBLE_ID             */
    'W','I','N','U','S','B', 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,

    /* Registry property: DeviceInterfaceGUIDs (REG_MULTI_SZ) */
    (uint8_t)(MSOS20_REG_PROP_LEN & 0xFFu), (uint8_t)(MSOS20_REG_PROP_LEN >> 8),
    0x04u, 0x00u,                       /* FEATURE_REG_PROPERTY              */
    0x07u, 0x00u,                       /* wPropertyDataType: REG_MULTI_SZ   */
    MSOS20_PROP_NAME_LEN, 0x00u,
    'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'I',0,'n',0,'t',0,'e',0,
    'r',0,'f',0,'a',0,'c',0,'e',0,'G',0,'U',0,'I',0,'D',0,'s',0,
    0x00u, 0x00u,
    MSOS20_PROP_DATA_LEN, 0x00u,
    /* {1D4B2E5F-9C3A-4F8B-B0E7-2A6C8D3F5A91}, UTF-16LE.
     *
     * Not a free choice. It must be byte-for-byte the GUID that BB_WINUSB_GUID
     * in the host library's bb_winusb.c searches for: the host enumerates with
     * SetupDiGetClassDevs on that GUID and nothing else. If the two disagree,
     * the device binds WinUSB perfectly and the host then finds no devices at
     * all - which is a failure with no error message anywhere. The first
     * version of this file invented its own GUID and would have done that. */
    '{',0,'1',0,'D',0,'4',0,'B',0,'2',0,'E',0,'5',0,'F',0,'-',0,
    '9',0,'C',0,'3',0,'A',0,'-',0,'4',0,'F',0,'8',0,'B',0,'-',0,
    'B',0,'0',0,'E',0,'7',0,'-',0,'2',0,'A',0,'6',0,'C',0,'8',0,
    'D',0,'3',0,'F',0,'5',0,'A',0,'9',0,'1',0,'}',0,
    0x00u, 0x00u, 0x00u, 0x00u
};


/* ---- string descriptors ------------------------------------------------ */
/*
 * The device descriptor promises iManufacturer/iProduct/iSerialNumber, so
 * Windows asks for all three. A device that promises a string and then stalls
 * the request enumerates unreliably in a way that looks intermittent.
 *
 * The serial number matters more than the other two, and not for cosmetics: it
 * is what lets a host pick a specific board on a bench with several attached.
 * The STM32 project settled the native-USB-vs-VCP question partly on this -
 * "the first COM port in the list" is a coin flip, and driving the wrong board
 * looks exactly like the firmware misbehaving. So the serial is derived from
 * the die's own unique ID rather than being a constant.
 */
static const uint8_t s_str_lang[] = {
    4, DESC_STRING, 0x09u, 0x04u        /* 0x0409 en-US */
};

/* UTF-16LE, as all USB strings are. */
static const uint8_t s_str_manufacturer[] = {
    18, DESC_STRING,
    'N',0, 'X',0, 'P',0, ' ',0, 'R',0, 'T',0, '1',0, '1',0
};

static const uint8_t s_str_product[] = {
    26, DESC_STRING,
    'B',0,'U',0,'S',0,'B',0,'R',0,'I',0,'D',0,'G',0,'E',0,'-',0,'R',0,'T',0
};

/* Two hex digits per UID byte, 16 bytes of UID -> 32 chars -> 66-byte
 * descriptor. Built once at init because the UID does not change. */
static uint8_t s_str_serial[2u + 32u * 2u];

/* Offset of iSerialNumber in the device descriptor, derived from the layout
 * above rather than written as a literal: bLength, bDescriptorType, bcdUSB(2),
 * bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0,
 * idVendor(2), idProduct(2), bcdDevice(2), iManufacturer, iProduct. */
#define DEV_OFF_ISERIAL  16u

static void build_serial_string(void)
{
    static const char hex[] = "0123456789ABCDEF";
    size_t   o = 0;
    uint32_t any = 0u;

    for (uint8_t w = 0; w < 4u; w++) {
        any |= bb_uid_word(w);
    }

    /*
     * No UID, no serial number - and this is a correctness fix, not a cosmetic
     * one.
     *
     * bb_board_silicon() does not read OCOTP yet, so bb_uid_word() returns zero
     * on every word. Advertising the resulting string still "works" for one
     * board, but Windows keys the device node on VID + PID + serial: every
     * RT1186 running this image would claim the same serial and collapse into a
     * single device node, so two boards could not be used on one machine. A
     * constant serial is worse than none.
     *
     * iSerialNumber = 0 is the spec's way of saying the device has no serial,
     * and hosts then fall back to a port-derived instance id, which is unique
     * per board by construction. When the OCOTP read lands, this starts
     * publishing a real one with no other change.
     */
    if (any == 0u) {
        s_device_desc[DEV_OFF_ISERIAL] = 0u;
        return;
    }
    s_device_desc[DEV_OFF_ISERIAL] = 3u;

    s_str_serial[o++] = (uint8_t)sizeof s_str_serial;
    s_str_serial[o++] = DESC_STRING;
    for (uint8_t w = 0; w < 4u; w++) {
        uint32_t v = bb_uid_word(w);
        for (int nib = 7; nib >= 0; nib--) {
            s_str_serial[o++] = (uint8_t)hex[(v >> (nib * 4)) & 0xFu];
            s_str_serial[o++] = 0u;
        }
    }
}

/* ---- ring ------------------------------------------------------------- */

static void rx_push(const uint8_t *d, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        uint16_t next = (uint16_t)((s_rx_head + 1u) & RX_MASK);
        if (next == s_rx_tail) {
            /* Full. Dropping the newest keeps a half-assembled line intact,
             * which reads as a malformed command rather than as silent loss -
             * the same choice the console ring makes. */
            return;
        }
        s_rx_ring[s_rx_head] = d[i];
        s_rx_head            = next;
    }
}

static int rx_pop(void)
{
    if (s_rx_tail == s_rx_head) {
        return -1;
    }
    uint8_t c = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & RX_MASK);
    return (int)c;
}

/* ---- endpoint callbacks ------------------------------------------------ */

static usb_status_t ep_out_cb(usb_device_handle handle,
                              usb_device_endpoint_callback_message_struct_t *msg,
                              void *param)
{
    (void)handle;
    (void)param;

    if (!s_configured) {
        return kStatus_USB_Success;
    }
    if (msg->length != USB_UNINITIALIZED_VAL_32 && msg->length > 0u) {
        rx_push(msg->buffer, msg->length);
        s_rx_bytes += msg->length;
    }
    /* Re-arm immediately, always. An OUT endpoint with no buffer posted NAKs,
     * and a host that is streaming a long command sees that as backpressure -
     * so the one thing this callback must never do is return without queueing
     * the next receive. */
    return USB_DeviceRecvRequest(s_handle, EP_BULK_OUT, s_ep_out, s_bulk_mps);
}

static volatile bool s_tx_busy;

static usb_status_t ep_in_cb(usb_device_handle handle,
                             usb_device_endpoint_callback_message_struct_t *msg,
                             void *param)
{
    (void)handle;
    (void)param;
    if (msg->length != USB_UNINITIALIZED_VAL_32) {
        s_tx_bytes += msg->length;
    }
    s_tx_busy = false;
    return kStatus_USB_Success;
}

static usb_status_t open_endpoints(void)
{
    usb_device_endpoint_init_struct_t ep;
    usb_device_endpoint_callback_struct_t cb;

    memset(&ep, 0, sizeof ep);
    memset(&cb, 0, sizeof cb);

    ep.zlt             = 0u;
    ep.transferType    = USB_ENDPOINT_BULK;
    ep.maxPacketSize   = s_bulk_mps;
    ep.endpointAddress = EP_BULK_OUT;
    cb.callbackFn      = ep_out_cb;
    if (USB_DeviceInitEndpoint(s_handle, &ep, &cb) != kStatus_USB_Success) {
        return kStatus_USB_Error;
    }

    ep.endpointAddress = EP_BULK_IN;
    cb.callbackFn      = ep_in_cb;
    if (USB_DeviceInitEndpoint(s_handle, &ep, &cb) != kStatus_USB_Success) {
        return kStatus_USB_Error;
    }
    s_tx_busy = false;
    return USB_DeviceRecvRequest(s_handle, EP_BULK_OUT, s_ep_out, s_bulk_mps);
}

/* ---- device callback --------------------------------------------------- */

static usb_status_t device_cb(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t st = kStatus_USB_Error;

    (void)handle;
    switch (event) {
    case kUSB_DeviceEventBusReset:
        trc(TRC_RESET);
        s_resets++;
        s_configured = false;
        /* The speed is only knowable after the reset completes, and the
         * endpoints must be opened with the packet size that speed implies -
         * which is why this is read here rather than assumed at init. */
        if (USB_DeviceGetStatus(s_handle, kUSB_DeviceStatusSpeed, &s_speed)
            == kStatus_USB_Success) {
            usb_patch_config_for_speed(s_speed);
        }
        st = kStatus_USB_Success;
        break;

    case kUSB_DeviceEventSetConfiguration:
        s_setcfg++; trc(TRC_SETCFG);
        if (param == NULL) {
            break;
        }
        /* Configuration 0 is "unconfigure me", and it must be accepted - the
         * host uses it to put the device back to the Address state. Refusing it
         * stalls a legal request. */
        if (*(uint8_t *)param == 0u) {
            s_configured = false;
            s_cur_config = 0u;
            st = kStatus_USB_Success;
        } else if (*(uint8_t *)param == 1u) {
            s_configured = (open_endpoints() == kStatus_USB_Success);
            s_cur_config = 1u;
            st = kStatus_USB_Success;
        }
        break;

    case kUSB_DeviceEventGetDeviceDescriptor: {
        usb_device_get_device_descriptor_struct_t *d = param;
        s_desc_reqs++; s_dev_desc++; trc(TRC_DEVDESC);
        d->buffer = (uint8_t *)s_device_desc;
        d->length = sizeof s_device_desc;
        st = kStatus_USB_Success;
        break;
    }
    case kUSB_DeviceEventGetConfigurationDescriptor: {
        usb_device_get_configuration_descriptor_struct_t *d = param;
        s_desc_reqs++; s_cfg_desc++; trc(TRC_CFGDESC);
        d->buffer = s_config_desc;
        d->length = sizeof s_config_desc;
        st = kStatus_USB_Success;
        break;
    }
    case kUSB_DeviceEventGetDeviceQualifierDescriptor: {
        usb_device_get_device_qualifier_descriptor_struct_t *d = param;
        s_desc_reqs++; s_qual_desc++; trc(TRC_QUALDSC);
        d->buffer = (uint8_t *)s_qualifier_desc;
        d->length = sizeof s_qualifier_desc;
        st = kStatus_USB_Success;
        break;
    }
    case kUSB_DeviceEventGetBOSDescriptor: {
        usb_device_get_bos_descriptor_struct_t *d = param;
        s_bos_reqs++; trc(TRC_BOS);
        d->buffer = (uint8_t *)s_bos_desc;
        d->length = sizeof s_bos_desc;
        st = kStatus_USB_Success;
        break;
    }
    case kUSB_DeviceEventGetStringDescriptor: {
        usb_device_get_string_descriptor_struct_t *d = param;
        s_str_reqs++; trc(TRC_STRDESC);
        switch (d->stringIndex) {
        case 0u: d->buffer = (uint8_t *)s_str_lang;         d->length = sizeof s_str_lang; break;
        case 1u: d->buffer = (uint8_t *)s_str_manufacturer; d->length = sizeof s_str_manufacturer; break;
        case 2u: d->buffer = (uint8_t *)s_str_product;      d->length = sizeof s_str_product; break;
        case 3u: d->buffer = s_str_serial;                  d->length = sizeof s_str_serial; break;
        default: return kStatus_USB_InvalidRequest;
        }
        st = kStatus_USB_Success;
        break;
    }

    /*
     * The MS OS 2.0 descriptor set arrives as a *vendor* request on endpoint 0,
     * not as a standard GET_DESCRIPTOR - which is why it cannot be served from
     * the descriptor table and has to be handled here.
     *
     * Serving it is the entire reason Windows binds its own signed WinUSB with
     * nothing installed. Windows asks exactly once, during the first
     * enumeration of a device instance it has never seen before - so this is
     * the part of the path that cannot be tested by re-plugging the board you
     * developed against. It needs a device instance Windows has not met.
     */
    case kUSB_DeviceEventVendorRequest: {
        usb_device_control_request_struct_t *req = param;
        if (req == NULL || req->setup == NULL) {
            break;
        }
        s_last_setup   = req->setup->bRequest;
        s_vnd_reqtype  = req->setup->bmRequestType;
        s_vnd_windex   = req->setup->wIndex;
        s_vnd_wlength  = req->setup->wLength;
        if (req->setup->bRequest == MS_OS_20_VENDOR_CODE &&
            (req->setup->bmRequestType & USB_REQUEST_TYPE_TYPE_MASK)
                == USB_REQUEST_TYPE_TYPE_VENDOR) {
            s_msos_reqs++;
            req->buffer = (uint8_t *)s_msos20_desc;
            req->length = sizeof s_msos20_desc;
            s_vnd_sent  = (uint16_t)sizeof s_msos20_desc;
            st = kStatus_USB_Success;
        } else {
            s_stalls++;
        }
        break;
    }

    /*
     * The four requests below are why the device would not enumerate.
     *
     * `device_cb` leaves `st = kStatus_USB_Error` for anything it does not
     * handle, and ch9 turns that into an endpoint-0 stall. Windows issues
     * GET_CONFIGURATION (and may issue SET_INTERFACE) immediately after reading
     * the descriptors - so it read 12 descriptors happily, hit a stall on the
     * next request, and failed the device with Code 43. The counters showed
     * `descriptors=12 strings=0 setcfg=0`, which said the failure was *after*
     * the descriptors and *before* configuration, but not which request.
     *
     * Found by building the SDK's own usb_device_hid_generic for this board
     * unmodified. It enumerated first time, which proved the board, the cable,
     * the PHY and the clock were all fine and the fault was in this file - and
     * diffing its device callback against this one showed it handles these four
     * and this one did not.
     *
     * The lesson worth keeping: a callback that defaults to an error stalls
     * every request its author did not think of, and the host's diagnosis for
     * that is indistinguishable from a broken descriptor.
     */
    case kUSB_DeviceEventGetConfiguration:
        if (param != NULL) {
            *(uint8_t *)param = s_cur_config;
            st = kStatus_USB_Success;
        }
        break;

    case kUSB_DeviceEventGetInterface:
        if (param != NULL) {
            /* One interface, one alternate setting, so the answer is always 0.
             * The low byte carries the alternate setting back. */
            *(uint16_t *)param = 0u;
            st = kStatus_USB_Success;
        }
        break;

    case kUSB_DeviceEventSetInterface:
        if (param != NULL) {
            uint16_t v         = *(uint16_t *)param;
            uint8_t  interface = (uint8_t)((v & 0xFF00u) >> 8);
            uint8_t  alt       = (uint8_t)(v & 0x00FFu);
            /* Accept only interface 0, alternate 0 - the only one that exists.
             * Anything else is a request for something this device does not
             * have, and a stall is the correct answer to that. */
            if (interface == 0u && alt == 0u) {
                st = kStatus_USB_Success;
            }
        }
        break;

    case kUSB_DeviceEventSetRemoteWakeup:
        /* Accepted and ignored. This device does not wake the host, but the
         * configuration descriptor does not claim to either (bmAttributes has
         * the remote-wakeup bit clear), so the host will not rely on it. */
        st = kStatus_USB_Success;
        break;

    default:
        break;
    }
    return st;
}

/* ---- public API -------------------------------------------------------- */

void bb_usb_init(void)
{
    s_rx_head = s_rx_tail = 0u;
    s_line_len = 0u;
    s_line_overflow = false;
    s_configured = false;

    build_serial_string();

    /*
     * The USB PHY and its 480 MHz PLL, before anything else.
     *
     * Skipped in the first version, and the symptom was instructive: Windows
     * saw the device (VID_1209/PID_0002 appeared in Device Manager), issued
     * four bus resets, and gave up with an error - while the firmware reported
     * `bos=0 msos=0`, meaning not one descriptor request had reached the
     * handler. The digital side was fine; the analogue front end was never
     * configured, so nothing could be signalled across it.
     *
     * A device that appears and then fails to enumerate looks like a descriptor
     * bug, which is where the time goes if the PHY is not the first suspect.
     *
     * The calibration constants come from the board header rather than being
     * chosen here - they are trim values for this board's traces.
     */
    usb_phy_config_struct_t phy = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };
    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, BB_XTAL_HZ);
    CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, BB_XTAL_HZ);
    USB_EhciPhyInit(kUSB_ControllerEhci0, BOARD_XTAL0_CLK_HZ, &phy);
    s_phy_done = 1u;

    /*
     * Brought up through the *class* layer with zero classes registered, not
     * through USB_DeviceInit directly - and this is not a style choice.
     *
     * The SDK's ch9 handler routes GET_DESCRIPTOR and vendor requests through
     * USB_DeviceClassCallback(), which forwards every event to
     * configList->deviceCallback. A device initialised with USB_DeviceInit()
     * alone never receives them: it would compile, link, and then fail to
     * enumerate, because nothing would answer the very first descriptor
     * request. The class loop is a no-op with count = 0, so this costs nothing
     * and puts the callback where the stack actually calls it.
     *
     * EHCI, not KHCI. KHCI is the full-speed controller; asking for it would
     * cap the link at 12 Mbit/s no matter what the descriptors claim, and the
     * device would enumerate perfectly while being far slower than this part
     * can manage.
     */
    if (USB_DeviceClassInit(kUSB_ControllerEhci0, &s_class_config, &s_handle)
        != kStatus_USB_Success) {
        s_init_ok = 0u;
        return;
    }
    s_init_ok = 1u;
    /* USB_OTG1, the connector at J63 - not the MCU-Link debug USB at J23. */
    HAL_NVIC_SetPriority(USB_OTG1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USB_OTG1_IRQn);
    (void)USB_DeviceRun(s_handle);
}

void USB_OTG1_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(s_handle);
    SDK_ISR_EXIT_BARRIER;
}

void bb_usb_irq(void)
{
    USB_DeviceEhciIsrFunction(s_handle);
}

bool bb_usb_ready(void)
{
    return s_configured;
}

void bb_usb_write(const void *data, size_t len)
{
    const uint8_t *p = data;

    if (!s_configured) {
        return;
    }
    while (len > 0u) {
        size_t chunk = (len < BULK_BUF_SIZE) ? len : BULK_BUF_SIZE;

        /* One transfer outstanding on IN. The reply to a command is short and
         * strictly ordered after it, so there is nothing to gain from queueing
         * several - and a queue here would let two replies interleave. */
        uint32_t spin = 0u;
        while (s_tx_busy) {
            if (++spin > 4000000u) {
                return;         /* host stopped reading; drop rather than hang */
            }
        }
        memcpy(s_ep_in, p, chunk);
        s_tx_busy = true;
        if (USB_DeviceSendRequest(s_handle, EP_BULK_IN, s_ep_in,
                                  (uint32_t)chunk) != kStatus_USB_Success) {
            s_tx_busy = false;
            return;
        }
        p   += chunk;
        len -= chunk;
    }
}

int bb_usb_getchar(void)
{
    return rx_pop();
}

int bb_usb_getline(char *buf, size_t cap)
{
    for (;;) {
        int ci = rx_pop();
        if (ci < 0) {
            return -1;
        }
        char c = (char)ci;

        if (c == '\r' || c == '\n') {
            if (s_line_len == 0u && !s_line_overflow) {
                continue;
            }
            bool over = s_line_overflow;
            buf[s_line_len] = 0;
            int n = (int)s_line_len;
            s_line_len      = 0u;
            s_line_overflow = false;
            return over ? -2 : n;
        }
        if ((size_t)s_line_len + 1u >= cap) {
            s_line_overflow = true;
            continue;
        }
        buf[s_line_len++] = c;
    }
}

void bb_usb_stats(uint32_t *rx_bytes, uint32_t *tx_bytes, uint32_t *resets)
{
    if (rx_bytes != NULL) { *rx_bytes = s_rx_bytes; }
    if (tx_bytes != NULL) { *tx_bytes = s_tx_bytes; }
    if (resets   != NULL) { *resets   = s_resets; }
}

void bb_usb_diag(uint32_t *bos, uint32_t *msos, uint32_t *stalls, uint16_t *last)
{
    if (bos    != NULL) { *bos    = s_bos_reqs; }
    if (msos   != NULL) { *msos   = s_msos_reqs; }
    if (stalls != NULL) { *stalls = s_stalls; }
    if (last   != NULL) { *last   = s_last_setup; }
}

void bb_usb_link(uint32_t *dppu, uint32_t *frame, uint32_t *istr)
{
    /* Repurposed for what is actually diagnostic on this part: the enumerated
     * speed and the bulk packet size in force. A host measuring throughput
     * needs to know which of the two it got, and "it looked fast" is not an
     * answer - a device that quietly enumerated at full speed is the single
     * most likely reason a high-speed number comes out low. */
    if (dppu  != NULL) { *dppu  = (s_speed == USB_SPEED_HIGH) ? 480u : 12u; }
    if (frame != NULL) { *frame = s_bulk_mps; }
    /* Packed so one field carries the three facts that matter when nothing
     * enumerates: bit 0 configured, bit 1 USB_DeviceClassInit succeeded,
     * bit 2 the PHY init ran, and the string-descriptor count above it. */
    if (istr  != NULL) {
        *istr = (s_configured ? 1u : 0u)
              | (s_init_ok ? 2u : 0u)
              | (s_phy_done ? 4u : 0u)
              | (s_str_reqs << 8);
    }
}

bool bb_usb_typec(uint32_t *cc1, uint32_t *cc2)
{
    /* No CC sense the firmware can read: USB_OTG1 goes straight to J63. false
     * means "this board cannot tell you", which is not the same answer as
     * "nothing is attached". */
    (void)cc1;
    (void)cc2;
    return false;
}

/*
 * Reported through the existing two-field hook rather than a new command, so
 * bb_cmd.c and every host that parses SYS:USB? stay unchanged. `packets` is the
 * descriptor-request count and `bytes` the SET_CONFIGURATION count - between
 * them they say exactly how far enumeration got:
 *
 *   0 / 0   the host never asked for a descriptor: PHY, clock or interrupt
 *   n / 0   descriptors served but never configured: a descriptor is malformed
 *   n / n   configured; if state is still no-host, the endpoints failed to open
 */
void bb_usb_ep0_diag(uint16_t *packets, uint16_t *bytes)
{
    if (packets != NULL) { *packets = (uint16_t)s_desc_reqs; }
    if (bytes   != NULL) { *bytes   = (uint16_t)s_setcfg; }
}

void bb_usb_vendor_diag(uint32_t *reqtype, uint32_t *windex, uint32_t *wlength,
                        uint32_t *sent)
{
    if (reqtype != NULL) { *reqtype = s_vnd_reqtype; }
    if (windex  != NULL) { *windex  = s_vnd_windex; }
    if (wlength != NULL) { *wlength = s_vnd_wlength; }
    if (sent    != NULL) { *sent    = s_vnd_sent; }
}

void bb_usb_trace(uint64_t *events, uint32_t *dev_desc, uint32_t *cfg_desc,
                  uint32_t *state)
{
    if (events   != NULL) { *events   = s_trace; }
    if (dev_desc != NULL) { *dev_desc = s_dev_desc; }
    if (cfg_desc != NULL) { *cfg_desc = s_cfg_desc; }
    if (state != NULL) {
        /*
         * The controller's own view of where enumeration got to, which is the
         * one the SDK's chapter-9 layer tests: USB_DeviceCh9SetConfiguration
         * refuses unless this is Address or Configured, so a device stuck in
         * Default has had SET_ADDRESS fail silently. 0xFF means the query
         * itself failed, which is different from state 0.
         */
        uint8_t s = 0xFFu;
        if (s_handle != NULL) {
            (void)USB_DeviceGetStatus(s_handle, kUSB_DeviceStatusDeviceState, &s);
        }
        *state = s;
    }
}
