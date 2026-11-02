/**
 * @file bb_image.c
 * @brief Image banks — answered honestly, not emulated.
 *
 * The STM32 build used the H5's dual-bank flash to hold both images at once:
 * the bridge in one bank, the peripheral in the other, swapped by an option
 * byte and a reset in about a second with no programming. `SYS:BANK:SWAP` was
 * that, and `docs/BANKS.md` described it.
 *
 * That mechanism does not exist here. The RT1186 has no internal program flash
 * to bank; it boots XIP from 16 MB of external QSPI NOR through a boot
 * container at 0x04001000. Holding two images and choosing between them is
 * perfectly possible - two containers at different offsets, or one container
 * with two images - but it is a different mechanism with a different failure
 * mode, and pretending an option-byte swap is happening would be a lie the host
 * cannot detect.
 *
 * So the commands stay in the surface and report BB_ERR_UNSUPPORTED. A host
 * that asks gets a definite "this board cannot do that" rather than a timeout,
 * a plausible-looking wrong answer, or a missing command it has to guess about.
 * That is what error code 8 is for.
 *
 * When two-image support is built for real (it is in the spec, section 8.1),
 * this file is where it lands and the command surface will not have to change.
 */
#include "bb_image.h"

#include <string.h>

uint32_t bb_image_bank_size(void)
{
    /* 0 means "this board has no banks", which is the truth. A host that
     * divides by this gets an obvious fault rather than a wrong offset. */
    return 0u;
}

void bb_image_bank(bb_bank_t which, bb_bank_info_t *out)
{
    (void)which;
    if (out != NULL) {
        memset(out, 0, sizeof *out);
    }
}

bb_bank_t bb_image_running_bank(void)
{
    return BB_BANK_A;
}

bool bb_image_swapped(void)
{
    return false;
}

bb_swap_result_t bb_image_activate(bb_bank_t which)
{
    (void)which;
    return BB_SWAP_UNSUPPORTED;
}

const char *bb_swap_reason(bb_swap_result_t result)
{
    switch (result) {
    case BB_SWAP_OK:
        return "ok";
    case BB_SWAP_UNSUPPORTED:
        return "no-banks-on-this-board";
    default:
        return "unknown";
    }
}

/*
 * CRC-32, kept real rather than stubbed.
 *
 * The bank commands cannot work here, but the capture file format uses a CRC
 * per record - that is one of the three contracts the Rust side locked early,
 * because it is what makes a file cut short by a crash readable up to the first
 * record that does not check out. So this stays, and Phase 3 uses it.
 *
 * Bit-reversed polynomial, computed rather than tabled: 256 entries of table is
 * a kilobyte of an image that fitted in 62 KB on the STM32, and this is not on
 * a path where the difference is measurable.
 */
uint32_t bb_image_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (unsigned b = 0; b < 8u; b++) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

void bb_image_copy_str(char *dst, size_t dst_size, const char *src, size_t src_size)
{
    if (dst == NULL || dst_size == 0u) {
        return;
    }
    size_t n = 0;
    if (src != NULL) {
        while (n + 1u < dst_size && n < src_size && src[n] != 0) {
            dst[n] = src[n];
            n++;
        }
    }
    dst[n] = 0;
}
