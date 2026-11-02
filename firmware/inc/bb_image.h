/**
 * @file bb_image.h
 * @brief What an image says about itself, and switching between two of them.
 *
 * The flash on every board here is dual bank - FLASH_BANK_SIZE is half of
 * FLASH_SIZE, so 256 KB per bank on the H533RE and 1 MB on the other two -
 * against an image of well under 100 KB. So both banks can hold a complete
 * firmware and the board can switch between them by flipping one option byte
 * and resetting. No flashing, no debugger, about a second.
 *
 * Bank A is physical bank 1, bank B is physical bank 2. Which one is *running*
 * is a separate question: the SWAP_BANK option byte decides which physical bank
 * appears at 0x08000000, and the other is always visible at
 * 0x08000000 + FLASH_BANK_SIZE. So the image that is running is whichever bank
 * the swap bit maps low, and reading "the other image" needs no swap at all -
 * it is just memory.
 *
 * # The header
 *
 * Every image carries this struct at a fixed offset, which is what lets the
 * running firmware say what is in the *other* bank without executing it - and
 * what lets the host describe an image sitting in a file. Same struct, same
 * offset, read either way.
 *
 * The offset is fixed rather than found through a pointer for the same reason:
 * a pointer in the other bank would have to be dereferenced relative to an
 * image that is not mapped where it was linked.
 *
 * # The checksum
 *
 * CRC-32, the reflected one with the standard 0xEDB88320 polynomial - the same
 * value zlib.crc32 and every other common implementation produce, so the
 * stamping tool needs no special algorithm.
 *
 * It covers bytes [0, image_size) of the image, counting from the bank base,
 * with the four bytes of the crc field itself read as zero. Excluding the whole
 * header would have avoided the self-reference more simply, but then a header
 * that had been corrupted or half-written would still verify - and the header is
 * exactly what the switch decision is made from.
 *
 * Nothing swaps banks without this checking out. That is the whole safety story:
 * a half-written or wrong-part image in the other bank means the switch is
 * refused, and the board carries on running what it is running.
 */
#ifndef BB_IMAGE_H
#define BB_IMAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** 'B' 'B' 'I' 'M', little-endian. */
#define BB_IMAGE_MAGIC        0x4D494242u

/** Where the header sits, from the base of the bank holding the image.
 *
 * 1 KB in: past any vector table this family can have (the largest is a few
 * hundred bytes) and on a round number, so it can be found by eye in a hex
 * dump of a .bin. */
#define BB_IMAGE_HEADER_OFF   0x400u

#define BB_IMAGE_HEADER_VERSION  1u

/**
 * @brief Self-description stamped into every image.
 *
 * Fixed-width fields and no pointers, because it is read out of a bank that is
 * not mapped where it was linked, and out of a file by a program that is not
 * this one. Strings are NUL-padded and not guaranteed NUL-terminated when full;
 * bb_image_copy_str() is how to read one safely.
 */
typedef struct {
    uint32_t magic;              /**< BB_IMAGE_MAGIC */
    uint16_t header_version;     /**< BB_IMAGE_HEADER_VERSION */
    uint16_t header_size;        /**< sizeof this struct */

    char     board[24];          /**< BB_BOARD_NAME, e.g. "NUCLEO-H533RE" */
    char     profile[16];        /**< "bridge" or "peripheral" */
    char     version[16];        /**< BB_VERSION */
    char     built[24];          /**< compiler date and time */

    /** Bytes covered by the checksum, from the bank base. Stamped after the
     *  link, because nothing before it knows how long the image is. */
    uint32_t image_size;
    uint32_t image_crc;          /**< see the file comment for what it covers */

    /** The bank this image was linked for, so a host reading the header knows
     *  where the other bank starts without a table of boards. */
    uint32_t bank_size;
    /** Expected DBGMCU DEV_ID. A switch to an image built for another part is
     *  refused: it would come up with another board's pin map, if at all. */
    uint32_t dev_id;

    uint32_t reserved[4];
} bb_image_header_t;

/** The running image's own header. Lives in the .image_header section. */
extern const bb_image_header_t bb_image_self;

/** What a bank holds, decoded. */
typedef struct {
    bool     present;            /**< the magic was there  */
    bool     crc_ok;             /**< recomputed and matched */
    bool     dev_ok;             /**< built for this silicon */
    bool     running;            /**< this is the image executing now */
    char     board[25];
    char     profile[17];
    char     version[17];
    char     built[25];
    uint32_t image_size;
    uint32_t image_crc;          /**< as claimed by the header */
    uint32_t computed_crc;       /**< what the bytes actually add up to */
    /** Where this bank is in the address space right now. */
    uint32_t address;
} bb_bank_info_t;

/** Bank A is physical bank 1, bank B is physical bank 2. */
typedef enum { BB_BANK_A = 0, BB_BANK_B = 1 } bb_bank_t;

/** CRC-32 as the header defines it. Exposed for the self test. */
uint32_t bb_image_crc32(const uint8_t *data, size_t len);

/** Bytes per bank on this part. */
uint32_t bb_image_bank_size(void);

/**
 * @brief Inspect one bank, recomputing its checksum.
 *
 * Costs a pass over the image - a few milliseconds for 90 KB - so it is a
 * command rather than something done on every status poll.
 */
void bb_image_bank(bb_bank_t which, bb_bank_info_t *out);

/** Which bank is executing: A unless the SWAP_BANK option byte is set. */
bb_bank_t bb_image_running_bank(void);

/** True when the SWAP_BANK option byte is programmed. */
bool bb_image_swapped(void);

/** Why a switch was refused, or NULL when it was not. */
typedef enum {
    BB_SWAP_OK = 0,
    BB_SWAP_ALREADY,             /**< that bank is already running */
    BB_SWAP_NO_IMAGE,            /**< no header there */
    BB_SWAP_BAD_CRC,
    BB_SWAP_WRONG_PART,
    BB_SWAP_OPTION_FAILED,       /**< the option byte would not program */
    /* Added for the RT1186: this board has no bankable internal flash at all,
     * which is a different answer from "the swap was refused". Kept as a
     * distinct value so a host can tell "never going to work here" from
     * "try again" - the STM32 reasons are all recoverable, this one is not. */
    BB_SWAP_UNSUPPORTED
} bb_swap_result_t;

const char *bb_swap_reason(bb_swap_result_t result);

/**
 * @brief Make @p which the bank that runs, and reset into it.
 *
 * Verifies the target bank first and refuses on anything short of a complete,
 * checksum-clean image built for this silicon. On success the option byte is
 * programmed and the board resets, so this does not return.
 */
bb_swap_result_t bb_image_activate(bb_bank_t which);

/** Copy a fixed-width header string out safely, NUL-terminating it. */
void bb_image_copy_str(char *dst, size_t dst_size, const char *src, size_t src_size);

#endif /* BB_IMAGE_H */
