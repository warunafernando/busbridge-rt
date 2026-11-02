/**
 * @file bb_ee_mdio.c
 * @brief MDIO target — declared, not yet implemented, and says so.
 *
 * The STM32 build had one (443 lines), answering as a PHY so the controller's
 * clause-22 and clause-45 code could be exercised against something.
 *
 * Not carried over yet, and there is a specific reason it is the last of the
 * four rather than merely the fourth. An MDIO target has to sample MDIO on
 * every MDC edge and drive the turnaround bit low at exactly the right bit
 * position - it is a bit-level state machine on an externally supplied clock,
 * with no peripheral to help. On the STM32 that was an EXTI on MDC. Here the
 * right answer is almost certainly FlexIO rather than an interrupt per edge,
 * which makes it a job for the same machinery the capture engine needs - so it
 * is better built after that exists than before it, twice.
 *
 * Reported absent rather than stubbed: a PHY that acknowledges reads and
 * returns nothing meaningful looks exactly like a real PHY with a fault.
 */
#include "bb_ee.h"
#include "bb_ee_ports.h"

#include <stdio.h>
#include <string.h>

void bb_ee_mdio_init(void)
{
}

void bb_ee_mdio_describe(char *out, size_t cap)
{
    (void)snprintf(out, cap, "mdio not-implemented");
}

void bb_ee_mdio_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    (void)snprintf(out->pins,   sizeof out->pins,   "-");
    (void)snprintf(out->detail, sizeof out->detail, "not-implemented");
    out->state = BB_STATE_ABSENT;
}

void bb_ee_mdio_mdc_edge(void)
{
}

unsigned bb_ee_mdio_mdc_line(void)
{
    /* The pad it would use, so EE:PORTS? can say where it will be. */
    return 17u;   /* GPIO_EMC_B1_17 = GPIO2_IO17, D8 */
}
