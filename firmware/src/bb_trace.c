/**
 * @file bb_trace.c
 * @brief The controller-side event ring behind `SNIFF:SRC self`.
 *
 * The contract, the vocabulary and the reasons are all in bb_trace.h. This is
 * the mechanism, and it is deliberately small: a ring, a counter, and one
 * decision about what to do when it fills.
 */
#include "bb_trace.h"

#include "bb_board.h"

static struct {
    bb_trace_ev_t ev[BB_TRACE_RING];
    uint16_t      head;      /* next slot to write */
    uint16_t      tail;      /* next slot to read */
    uint32_t      total;     /* pushed since the last clear */
    uint32_t      lost;      /* dropped because the ring was full */
    uint32_t      t0;        /* bb_cycles() at the last clear or arm */
    bool          armed;
    /*
     * A loss that has not yet been announced in the stream.
     *
     * Counting drops is not enough. A host reading tokens sees a sequence that
     * is *plausible and wrong* - a frame whose middle bytes are simply absent
     * reads as a shorter frame, not as a damaged one - and the count lives in
     * a different command it may never ask for. So the first push that finds
     * room after a loss spends a slot on BB_EV_DROPPED, which the host library
     * has always rendered as `!` and always meant "there is a hole here".
     *
     * The slot is spent on the hole rather than on the event, because an
     * unmarked hole costs more than one more dropped event.
     */
    bool          hole;
} s;

static inline bool ring_full(void)
{
    return (uint16_t)((s.head + 1u) % BB_TRACE_RING) == s.tail;
}

static void put(bb_ev_kind_t kind, uint8_t value, bool flag, bool derived)
{
    bb_trace_ev_t *e = &s.ev[s.head];

    e->t_cyc   = bb_cycles();
    e->kind    = (uint8_t)kind;
    e->value   = value;
    e->flag    = flag ? 1u : 0u;
    e->derived = derived ? 1u : 0u;
    s.head     = (uint16_t)((s.head + 1u) % BB_TRACE_RING);
}

void bb_trace_arm(void)
{
    s.t0    = bb_cycles();
    s.armed = true;
}

void bb_trace_disarm(void)
{
    s.armed = false;
}

bool bb_trace_armed(void)
{
    return s.armed;
}

void bb_trace_clear(void)
{
    s.head  = 0u;
    s.tail  = 0u;
    s.total = 0u;
    s.lost  = 0u;
    s.hole  = false;
    s.t0    = bb_cycles();
}

void bb_trace_push_at(bb_ev_kind_t kind, uint8_t value, bool flag,
                      bool derived, uint32_t t_cyc)
{
    if (!s.armed) {
        return;
    }
    if (ring_full()) {
        s.lost++;
        s.hole = true;
        return;
    }
    if (s.hole) {
        put(BB_EV_DROPPED, 0u, false, false);
        s.hole = false;
        if (ring_full()) {
            s.lost++;
            s.hole = true;
            return;
        }
    }
    put(kind, value, flag, derived);
    /* put() stamps with now; this event's time was taken earlier. */
    s.ev[(s.head + BB_TRACE_RING - 1u) % BB_TRACE_RING].t_cyc = t_cyc;
    s.total++;
}

void bb_trace_push(bb_ev_kind_t kind, uint8_t value, bool flag, bool derived)
{
    if (!s.armed) {
        return;
    }
    if (ring_full()) {
        s.lost++;
        s.hole = true;
        return;
    }
    if (s.hole) {
        put(BB_EV_DROPPED, 0u, false, false);
        s.hole = false;
        if (ring_full()) {
            /* The hole marker took the last slot. The event it was marking is
             * dropped too, and counted - which is correct: the marker says
             * "something is missing here" and this is one more of it. */
            s.lost++;
            s.hole = true;
            return;
        }
    }
    put(kind, value, flag, derived);
    s.total++;
}

unsigned bb_trace_pending(void)
{
    return (unsigned)((s.head + BB_TRACE_RING - s.tail) % BB_TRACE_RING);
}

unsigned bb_trace_drain(bb_trace_ev_t *out, unsigned max)
{
    unsigned n = 0u;

    while ((n < max) && (s.tail != s.head)) {
        out[n++] = s.ev[s.tail];
        s.tail   = (uint16_t)((s.tail + 1u) % BB_TRACE_RING);
    }
    return n;
}

unsigned bb_trace_peek(bb_trace_ev_t *out, unsigned max, unsigned skip)
{
    unsigned n = 0u;
    uint16_t t = s.tail;

    for (unsigned i = 0; (i < skip) && (t != s.head); i++) {
        t = (uint16_t)((t + 1u) % BB_TRACE_RING);
    }
    while ((n < max) && (t != s.head)) {
        out[n++] = s.ev[t];
        t        = (uint16_t)((t + 1u) % BB_TRACE_RING);
    }
    return n;
}

uint32_t bb_trace_total(void) { return s.total; }
uint32_t bb_trace_lost(void)  { return s.lost; }
uint32_t bb_trace_t0(void)    { return s.t0; }
