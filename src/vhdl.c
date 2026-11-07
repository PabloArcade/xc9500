/*
 * vhdl.c -- XC9500/XC9500XL VHDL output stage
 *
 * Converts a fully decoded chip_t into synthesisable behavioural VHDL
 * plus a matching UCF (pin constraints).
 *
 * Output structure
 * ----------------
 *   1.  File header + library clause
 *   2.  Entity with ports (GCK/GTS/GSR/input/output)
 *   3.  Architecture internal signals
 *       -  q_ registers (registered outputs with non-trivial OE logic)
 *       -  Buried macrocell signals
 *       -  Named intermediate signals (complex RST/PST/CLK/OE conditions)
 *   4.  Intermediate concurrent assignments (conditions -> named signals)
 *   5.  Processes, one per active registered MC
 *   6.  Output-enable concurrent assignments
 *   7.  Combinatorial output assignments
 *
 * Naming conventions
 * ------------------
 *   gck1 / gck2 / gck3          global clock inputs
 *   gts1 / gts2                 global tri-state enables
 *   gsr                         global set/reset
 *   n_p{N}                      I/O pin N
 *   q_n_p{N}                    internal registered signal for pin N (before OE)
 *   fb{F}mc{M}                  buried macrocell FB=F, MC=M
 *   fb{F}mc{M}_rst / _pst       intermediate reset/preset condition
 *   fb{F}mc{M}_clk              intermediate PT-CLK condition
 *   fb{F}mc{M}_oe               intermediate PT-OE condition
 *   fb{F}mc{M}_ce               intermediate clock-enable condition
 */

#include "decode.h"
#include "vendor_cells.h"
#include "xc9500.h"
#include "device.h"
#include "target.h"
#include "jed.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* Stamped into every emitted .vhd header. */
#ifndef XC9500TOOL_VERSION
#error "XC9500TOOL_VERSION must come from device.h"
#endif

/* MAX_PINS must exceed the largest BGA ball encoding
 * (1000 + (row-'A'+1)*100 + col, see bga_pin_name() in decode.h):
 * CS144 N13 = 1413, BG256 Y20 = 3520. Do not lower it. */
/* BG352's two-letter rows AA..AF encode as rows 27..32 (bga_pin_name()),
 * so the largest ball is AF26 = 4226. */
#define MAX_PINS  4300

/* ===========================================================================
 * 1  Low-level helpers
 * =========================================================================== */

/* Canonical lowercase identifier for a macrocell (used in intermediate names) */
static void mc_label(int fb, int mc, char *buf, int sz) {
    snprintf(buf, sz, "fb%dmc%d", fb, mc);
}

/* Port/signal name for a macrocell:
 *   pin > 0  -> "n_p{pin}"
 *   pin = 0  -> "fb{fb}mc{mc}"   (buried, lowercase) */


/* Internal registered signal:
 *   has pin   -> "q_n_p{pin}"
 *   buried    -> "fb{fb}mc{mc}"   (buried signals are used directly) */
static const chip_t *g_reclass_chip = NULL;
static void mc_qname(const device_t *dev, int fb, int mc, char *buf, int sz) {
    int pin = dev->mc_pin ? dev->mc_pin[(fb-1)*18 + mc-1] : 0;
    /* A macrocell reclassified as buried (drives no pad but carries logic)
     * uses the buried spelling, the one its q_ declaration uses. */
    if (g_reclass_chip && fb >= 1 && fb <= 16 && mc >= 1 && mc <= MACROCELLS_PER_FB
        && g_reclass_chip->mc[fb-1][mc-1].tri_logic_pin) pin = 0;
    if (!pin) { mc_label(fb, mc, buf, sz); return; }
    if (pin_is_bga(pin)) { char b[16]; bga_pin_name(pin,b,sizeof b); snprintf(buf,sz,"q_%s",b); }
    else snprintf(buf, sz, "q_n_p%d", pin);
}

/* ---------------------------------------------------------------------------
 * BGA-aware pin spelling. One pin needs two different strings:
 *
 *   pin_sig()  VHDL identifier   BGA -> "ball_B7"   numeric -> "n_p31"
 *   pin_loc()  UCF LOC string    BGA -> "B7"        numeric -> "P31"
 *
 * cpldfit rejects   NET "n_p1013" LOC = "P1013";
 * with ERROR:Cpld:832 - 'n_p1013' is assigned to an invalid location.
 * The correct line is   NET "ball_A3" LOC = "A3";
 * ------------------------------------------------------------------------ */

/* VHDL identifier for a pin. */
static void pin_sig(int pin, char *buf, int sz) {
    if (pin_is_bga(pin)) { char b[16]; bga_pin_name(pin, b, sizeof b);
                           snprintf(buf, sz, "%s", b); }
    else                   snprintf(buf, sz, "n_p%d", pin);
}

/* UCF LOC target for a pin. Derived from the ball encoding directly rather
 * than by string-slicing bga_pin_name(), so the two cannot drift apart.
 * BGA encoding: 1000 + (row-'A'+1)*100 + col, so 1103 -> row 1 ('A') col 3. */
static void pin_loc(int pin, char *buf, int sz) {
    if (pin < 0 || pin >= MAX_PINS) {   /* not a pin of any supported package */
        snprintf(buf, sz, "?");
        return;
    }
    if (pin_is_bga(pin)) {
        int c = pin - 1000, r = c / 100;   /* rows 27..32 = AA..AF */
        if (r <= 26) snprintf(buf, sz, "%c%d", (char)('A' + r - 1), c % 100);
        else         snprintf(buf, sz, "A%c%d", (char)('A' + r - 27), c % 100);
    } else {
        snprintf(buf, sz, "P%d", pin);
    }
}

/* Can this pin number exist on this package at all?
 * True if the pin appears in the device's own mc_pin[] table. A pin that
 * does not comes from another package's table (the XC9536XL FB2 decode uses
 * PC44 pin numbers, see decode.c xl9536_fb2_m_code_pin), and a LOC on it
 * would be wrong -- on CS48 also a cpldfit error, since P22 is not a
 * location on a 48-ball BGA. */
static int pin_on_package(const device_t *dev, int pin) {
    if (pin <= 0) return 0;
    if (!dev->mc_pin) return 1;          /* no table to check against */
    for (int i = 0; i < dev->n_fbs * 18; i++)
        if (dev->mc_pin[i] == pin) return 1;
    return 0;
}

/* Inverse of pin_sig().  "ball_B7" -> 1207, "n_p31" -> 31, else 0.
 * The column is the whole digit run after the row letter (CS144 up to 13,
 * BG256 up to 20). */
static int sig_to_pin(const char *s) {
    if (strncmp(s, "ball_", 5) == 0) {
        char r = s[5];
        int row = 0, d = 6;
        if (r >= 'A' && r <= 'Z' && isdigit((unsigned char)s[6])) row = r - 'A' + 1;
        else if (r == 'A' && s[6] >= 'A' && s[6] <= 'Z' && isdigit((unsigned char)s[7])) {
            row = 26 + (s[6] - 'A' + 1); d = 7;    /* AA..AF */
        }
        if (row) {
            const char *p = s + d;
            while (isdigit((unsigned char)*p)) p++;
            if (*p == '\0') {
                int col = atoi(s + d);
                if (col >= 1 && col <= 99)
                    return 1000 + row * 100 + col;
            }
        }
        return 0;
    }
    if (strncmp(s, "n_p", 3) == 0 && isdigit((unsigned char)s[3]))
        return atoi(s + 3);
    return 0;
}

/* Does this device use BGA ball numbering? */
/* Not called at present; kept as the device-level companion of
 * pin_is_bga(). Marked used so the build stays clean. */
__attribute__((unused))
static int dev_is_bga(const device_t *dev) {
    if (!dev->mc_pin) return 0;
    for (int i = 0; i < dev->n_fbs * 18; i++)
        if (pin_is_bga(dev->mc_pin[i])) return 1;
    return 0;
}

/* May this pin be emitted as a port and given a LOC on this package?
 *
 * decode.c's xl9536_fb2_m_code_pin[] holds PC44 pin numbers and is also
 * used for CS48/VQ64, so a CS48 decode can yield "n_p22". P22 is not a
 * location on a 48-ball BGA and cpldfit refuses it. Such a pin becomes a
 * visible placeholder instead. Set XC9500_WARN_OFFPKG=1 to have every
 * off-package pin reported on stderr for any device. */
static int pin_usable(const device_t *dev, int pin) {
    if (pin <= 0 || pin >= MAX_PINS) return 0;
    if (!pin_on_package(dev, pin)) {
        if (getenv("XC9500_WARN_OFFPKG"))
            fprintf(stderr, "OFFPKG %s pin %d\n", dev->id, pin);
        /* Non-BGA devices: a pin no macrocell owns in this package (e.g.
         * XC9536XL-VQ44 pin 24 = TDO, a dedicated JTAG pin; XC9536XL-VQ64
         * pin 14, absent from that package's mc_pin table) also gets the
         * offpkg_pNN placeholder: no VHDL port and no UCF LOC. */
        return 0;
    }
    return 1;
}

/* Does this MC need a separate internal q_ signal?
 * Yes when:  it is a registered output (not purely combinatorial) AND
 *            the pin is an I/O (oe_mode != 1 -> non-trivial OE logic)
 *            OR it is buried (always uses the signal directly, no q_ needed).
 * In practice: only registered pin-bearing MCs with GOE or PT-OE need q_. */
/* Does this macrocell drive its pad?
 * oe_mode==0 is tristate: the macrocell carries logic but never drives the
 * pin; the logic is read internally through the FastCONNECT. Emitting it
 * as a driven output would make the rebuild drive a pin the original leaves
 * floating. Every site that decides pad ownership uses this predicate:
 * classification, the driver, the port declaration and literal resolution. */
static int drives_pad(const mc_t *m) {
    return m->pin > 0 && m->oe_mode != 0;
}

static int mc_needs_q(const mc_t *m) {
    if (m->is_combi) return 0;
    if (!m->pin)     return 0;   /* buried: signal IS the q-value */
    return 1;                    /* all registered pin-outputs get q_ */
}

/* True if the MC is active and has actual logic */
static int mc_active(const mc_t *m) {
    /* A macrocell with a real OE condition drives its pad whatever
     * its data_pts[] holds (same rule as is_real_output in decode.c). */
    return m->n_data_pts > 0 || m->has_reset || m->has_preset || m->has_ce
           || m->has_clk_pt   /* the clock term is not in data_pts */
           || m->gsr_reset || m->gsr_set || m->is_combi
           || m->oe_mode != 0
           || m->ce_const0;   /* a held register is still a register its readers name */
}

/* True when emit_data_expr()'s OE rule (below) fires for this
 * macrocell: n==0, a real OE condition, not PT-clocked. That rule already
 * applies CPTINV's polarity when it picks the '0'/'1' literal, so a caller
 * that wraps its output in `not (...)` on m->d_inverted (emit_combinatorial())
 * must not do so again here. Example: cpldfit `y8 = Vcc;` must emit
 * `'1'`, not `not ('1')`. */
static int oe_honest_rule_fires(const mc_t *m, int n) {
    return n == 0 && m->oe_mode != 0 && !m->has_clk_pt;
}

/* True if the MC is a registered output visible on a pin */


/* BGA pins encoded as 1000+(row-'A'+1)*100+col (A1=1101, G7=1707),
 * see bga_pin_name() in decode.h */
/* -- GCK / GTS / GSR pin lookup ------------------------------------------- */

/* For VQ44 packages the global signals sit at well-known pin positions,
 * derived from mc_pin rather than hard-coded numbers:
 *
 *   XC9572XL VQ44: GCK1=FB1_MC9(P43) GCK2=FB1_MC11(P44) GCK3=FB1_MC14(P1)
 *                  GTS1=FB2_MC14(P36) GTS2=FB2_MC11(P34) GSR=FB2_MC9(P33)
 *   XC9536XL VQ44: GCK1=FB1_MC3(P43) GCK2=FB1_MC5(P44)  GCK3=FB1_MC7(P1)
 *                  GTS1=FB2_MC3(P36)  GTS2=FB2_MC5(P34)  GSR=FB2_MC6(P33)
 *
 * Rule: GCK MCs in FB1; GTS/GSR MCs in FB2 of the first pair_base=0 group. */

/* Return the physical pin number for GCKn (1/2/3), 0 if not found.
 *
 * The GCK macrocell position is per density, not derivable from the
 * function-block count alone (Xilinx datasheet "I/O Pins" tables):
 *     2 FB  XC9536/XL : FB1_MC3   FB1_MC5   FB1_MC7
 *     4 FB  XC9572/XL : FB1_MC9   FB1_MC11  FB1_MC14
 *     6 FB  XC95108   : FB1_MC12  FB1_MC14  FB1_MC16   (PC84 P9/P10/P12)
 *     8 FB  XC95144   : FB1_MC17  FB3_MC2   FB3_MC8    (TQ100 P22/P23/P27)
 *     8 FB  XC95144XL : FB1_MC17  FB3_MC2   FB3_MC8    (TQ100 P22/P23/P27)
 * XC95144 is the outlier of the family: GCK2 and GCK3 live in FB3, not FB1.
 *
 * Returns 0 for a part not covered, so the port is not emitted: a wrong
 * LOC is worse than a missing one. */

/* ---------------------------------------------------------------------
 * Global-pin positions from Xilinx's own <device>.chp pad tables, which
 * name GCK1/2/3, GTS1/2 and GSR per pad and per package. The rules in
 * the functions below hold for the 2-FB and 4-FB parts only:
 *
 *   GTS: XC95108 and XC95144 use FB2_MC8/MC9, XC95216 FB2_MC2/MC5,
 *        XC95288(XL) FB4_MC8/MC12.
 *   GSR: XC95108 FB2_MC5, XC95144 FB2_MC2, XC95216 FB4_MC11,
 *        XC95288(XL) FB6_MC15.
 *   GTS1 is package-dependent on XC9572XL: TQ100 at FB2_MC7,
 *        VQ44/PC44/CS48/VQ64 at FB2_MC14.
 *   XC95144 PQ100: GSR is FB2_MC2 = pin 1.
 *
 * Exact device-id match only. Anything not listed -- generic family rows
 * included -- falls through to the rules. */
/* From Xilinx's <device>.chp pad tables (ISE 14.7 xc9500/data and
 * xc9500xl/data). Order: GCK1 GCK2 GCK3 GTS1 GTS2 GSR GTS3 GTS4,
 * each as {fb, mc}. */
/* Slots 6,7 are GTS3 and GTS4, which exist on XC95144(XL), XC95216 and
 * XC95288(XL) only. XC95144XL / XC95288XL have them at FB2_MC5/MC6 and
 * FB4_MC2/MC5, as the non-XL parts. */
static const struct { const char *id; signed char g[8][2]; } chp_globals[] = {
        /* Generic rows, for a JED that never names its package (device.c
     * has an "XCxxxx-" entry for each). Without them gts_pin_num() falls back
     * to its rule, which picks FB2_MC14; on XC95144 the real GTS1 pad is
     * FB2_MC8 (P3 on TQ100). XC9572 and XC9572XL are omitted: their GTS1 is
     * package-dependent, so a generic value would be a guess. */
    { "XC95108-",         {{1,12},{1,14},{1,16},{2, 8},{2, 9},{2, 5},{0, 0},{0, 0}} },
    { "XC95144-",         {{1,17},{3, 2},{3, 8},{2, 8},{2, 9},{2, 2},{2, 5},{2, 6}} },
    { "XC95144XL-",       {{1,17},{3, 2},{3, 8},{2, 8},{2, 9},{2, 2},{2, 5},{2, 6}} },
    { "XC95216-",         {{3, 3},{3, 6},{3,14},{2, 2},{2, 5},{4,11},{4,12},{4,15}} },
    { "XC95288-",         {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC95288XL-",       {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC9536-",          {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
    { "XC9536XL-",        {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
{ "XC95108-PC84", {{1,12},{1,14},{1,16},{2, 8},{2, 9},{2, 5},{0, 0},{0, 0}} },
    { "XC95108-PQ100", {{1,12},{1,14},{1,16},{2, 8},{2, 9},{2, 5},{0, 0},{0, 0}} },
    { "XC95108-PQ160", {{1,12},{1,14},{1,16},{2, 8},{2, 9},{2, 5},{0, 0},{0, 0}} },
    { "XC95108-TQ100", {{1,12},{1,14},{1,16},{2, 8},{2, 9},{2, 5},{0, 0},{0, 0}} },
    { "XC95144-PQ100", {{1,17},{3, 2},{3, 8},{2, 8},{2, 9},{2, 2},{2, 5},{2, 6}} },
    { "XC95144-PQ160", {{1,17},{3, 2},{3, 8},{2, 8},{2, 9},{2, 2},{2, 5},{2, 6}} },
    { "XC95144-TQ100", {{1,17},{3, 2},{3, 8},{2, 8},{2, 9},{2, 2},{2, 5},{2, 6}} },
    { "XC95144XL-CS144", {{1,17},{3, 2},{3, 8},{2, 8},{2, 9},{2, 2},{2, 5},{2, 6}} },
    { "XC95144XL-TQ100", {{1,17},{3, 2},{3, 8},{2, 8},{2, 9},{2, 2},{2, 5},{2, 6}} },
    { "XC95144XL-TQ144", {{1,17},{3, 2},{3, 8},{2, 8},{2, 9},{2, 2},{2, 5},{2, 6}} },
    { "XC95216-BG352", {{3, 3},{3, 6},{3,14},{2, 2},{2, 5},{4,11},{4,12},{4,15}} },
    { "XC95216-HQ208", {{3, 3},{3, 6},{3,14},{2, 2},{2, 5},{4,11},{4,12},{4,15}} },
    { "XC95216-PQ160", {{3, 3},{3, 6},{3,14},{2, 2},{2, 5},{4,11},{4,12},{4,15}} },
    { "XC95288-BG352", {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC95288-HQ208", {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC95288XL-BG256", {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC95288XL-CS280", {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC95288XL-FG256", {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC95288XL-PQ208", {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC95288XL-TQ144", {{3,10},{3,14},{5, 8},{4, 8},{4,12},{6,15},{4, 2},{4, 5}} },
    { "XC9536-CS48", {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
    { "XC9536-PC44", {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
    { "XC9536-VQ44", {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
    { "XC9536XL-CS48", {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
    { "XC9536XL-PC44", {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
    { "XC9536XL-VQ44", {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
    { "XC9536XL-VQ64", {{1, 3},{1, 5},{1, 7},{2, 3},{2, 5},{2, 6},{0, 0},{0, 0}} },
    { "XC9572-PC44", {{1, 9},{1,11},{1,14},{2,14},{2,11},{2, 9},{0, 0},{0, 0}} },
    { "XC9572-PC84", {{1, 9},{1,11},{1,14},{2, 7},{2,11},{2, 9},{0, 0},{0, 0}} },
    { "XC9572-PQ100", {{1, 9},{1,11},{1,14},{2, 7},{2,11},{2, 9},{0, 0},{0, 0}} },
    { "XC9572-TQ100", {{1, 9},{1,11},{1,14},{2, 7},{2,11},{2, 9},{0, 0},{0, 0}} },
    { "XC9572XL-CS48", {{1, 9},{1,11},{1,14},{2,14},{2,11},{2, 9},{0, 0},{0, 0}} },
    { "XC9572XL-PC44", {{1, 9},{1,11},{1,14},{2,14},{2,11},{2, 9},{0, 0},{0, 0}} },
    { "XC9572XL-TQ100", {{1, 9},{1,11},{1,14},{2, 7},{2,11},{2, 9},{0, 0},{0, 0}} },
    { "XC9572XL-VQ44", {{1, 9},{1,11},{1,14},{2,14},{2,11},{2, 9},{0, 0},{0, 0}} },
    { "XC9572XL-VQ64", {{1, 9},{1,11},{1,14},{2,14},{2,11},{2, 9},{0, 0},{0, 0}} },
};
#define N_CHP_GLOBALS 36

/* which: 0=GCK1 1=GCK2 2=GCK3 3=GTS1 4=GTS2 5=GSR. Returns a pin, or 0. */
static int chp_global_pin(const device_t *dev, int which) {
    if (!dev || !dev->id || !dev->mc_pin || which < 0 || which > 7) return 0;
    for (int i = 0; i < N_CHP_GLOBALS; i++) {
        if (strcmp(dev->id, chp_globals[i].id) != 0) continue;
        int fb = chp_globals[i].g[which][0], mc = chp_globals[i].g[which][1];
        if (fb < 1 || fb > dev->n_fbs || mc < 1 || mc > 18) return 0;
        return dev->mc_pin[(fb - 1) * 18 + mc - 1];
    }
    return 0;
}

static int gck_pin_num(const device_t *dev, int gck) {
    if (!dev->mc_pin || gck < 1 || gck > 3) return 0;
    { int p = chp_global_pin(dev, gck - 1); if (p) return p; }
    /* one named type: separately written anonymous structs are distinct types in C */
    typedef struct { int fb, mc; } gpos_t;
    static const gpos_t g2[3] = { {1,3},  {1,5},  {1,7}  };
    static const gpos_t g4[3] = { {1,9},  {1,11}, {1,14} };
    static const gpos_t g6[3] = { {1,12}, {1,14}, {1,16} };
    static const gpos_t g8[3] = { {1,17}, {3,2},  {3,8}  };
    const gpos_t *g;
    if      (dev->n_fbs == 2) g = g2;
    else if (dev->n_fbs == 4) g = g4;
    else if (dev->n_fbs == 6) g = g6;
    else if (dev->n_fbs == 8) g = g8;   /* XC95144 and XC95144XL alike */
    /* XC95288XL is covered by chp_globals (chp_global_pin() runs first):
     * GCK FB3_10/FB3_14/FB5_8, GTS FB4_2/FB4_5/FB4_8/FB4_12, GSR FB6_15.
     * Reaching here means a 10/12/16-FB part with no chp_globals row. */
    else return 0;
    return dev->mc_pin[(g[gck-1].fb - 1) * 18 + g[gck-1].mc - 1];
}

/* Return the physical pin number for GTSn (1/2), 0 if not found.
 *
 * GTS1 lives at FB2_MC7, but in VQ44 that MC is buried so it remaps to
 * FB2_MC14.
 * Detection: if FB2_MC7 has a pin -> GTS1 at MC7 (TQ100, full package)
 *            if FB2_MC7 is buried -> GTS1 at MC14 (VQ44, reduced package)
 * GTS2 is always at FB2_MC11.
 * For XC9536XL (n_fbs=2): GTS1=FB2_MC3, GTS2=FB2_MC5.
 */
static int gts_pin_num(const device_t *dev, int gts) {
    if (!dev->mc_pin) return 0;
    /* slots are GCK1,2,3 = 0,1,2 ; GTS1 = 3 ; GTS2 = 4 ; GSR = 5 ;
     * GTS3 = 6 ; GTS4 = 7. */
    {   int slot = (gts == 1) ? 3 : (gts == 2) ? 4 : (gts == 3) ? 6 : 7;
        int p = chp_global_pin(dev, slot);
        if (p) return p;
        if (gts >= 3) return 0;   /* no GTS3/GTS4 on this device */
    }
    int fb = 2;
    if (dev->n_fbs < 4) {
        /* XC9536XL: GTS1=MC3, GTS2=MC5 */
        int mc_small[2] = {3, 5};
        return dev->mc_pin[(fb-1)*18 + mc_small[gts-1] - 1];
    }
    /* XC9572XL: detect from package whether MC7 is available */
    int mc7_pin = dev->mc_pin[(fb-1)*18 + 7 - 1];
    if (gts == 1) {
        /* GTS1: MC7 if active, else MC14 */
        int mc = (mc7_pin != 0) ? 7 : 14;
        return dev->mc_pin[(fb-1)*18 + mc - 1];
    } else {
        /* GTS2: always MC11 */
        return dev->mc_pin[(fb-1)*18 + 11 - 1];
    }
}

/* Return the physical pin number for GSR, 0 if not found. */
static int gsr_pin_num(const device_t *dev) {
    if (!dev->mc_pin) return 0;
    { int p = chp_global_pin(dev, 5); if (p) return p; }
    int fb = 2;
    /* GSR -> FB2_MC9 (large) / FB2_MC6 (small) */
    int mc = (dev->n_fbs >= 4) ? 9 : 6;
    return dev->mc_pin[(fb-1)*18 + mc - 1];
}

/* -- VHDL-specific active test: stricter than mc_active() ---------------- */
/* Skips tristate pin-bearing MCs (pure inputs or IMUX A-byte contamination) */
/* Is this macrocell read by any other macrocell's logic?
 *
 * A macrocell with zero product terms is not "active" -- a T flip-flop with
 * no toggle term simply holds its init value -- but other logic can still
 * read it. If it were skipped, every literal naming it would become a stub
 * tied to '0', and every product term reading it would be constant false. */
/* Split a wired composite "A|B|C" into its components.
 * Returns the number of components written (>=1); 1 means the name was not a
 * composite at all, and parts[0] is a copy of it. Returns 0 if a component
 * does not fit, so every caller can treat 0 as "do not use this name".
 * The caller must have stripped any "NOT_" prefix first -- the polarity
 * belongs to the whole composite, not to its members. */
#define OR_MAX_PARTS 112
#define OR_PART_LEN  32
static int or_parts(const char *base, char parts[][OR_PART_LEN], int maxp) {
    int n = 0;
    const char *p = base;
    while (n < maxp) {
        const char *bar = strchr(p, '|');
        size_t len = bar ? (size_t)(bar - p) : strlen(p);
        if (len == 0 || len >= OR_PART_LEN) return 0;
        memcpy(parts[n], p, len);
        parts[n][len] = 0;
        n++;
        if (!bar) return n;
        p = bar + 1;
    }
    return 0;                                   /* more parts than we can hold */
}

/* or_parts() keeps a member's '!' (FBINV) marker for or_expand_expr();
 * every caller that wants the member's NAME uses this instead. */
static int or_parts_names(const char *base, char parts[][OR_PART_LEN], int maxp) {
    int n = or_parts(base, parts, maxp), i;
    for (i = 0; i < n; i++)
        if (parts[i][0] == '!') memmove(parts[i], parts[i] + 1, strlen(parts[i]));
    return n;
}

/* Override the resolved name when the fuses take the identity path
 * to a macrocell reclassified as buried. Returns 1 if buf was rewritten. */
static int identity_override_c(const chip_t *chip, const device_t *dev,
                               int fb, int pair, int is_comp, char *buf, int sz) {
    int gf = 0, gm = 0;
    if (!pair_via_identity(dev, chip->fuses, fb, pair, &gf, &gm)) return 0;
    if (gf < 1 || gf > dev->n_fbs || gm < 1 || gm > MACROCELLS_PER_FB) return 0;
    if (!chip->mc[gf-1][gm-1].tri_logic_pin) return 0;
    /* this path builds its own name, so decode.c's FBINV member rule
     * (pair_signal_name) is applied here too. */
    if (!dev->is_xl && device_col(dev, chip->fuses, gf, 46)[mc_off(gm)] == 0
        && mc_is_wired_and_member(dev, chip->fuses, gf, gm))
        is_comp = !is_comp;
    snprintf(buf, sz, "%sn_FB%dMC%d", is_comp ? "NOT_" : "", gf, gm);
    return 1;
}

static int mc_is_referenced(const chip_t *chip, const device_t *dev,
                            int fb_t, int mc_t_) {
    char want[32];
    int fb, mc, p, l;
    snprintf(want, sizeof want, "n_FB%dMC%d", fb_t, mc_t_);
    for (fb = 1; fb <= dev->n_fbs; fb++) {
        for (mc = 1; mc <= MACROCELLS_PER_FB; mc++) {
            const mc_t *o = &chip->mc[fb-1][mc-1];
            const pt_t *lists[7];
            int nl = 0, np[7], k;
            if (fb == fb_t && mc == mc_t_) continue;
            if (!mc_active(o)) continue;
            lists[nl] = o->data_pts;  np[nl++] = o->n_data_pts;
            lists[nl] = &o->rst_pt;   np[nl++] = o->has_reset  ? 1 : 0;
            lists[nl] = &o->pst_pt;   np[nl++] = o->has_preset ? 1 : 0;
            lists[nl] = &o->ce_pt;    np[nl++] = o->has_ce     ? 1 : 0;
            lists[nl] = &o->oe_pt;    np[nl++] = o->has_oe_pt  ? 1 : 0;
            lists[nl] = &o->clk_pt;   np[nl++] = o->has_clk_pt ? 1 : 0;
            lists[nl] = &o->ptinv_pt; np[nl++] = o->has_ptinv  ? 1 : 0;
            for (k = 0; k < nl; k++) {
                for (p = 0; p < np[k]; p++) {
                    const pt_t *pt = &lists[k][p];
                    for (l = 0; l < pt->n_lits; l++) {
                        char nm[SIGNAME_MAX];
                        pair_signal_name(dev, chip->fuses, fb, pt->lits[l].pair,
                                         0, nm, sizeof nm);
                        identity_override_c(chip, dev, fb, pt->lits[l].pair, 0, nm, sizeof nm);
                        if (strcmp(nm, want) == 0) return 1;
                        /* a macrocell can be read as one of several
                         * sources wired onto a pair; without this it would
                         * look unreferenced and be dropped. */
                        if (strchr(nm, '|') != NULL) {
                            char parts[OR_MAX_PARTS][OR_PART_LEN];
                            int np = or_parts(nm, parts, OR_MAX_PARTS), pi;
                            for (pi = 0; pi < np; pi++)
                                if (strcmp(parts[pi] + (parts[pi][0] == '!'), want) == 0) return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

static int mc_vhdl_active(const mc_t *m) {
    /* A pin-bearing macrocell with oe_mode==0 is just an input when it
     * carries no logic. When it does carry logic it is a buried node whose
     * pad happens to exist: keep it, or every literal naming it becomes a
     * constant '0'. Still skip a truly empty pad. */
    if (m->pin > 0 && m->oe_mode == 0) {
        if (m->n_data_pts > 0 || m->has_clk_pt || m->has_reset
            || m->has_preset || m->has_ce || m->ce_const0) return mc_active(m);
        return 0;
    }
    return mc_active(m);
}

/* ===========================================================================
 * 2  Context: collect all ports and signals needed
 * =========================================================================== */

/* Must exceed the BGA pin encoding (1000 + (row-'A'+1)*100 + col):
 * out_mc[], has_q[] and input_pin[] are indexed by pin number. */

typedef struct {
    const device_t *dev;
    const chip_t   *chip;

    /* Which GCK clocks are actually used (bit mask) */
    int gck_used;   /* bit 0=GCK1, 1=GCK2, 2=GCK3 */

    /* A GCK pad is a dual-function I/O. If the design also drives that
     * pin as an output, the pin cannot be declared twice (once as `gckN : in`
     * and once as `n_pN : out`) -- cpldfit rejects it with
     *   ERROR:Cpld:8 Cannot assign macrocell gck1 to Pin 43 (FB1_3).
     *                Location is already occupied by n_p43.
     * In silicon the global clock net is fed from that pad, so the clock is
     * that output's value. gckN is aliased to the driving macrocell's internal
     * signal and neither the port nor the LOC is emitted. */
    char gck_alias[4][24];
    /* Pin P is a GCK pad that the design also drives, and whose clock
     * is taken from the port. If the port would otherwise be mode `out`, it
     * becomes `buffer` so VHDL-93 permits reading it. */
    int gck_pad_readback[MAX_PINS];
    int gts_used;   /* bit 0=GTS1, 1=GTS2 */
    int gsr_used;
    /* The pin driving each global net. A non-XL part can feed a clock or OE net
     * from its neighbour's pin (FB1 MCLK3..5 / MOE N..) -- cpldfit does so when one pin drives two nets (both polarities). gck_used
     * and gts_used are pin masks (the ports the design really has); a macrocell's net is mapped through these. gts_inv[n]: non-XL
     * FB1 OE<n-1>I set -- the net carries the complement of its pin, so its outputs are enabled on '0'. Index 1-based; 0 unused. */
    int gck_src[4];
    int gts_src[5];
    int gts_inv[5];

    /* Input pins appearing in literals (excluding GCK/GTS/GSR) */
    int input_pin[MAX_PINS];   /* 1 = used as input */

    /* Registered output MCs, indexed by pin */
    const mc_t *out_mc[MAX_PINS];

    /* Does this output need an internal q_ signal?  (yes for non-always-on) */
    int has_q[MAX_PINS];

    /* For literals: is pin P a registered output? (-> use q_n_p{P}) */
    int reg_out_pin[MAX_PINS];

    /* For literals: is pin P a combinatorial output?
     * VHDL-93 forbids reading a port of mode `out`, and a combinatorial
     * macrocell output is often also a FastCONNECT source (sometimes its own).
     * So each such pin gets an internal signal c_n_p{P}; the port is driven
     * from it and every read uses it (else XST ERROR:HDLParsers:1411). */
    int comb_out_pin[MAX_PINS];
    int od_pin[MAX_PINS];      /* open-drain (combi + PT-OE) */

    /* Buried active MCs */
    /* sized for 16 FBs (XC95288/XC95288XL, 288 MCs) */
    const mc_t *buried[MAX_FBS*MACROCELLS_PER_FB];
    int n_buried;

    /* All active MCs in FB/MC order */
    const mc_t *active[MAX_FBS*MACROCELLS_PER_FB];
    int n_active;

    /* Unresolved-literal ports.
     * An unresolved FastCONNECT pair emitted as an undriven
     *     signal fbN_pairM : std_logic := '0';
     * is reduced to a constant by XST, and cpldfit is then free to
     * re-synthesise the whole macrocell around it. A real input port keeps
     * the value opaque to the optimiser.
     *
     * Costs one device pin per distinct unresolved literal, so it is applied
     * only while free pins remain; beyond that the stub is used, because
     * failing to fit is worse than losing structure. */
    char unres[16][64];
    int  unres_pin[16];
    int  n_unres;

} vctx_t;

/* A device pin that no macrocell in this design occupies, for use as an
 * unresolved-literal port. Skips the global pads, which must never be
 * repurposed, and every pin owned by a macrocell. Returns 0 when nothing
 * is free; the caller then falls back to the undriven stub. */
static int pin_usable(const device_t *dev, int pin);

/* -noportfix: when set, unresolved literals stay as undriven stub signals. */
int vhdl_no_port_fix = 0;

static int next_free_pin(const vctx_t *v) {
    int fb, mc, p, i;
    unsigned char taken[MAX_PINS];
    memset(taken, 0, sizeof taken);
    /* Only pins this design uses are taken, not every pin a macrocell could
     * own (on XC95108-PC84 the mc_pin table covers every usable pin). */
    for (p = 1; p < MAX_PINS; p++)
        if (v->out_mc[p] || v->input_pin[p]) taken[p] = 1;
    /* every global pad is taken, in use or not. */
    {   int gp, gi;
        for (gi = 1; gi <= 3; gi++) { gp = gck_pin_num(v->dev, gi); if (gp > 0 && gp < MAX_PINS) taken[gp] = 1; }
        for (gi = 1; gi <= 4; gi++) { gp = gts_pin_num(v->dev, gi); if (gp > 0 && gp < MAX_PINS) taken[gp] = 1; }
        gp = gsr_pin_num(v->dev); if (gp > 0 && gp < MAX_PINS) taken[gp] = 1;
    }
    (void)fb; (void)mc;
    for (i = 0; i < v->n_unres; i++)
        if (v->unres_pin[i] > 0 && v->unres_pin[i] < MAX_PINS)
            taken[v->unres_pin[i]] = 1;
    for (p = 1; p < MAX_PINS; p++) {
        if (taken[p]) continue;
        if (!pin_usable(v->dev, p)) continue;
        return p;
    }
    return 0;
}

/* Scan all literal PTs of all active MCs to mark input pins */
/* Does this pad belong to a macrocell reclassified as buried?
 * Such a macrocell records `tri_logic_pin` and has ->pin zeroed, so its logic
 * is emitted as `fbNmcM`. Its pad is not a free input, and a literal spelled
 * `n_p<pin>` must bind to the buried signal -- otherwise one physical node
 * has two names and two drivers.
 *
 * The macrocell must be in the active list (the set the emitter writes out);
 * matching against the whole chip array would claim pads of macrocells that
 * are never emitted. */
static const mc_t *reclassified_pad(const vctx_t *v, int pin) {
    if (pin <= 0) return NULL;
    for (int i = 0; i < v->n_active; i++) {
        const mc_t *m = v->active[i];
        if (m->tri_logic_pin != pin) continue;
        /* A register whose OE is permanently off (oe_mode == 0) does not
         * own its pad. decode.c names a literal `n_p<pin>` only when it reads
         * the pad (XL: IMUX code selects the pad; non-XL: FastCONNECT
         * candidate list), and `n_FB<f>MC<m>` when it reads the macrocell
         * output (cpldfit's `sig.PIN` vs `sig`). So a pad literal stays the
         * pad: an input port, LOC'd, read as such; the register stays buried.
         * This applies to both families. */
        if (v->dev && m->oe_mode == 0) return NULL;
        return m;
    }
    return NULL;
}

static void scan_inputs(vctx_t *v) {
    const chip_t *chip = v->chip;
    const device_t *dev = v->dev;

    for (int i = 0; i < v->n_active; i++) {
        const mc_t *m = v->active[i];
        const pt_t *all[MAX_PTS + 4];
        int np = 0;
        for (int p = 0; p < m->n_data_pts; p++) all[np++] = &m->data_pts[p];
        if (m->has_reset)  all[np++] = &m->rst_pt;
        if (m->has_preset) all[np++] = &m->pst_pt;
        if (m->has_ce)     all[np++] = &m->ce_pt;
        if (m->has_oe_pt)  all[np++] = &m->oe_pt;
        if (m->has_clk_pt) all[np++] = &m->clk_pt;
        if (m->has_ptinv)  all[np++] = &m->ptinv_pt;

        for (int p = 0; p < np; p++) {
            const pt_t *pt = all[p];
            for (int l = 0; l < pt->n_lits; l++) {
                char buf[SIGNAME_MAX];
                pair_signal_name(dev, chip->fuses, m->fb, pt->lits[l].pair,
                                 pt->lits[l].is_comp, buf, sizeof buf);
                identity_override_c(chip, dev, m->fb, pt->lits[l].pair, pt->lits[l].is_comp, buf, sizeof buf);
                /* Strip NOT_ prefix to get base signal name */
                const char *whole = (strncmp(buf,"NOT_",4)==0) ? buf+4 : buf;
                /* A wired pair names several sources. Each is a separate
                 * reference and is classified on its own (sig_to_pin() on the
                 * composite returns 0). One component is the degenerate case,
                 * so both go through the same loop. */
                char oparts[OR_MAX_PARTS][OR_PART_LEN];
                int onp = or_parts_names(whole, oparts, OR_MAX_PARTS), opi;
                for (opi = 0; opi < (onp ? onp : 1); opi++) {
                /* onp == 0 means the name would not split -- a component too
                 * long, or too many of them. Use it whole rather than copying
                 * it into a 32-byte slot, where a truncated name could match
                 * some other pin. */
                const char *base = onp ? oparts[opi] : whole;
                /* Track external input pins. BGA parts name their pins
                 * ball_XN, not n_pNN, so both spellings are matched. */
                int pin = sig_to_pin(base);
                if (pin > 0 && pin < MAX_PINS && !v->out_mc[pin]
                    && pin_usable(dev, pin)
                    && !reclassified_pad(v, pin))
                    v->input_pin[pin] = 1;

                /* Allocate a real port for an unresolved literal. This is
                 * the only non-const pass over the literals, so the table is
                 * built here and only consulted elsewhere. */
                if (pin <= 0 && strstr(base, "_PAIR") != NULL) {
                    char sn[64];
                    int si = 0, ci, dup = 0;
                    for (ci = 0; base[ci] && si < 62; ci++) {
                        char c = (char)tolower((unsigned char)base[ci]);
                        sn[si++] = isalnum((unsigned char)c) ? c : '_';
                    }
                    sn[si] = 0;
                    for (ci = 0; ci < v->n_unres; ci++)
                        if (!strcmp(v->unres[ci], sn)) { dup = 1; break; }
                    if (!dup && v->n_unres < 16) {
                        int up = vhdl_no_port_fix ? 0 : next_free_pin(v);
                        if (up > 0) {
                            snprintf(v->unres[v->n_unres], 64, "%s", sn);
                            v->unres_pin[v->n_unres] = up;
                            v->n_unres++;
                        }
                    }
                }
                }
            }
        }
    }
    /* GCK1/GCK2 are handled below. GCK3, GTS1, GTS2 and GSR can also be
     * read through the FastCONNECT as ordinary data. They are stripped from
     * the input list only when this design also uses that pin as its global
     * function (some MC is clocked by GCK3, uses a GTS enable, or uses GSR);
     * the signal then already has a port under the gckN/gtsN/gsr name. When
     * the global function is not used, the pin is ordinary data. */
    /* GCK1/GCK2 likewise are stripped only when the design uses them as a
     * clock. Otherwise the pad is ordinary data: e.g. on XC95108-PC84 with
     * only GCK1 in use, GCK2's pad (P10 = FB1_MC14) is a legitimate source
     * of FastCONNECT pair 29, and cpldfit fits `o1.D = g2 & a` with g2 on P10. */
    for (int g = 1; g <= 2; g++)
        if (v->gck_used & (1 << (g-1))) { int p = gck_pin_num(dev,g); if (p) v->input_pin[p]=0; }
    if (v->gck_used & 4) { int p = gck_pin_num(dev,3); if (p) v->input_pin[p]=0; }
    /* GTS3/GTS4 too: a GOE3/GOE4 design declares gts3/gts4 as ports, and
     * they need their LOC, pad surrender and naming like GTS1/GTS2. */
    for (int g = 1; g <= 4; g++) {
        if (v->gts_used & (1 << (g-1))) { int p = gts_pin_num(dev,g); if (p) v->input_pin[p]=0; }
    }
    /* gsr_used is driven by the decoded per-macrocell GSR enables. */
    for (int i = 0; i < v->n_active; i++)
        if (v->active[i]->gsr_reset || v->active[i]->gsr_set) { v->gsr_used = 1; break; }
    if (v->gsr_used) { int p = gsr_pin_num(dev); if (p) v->input_pin[p]=0; }
    /* A global pad the design uses as that global cannot also carry a
     * macrocell output -- the UCF would LOC two nets to one pin and cpldfit
     * refuses ("Cannot assign macrocell gsr to Pin 74"). The macrocell keeps
     * its logic and is reclassified as buried (tri_logic_pin, which re-points
     * every literal); only the pad is given to the global, as in silicon. */
    {   int gp[8], ng = 0;
        if (v->gsr_used) gp[ng++] = gsr_pin_num(dev);
        for (int g = 1; g <= 4; g++)   /* GTS3/GTS4 too */
            if (v->gts_used & (1 << (g-1))) gp[ng++] = gts_pin_num(dev, g);
        for (int i = 0; i < ng; i++) {
            int p = gp[i];
            if (p <= 0 || p >= MAX_PINS || !v->out_mc[p]) continue;
            mc_t *gm = (mc_t *)v->out_mc[p];
            gm->tri_logic_pin = p;      /* literals bind to fbNmcM */
            gm->pin = 0;
            v->out_mc[p] = NULL;
            v->has_q[p] = 0;
            v->comb_out_pin[p] = 0;
            v->reg_out_pin[p] = 0;
            /* Also put it in the buried list. build_ctx() pass 1
             * classified this macrocell once, as a pad driver (out_mc[]);
             * scan_inputs() runs after that and has just taken its pad, so
             * unless it is added here it is in neither list: the declaration
             * pass (which walks v->buried[]) would not declare `fbNmcM`
             * while the process emitter (which walks v->active[]) still
             * writes `fbNmcM <= ...`, and the file would not compile.
             *
             * The predicate is pass 1's own, verbatim -- the two must match.
             * Combinatorial macrocells need no entry: buried_emitted()
             * recognises them through its second loop, whose test
             * `m->is_combi && !drives_pad(m)` holds once `gm->pin` is
             * cleared above. */
            if (!gm->is_combi && v->n_buried < (int)(sizeof v->buried / sizeof v->buried[0]))
                v->buried[v->n_buried++] = gm;
        }
    }
    /* Remove registered output pins too */
    for (int pin = 0; pin < MAX_PINS; pin++)
        if (v->out_mc[pin]) v->input_pin[pin] = 0;
}

static void build_ctx(vctx_t *v, const chip_t *chip, const char *jed_basename) {
    const device_t *dev = chip->dev;
    memset(v, 0, sizeof *v);
    v->dev  = dev;
    v->chip = chip;
    {   /* each global net's source pin (identity on XL and wherever the default cell is the set one) */
        int n_gts = 0, g;
        for (g = 1; g <= 4; g++) if (gts_pin_num(dev, g) > 0) n_gts++;
        unsigned long nf = (unsigned long)dev->n_fbs * (unsigned long)dev->jed_fuses_per_fb;
        for (g = 1; g <= 3; g++) v->gck_src[g] = vendor_cells_global_src(dev, chip->fuses, nf, 0, g, n_gts, NULL);
        for (g = 1; g <= 4; g++) v->gts_src[g] = g <= n_gts ? vendor_cells_global_src(dev, chip->fuses, nf, 1, g, n_gts, &v->gts_inv[g]) : g;
    }

    /* Pass 1: collect active MCs */
    for (int fb = 1; fb <= dev->n_fbs; fb++) {
        for (int mc = 1; mc <= MACROCELLS_PER_FB; mc++) {
            const mc_t *m = &chip->mc[fb-1][mc-1];
            if (!mc_vhdl_active(m) && !mc_is_referenced(chip, dev, fb, mc)) continue;
            v->active[v->n_active++] = m;

            /* Track GCK usage */
            /* the masks are of the pins the nets come from */
            if (m->clk_mode == CLK_MODE_GCK1) v->gck_used |= 1 << (v->gck_src[1] - 1);
            if (m->clk_mode == CLK_MODE_GCK2) v->gck_used |= 1 << (v->gck_src[2] - 1);
            if (m->clk_mode == CLK_MODE_GCK3) v->gck_used |= 1 << (v->gck_src[3] - 1);

            /* Track GTS usage */
            if (m->oe_mode == 2) v->gts_used |= 1 << (v->gts_src[1] - 1);   /* GOE1 -> GTS1 */
            if (m->oe_mode == 3 && !m->has_oe_pt)
                                  v->gts_used |= 1 << (v->gts_src[2] - 1);   /* GOE2 -> GTS2 */
            /* XC95144/216/288 have four global enables. */
            if (m->oe_mode == 4) v->gts_used |= 1 << (v->gts_src[3] - 1);   /* GOE3 -> GTS3 */
            if (m->oe_mode == 5) v->gts_used |= 1 << (v->gts_src[4] - 1);   /* GOE4 -> GTS4 */

            /* Classify MC */
            if (drives_pad(m)) {
                /* Registered or combinatorial output. The predicate must
                 * match emit_combinatorial(), which drives every active
                 * combinatorial macrocell that has a pin; otherwise a
                 * tristated combinatorial output gets a `c_n_pNN <= ...`
                 * driver with no declaration. */
                v->out_mc[m->pin] = m;
                v->has_q[m->pin]  = mc_needs_q(m);
                v->reg_out_pin[m->pin]  = !m->is_combi;
                v->comb_out_pin[m->pin] =  m->is_combi;
                v->od_pin[m->pin] = (m->is_combi && m->has_oe_pt && m->oe_mode == 3);
            } else if (!m->pin && !m->is_combi) {
                /* Buried registered */
                v->buried[v->n_buried++] = m;
            }
        }
    }

    /* Pass 1b: resolve GCK pads that the design also drives as outputs.
     * See the gck_alias comment in vctx_t. */
    /* pre-pass: can every claimed GCK pad be taken from its port? */
    int gck_all_convertible = 1;
    for (int g = 1; g <= 3; g++) {
        if (!(v->gck_used & (1 << (g-1)))) continue;
        int gp = gck_pin_num(dev, g);
        if (gp <= 0 || gp >= MAX_PINS || !v->out_mc[gp]) continue;
        if (!v->has_q[gp]) gck_all_convertible = 0;
    }
    for (int g = 1; g <= 3; g++) {
        v->gck_alias[g][0] = '\0';
        if (!(v->gck_used & (1 << (g-1)))) continue;
        int gp = gck_pin_num(dev, g);
        if (gp <= 0 || gp >= MAX_PINS || !v->out_mc[gp]) continue;
        char pnm[16];
        pin_sig(gp, pnm, sizeof pnm);
        /* Take the clock from the PORT, not from the driving macrocell's
         * internal signal (q_n_p16 / c_n_p16). The internal signal has the
         * right value but the wrong structure: ISE sees a clock produced by
         * internal logic, calls it gated, and refuses the global buffer --
         *
         *   WARNING:Cpld:1239 - The global clock designation (BUFG) on
         *      signal 'n_p15_IBUF' is ignored. Most likely the signal is
         *      gated and therefore cannot be used as a global control signal.
         *
         * and every register falls back to a product-term clock. Read from
         * the port, cpldfit reports "Signal 'n_p16' mapped onto global clock
         * net GCK2." This also matches the silicon: the global clock net is
         * fed from the pad, so when the macrocell tristates, the clock is
         * whatever the board drives.
         *
         * The port must be readable. Registered pins with a PT-OE are
         * already `inout`; a combinatorial or always-on output would be
         * `out`, which VHDL-93 forbids reading, so it is promoted to
         * `buffer`. */
        /* All or nothing per design. ISE will not globalise a
         * combinatorial pad -- it is gated logic, and it reads back 'Z' while
         * tristated, which the internal signal cannot represent -- and a
         * half-globalised design fits worse than a consistently internal one.
         * So if any claimed GCK pad of this design is not registered, none
         * of them is converted and all keep the internal-signal alias. */
        if (!gck_all_convertible) {
            if (v->has_q[gp])              snprintf(v->gck_alias[g], 24, "q_%s", pnm);
            else if (v->comb_out_pin[gp])  snprintf(v->gck_alias[g], 24, "c_%s", pnm);
            else                           snprintf(v->gck_alias[g], 24, "%s",   pnm);
            continue;
        }
        snprintf(v->gck_alias[g], 24, "%s", pnm);
        v->gck_pad_readback[gp] = 1;
    }
    /* Pass 2: mark input pins from all literals */
    scan_inputs(v);
    (void)jed_basename;
}

/* ===========================================================================
 * 3  VHDL expression emitters
 * =========================================================================== */

/* A global OE net n (1-based) enables its outputs on '0' when the macrocell's own XL polarity (COEINV) says so
 * XOR the non-XL net carries its pin's complement (FB1 OE<n-1>I). On XL parts gts_inv is always 0. */
static int gts_active_low(const vctx_t *v, const mc_t *m, int n) {
    return (m->oe_gts_invert ? 1 : 0) ^ (n >= 1 && n <= 4 ? v->gts_inv[n] : 0);
}
static const char *gts_note(const vctx_t *v, int n) {
    static char b[96];
    if (n < 1 || n > 4 || (v->gts_src[n] == n && !v->gts_inv[n])) return "";
    if (v->gts_src[n] != n)
        snprintf(b, sizeof b, " (net GTS%d <- pin GTS%d%s)", n, v->gts_src[n], v->gts_inv[n] ? ", complement" : "");
    else snprintf(b, sizeof b, " (net GTS%d = complement of its pin: OE%dI)", n, n - 1);
    return b;
}

/* Name to use for global clock g (1..3): either the input port gckN, or the
 * internal signal of the output that occupies the GCK pad.  See gck_alias. */
static const char *gck_sig_name(const vctx_t *v, int g) {
    static const char *ports[] = { "", "gck1", "gck2", "gck3" };
    if (g < 1 || g > 3) return "";
    if (v->gck_src[g]) g = v->gck_src[g];   /* g is the NET; the port is its PIN */
    if (v->gck_alias[g][0]) return v->gck_alias[g];
    return ports[g];
}

/* Is this macrocell emitted under its mc_label() name? True for a
 * buried registered macrocell (v->buried) and for a buried combinatorial
 * one carrying data product terms. The collector at "Classify MC" takes
 * only !m->pin && !m->is_combi, so buried combinatorial nodes are found
 * here. */
static int buried_emitted(const vctx_t *v, int fb, int mc) {
    for (int i = 0; i < v->n_buried; i++)
        if (v->buried[i]->fb == fb && v->buried[i]->mc == mc) return 1;
    for (int i = 0; i < v->n_active; i++) {
        const mc_t *m = v->active[i];
        /* The `n_data_pts > 0` test here must match the emission guard,
         * which does not require terms for a referenced node; otherwise the
         * macrocell is emitted as `fbNmcM` while its references are also
         * declared as unresolved stubs beside it. */
        if (m->fb == fb && m->mc == mc && m->is_combi && !drives_pad(m)
            && (m->n_data_pts > 0
                || mc_is_referenced(v->chip, v->dev, fb, mc))) return 1;
    }
    return 0;
}

/* Convert a pair_signal_name() result to a valid VHDL expression.
 * If the signal is a registered output pin, substitute the q_ signal.
 * Unresolved INPUT{N}(imux=C) signals are sanitised to valid identifiers
 * of the form unk_N_cC  (declared as local signals, value '0'). */
static void lit_to_expr(const vctx_t *v, const char *raw, char *out, int sz);

/* A multi-survivor plane row is a FastCONNECT wired-AND: the vendor
 * writes each such row as one product term of its members. Each component
 * goes through lit_to_expr() unchanged; parenthesised because a pair
 * literal sits inside an AND of literals. A member whose FBINV is
 * programmed enters the AND complemented; decode marks it '!'
 * (nonxl_plane_wired_or_name) and it is emitted as `not <member>`.
 * The composite keeps decode's internal '|' member separator (decode.h);
 * '|' lists the wired-AND's members, it is not an OR. */
static int or_expand_expr(const vctx_t *v, const char *base, const char *pol,
                          char *out, int sz) {
    char parts[OR_MAX_PARTS][OR_PART_LEN];
    int np, i, used;
    if (strchr(base, '|') == NULL) return 0;
    np = or_parts(base, parts, OR_MAX_PARTS);
    if (np < 2) return 0;
    used = snprintf(out, sz, "%s(", pol);
    if (used < 0 || used >= sz) return 0;
    for (i = 0; i < np; i++) {
        char one[LITEXPR_MAX];
        int n;
        if (parts[i][0] == '!') {            /* FBINV member */
            char inner[LITEXPR_MAX];
            lit_to_expr(v, parts[i] + 1, inner, sizeof inner);
            n = snprintf(one, sizeof one, "not %s", inner);
            if (n < 0 || n >= (int)sizeof one) return 0;   /* never truncate */
        } else
            lit_to_expr(v, parts[i], one, sizeof one);
        n = snprintf(out + used, sz - used, "%s%s", i ? " and " : "", one);
        if (n < 0 || n >= sz - used) return 0;   /* never truncate */
        used += n;
    }
    if (used + 1 >= sz) return 0;
    out[used] = ')'; out[used + 1] = 0;
    return 1;
}

static void lit_to_expr(const vctx_t *v, const char *raw, char *out, int sz) {
    const char *pol  = "";
    const char *base = raw;
    if (strncmp(raw, "NOT_", 4) == 0) { pol = "not "; base = raw + 4; }

    /* A wired pair. This must come before every test below:
     * sig_to_pin() would parse the leading "n_FB2MC15" out of the composite
     * and emit one of the sources as if it were the whole pair. */
    if (strchr(base, '|') != NULL) {
        if (or_expand_expr(v, base, pol, out, sz)) return;
        /* Does not fit, or a malformed component: never emit part of the
         * composite. Fall through to the sanitiser, which yields an
         * unresolved identifier exactly as a placeholder would. */
    }

    /* A marked guess ends in '?' or '?NofM'. This test must come before
     * sig_to_pin() below: that function parses the leading "n_pNN" out of a
     * name like "n_p1?1of14" and returns pin 1, so the marker would be
     * dropped and the guess emitted as fact. Emitted code never asserts an
     * unverified name, so a guessed literal becomes an unresolved
     * placeholder in the VHDL. */
    if (strchr(base, '?') != NULL) {
        char san[64]; int si = 0, ci;
        for (ci = 0; base[ci] && si < 62; ci++) {
            char c = (char)tolower((unsigned char)base[ci]);
            san[si++] = isalnum((unsigned char)c) ? c : '_';
        }
        san[si] = 0;
        snprintf(out, sz, "%sguess_%s", pol, san);
        return;
    }

    {
      int pin = sig_to_pin(base);
      if (pin > 0 && !pin_usable(v->dev, pin)) {
        /* Off-package pin from another package's table. Emit a placeholder;
         * it is declared as an unresolved signal and never receives a LOC. */
        snprintf(out, sz, "%soffpkg_p%d", pol, pin);
        return;
      }
      if (pin > 0) {
        char pnm[16];
        pin_sig(pin, pnm, sizeof pnm);
        if (pin > 0 && pin < MAX_PINS && v->reg_out_pin[pin]) {
            snprintf(out, sz, "%sq_%s", pol, pnm);
            return;
        }
        /* A literal naming a global pad that the design uses as that
         * global must spell the port name: the pad is stripped from the
         * ordinary input list, so `n_p22` would have no declaration. */
        if (pin > 0 && v->dev) {
            int gg;
            for (gg = 1; gg <= 3; gg++)
                if ((v->gck_used & (1 << (gg-1))) && gck_pin_num(v->dev, gg) == pin
                    && v->gck_alias[gg][0] == '\0') {
                    /* Only when the port exists. gck_alias is set when the pad
                     * is also a macrocell output, in which case the literal
                     * keeps that spelling, not the port name. */
                    snprintf(out, sz, "%sgck%d", pol, gg); return;
                }
            for (gg = 1; gg <= 4; gg++)   /* GTS3/GTS4 too */
                if ((v->gts_used & (1 << (gg-1))) && gts_pin_num(v->dev, gg) == pin) {
                    snprintf(out, sz, "%sgts%d", pol, gg); return;
                }
            if (v->gsr_used && gsr_pin_num(v->dev) == pin) {
                snprintf(out, sz, "%sgsr", pol); return;
            }
        }
        {   const mc_t *rc = reclassified_pad(v, pin);
            if (rc) {
                char lbl[32];
                mc_label(rc->fb, rc->mc, lbl, sizeof lbl);
                snprintf(out, sz, "%s%s", pol, lbl);
                return;
            }
        }
        if (pin > 0 && pin < MAX_PINS && v->comb_out_pin[pin]) {
            /* For an open-drain output the pre-tristate value c_ is a
             * constant; what consumers see is the pin, which floats high when
             * the OE is off. Read the port. */
            if (v->od_pin[pin]) { snprintf(out, sz, "%s%s", pol, pnm); return; }
            snprintf(out, sz, "%sc_%s", pol, pnm);
            return;
        }
        snprintf(out, sz, "%s%s", pol, pnm);
        return;
      }
    }
    /* A reference to a buried macrocell.
     *
     * decode.c's signal_name() spells a pinless macrocell "n_FB{f}MC{m}".
     * This file's convention (mc_label() above) is "fb{f}mc{m}" with no n_
     * prefix, and that is the name the buried macrocell's own logic is
     * emitted under. Rewrite the reference to the driven name, but only if
     * that macrocell is in this design's buried list. If it is not emitted,
     * fall through and keep the unresolved placeholder rather than refer to
     * an identifier that does not exist. */
    {
        int bf = 0, bm = 0;
        if (sscanf(base, "n_FB%dMC%d", &bf, &bm) == 2) {
            if (buried_emitted(v, bf, bm)) {
                char lbl[32];
                mc_label(bf, bm, lbl, sizeof lbl);
                snprintf(out, sz, "%s%s", pol, lbl);
                return;
            }
            /* The same name can also denote a pinned macrocell's output:
             * the decoder names a macrocell's output `n_FB{f}MC{m}` wherever
             * the fuses select the output rather than the pad. Spell it the
             * way the emitter drives it, or every product term reading it
             * would be ANDed with an unresolved '0' stub. */
            if (bf >= 1 && bf <= v->dev->n_fbs &&
                bm >= 1 && bm <= MACROCELLS_PER_FB) {
                const mc_t *sm = &v->chip->mc[bf-1][bm-1];
                int sp = sm->pin;
                if (sp > 0 && sp < MAX_PINS) {
                    char pnm[16]; pin_sig(sp, pnm, sizeof pnm);
                    if (v->reg_out_pin[sp]) {
                        snprintf(out, sz, "%sq_%s", pol, pnm); return;
                    }
                    if (v->comb_out_pin[sp]) {
                        snprintf(out, sz, "%sc_%s", pol, pnm); return;
                    }
                    if (v->input_pin[sp]) {          /* the pad is an input */
                        snprintf(out, sz, "%s%s", pol, pnm); return;
                    }
                }
            }
        }
    }

    /* Unresolved IMUX: INPUT{N} or INPUT{N}(imux=C)  or PAIR{N}
     * Sanitise to valid VHDL: replace non-alnum with '_', strip trailing _ */
    /* Non-XL stub FC tables yield names like FB4_PAIR17, which start with
     * "FB" and would otherwise fall through to the generic branch below and
     * never be declared (ERROR:HDLParsers:1209). Match "PAIR" anywhere.
     * Resolved names (n_pNN, q_n_pNN, fbNmcM) never contain the uppercase
     * substring, so this cannot over-match. */
    /* Also catch the multi-source diagnostic string from decode.c's
     * pair_signal_name() (pin names joined with '|', e.g. "n_p5|n_p12") --
     * '|' is not a valid VHDL identifier character. It goes through the same
     * sanitiser as INPUT/PAIR, giving a placeholder identifier instead of a
     * compile failure; decode's own output is unaffected. */
    /* A name ending in '?' is a marked guess (see xl9536_code2_guess in
     * decode.c). The decode output shows it, so a reader sees both the
     * candidate and its uncertainty. Emitted code must not assert it: it goes
     * through the same sanitiser as INPUT/PAIR, so the VHDL gets a
     * placeholder identifier instead. */
    if (strncmp(base, "INPUT", 5) == 0 || strstr(base, "PAIR") != NULL
        || strchr(base, '|') != NULL || strchr(base, '?') != NULL) {
        char san[64]; int si = 0;
        for (int ci = 0; base[ci] && si < 62; ci++) {
            char c = (char)tolower((unsigned char)base[ci]);
            if (isalnum((unsigned char)c)) san[si++] = c;
            else                          san[si++] = '_';
        }
        while (si > 0 && san[si-1] == '_') si--;
        san[si] = '\0';
        snprintf(out, sz, "%s%s", pol, san);
        return;
    }
    /* Buried / other: lowercase */
    char lb[64]; int i;
    for (i = 0; base[i] && i < 63; i++) lb[i] = tolower((unsigned char)base[i]);
    lb[i] = '\0';
    snprintf(out, sz, "%s%s", pol, lb);
}

/* Emit one product term as a VHDL expression to file f.
 * Returns the number of literals emitted (0 -> constant '0'). */
static int identity_override(const vctx_t *v, int fb, int pair, int is_comp,
                             char *buf, int sz) {
    return identity_override_c(v->chip, v->dev, fb, pair, is_comp, buf, sz);
}

static int emit_pt_expr(FILE *f, const vctx_t *v, int fb, const pt_t *pt) {
    if (pt->n_lits == 0) { fprintf(f, "'0'"); return 0; }
    if (pt->n_lits == 1) {
        char buf[SIGNAME_MAX], expr[LITEXPR_MAX];
        pair_signal_name(v->dev, v->chip->fuses, fb,
                         pt->lits[0].pair, pt->lits[0].is_comp, buf, sizeof buf);
        identity_override_c(v->chip, v->dev, fb, pt->lits[0].pair, pt->lits[0].is_comp, buf, sizeof buf);
        identity_override(v, fb, pt->lits[0].pair, pt->lits[0].is_comp, buf, sizeof buf);
        lit_to_expr(v, buf, expr, sizeof expr);
        fprintf(f, "%s", expr);
        return 1;
    }
    fprintf(f, "(");
    for (int l = 0; l < pt->n_lits; l++) {
        char buf[SIGNAME_MAX], expr[LITEXPR_MAX];
        pair_signal_name(v->dev, v->chip->fuses, fb,
                         pt->lits[l].pair, pt->lits[l].is_comp, buf, sizeof buf);
        identity_override_c(v->chip, v->dev, fb, pt->lits[l].pair, pt->lits[l].is_comp, buf, sizeof buf);
        identity_override(v, fb, pt->lits[l].pair, pt->lits[l].is_comp, buf, sizeof buf);
        lit_to_expr(v, buf, expr, sizeof expr);
        if (l > 0) fprintf(f, " and ");
        fprintf(f, "%s", expr);
    }
    fprintf(f, ")");
    return pt->n_lits;
}

/* Emit a sum-of-products (OR of PTs) as a VHDL expression. */
static void emit_sop_expr(FILE *f, const vctx_t *v, int fb,
                           const pt_t *pts, int n, int indent) {
    if (n == 0) { fprintf(f, "'0'"); return; }
    for (int p = 0; p < n; p++) {
        if (p > 0) {
            fprintf(f, "\n");
            for (int i = 0; i < indent; i++) fputc(' ', f);
            fprintf(f, "or ");
        }
        emit_pt_expr(f, v, fb, &pts[p]);
    }
}

/* True when a condition PT has more than one literal
 * (-> benefits from an intermediate named signal).
 * PT-CLK always gets an intermediate (even if 1 literal) for clarity. */
static int pt_is_complex(const pt_t *pt) {
    return pt->n_lits > 1;
}

/* Emit the clock expression for a process sensitivity list / edge test.
 * For PT-CLK, always uses the _clk intermediate signal (even if 1 literal). */
static void emit_clk_expr(FILE *f, const vctx_t *v __attribute__((unused)), const mc_t *m) {
    if (m->clk_mode == CLK_MODE_PT) {
        char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
        fprintf(f, "%s_clk", lbl);   /* always use intermediate for PT-CLK */
    } else {
        int idx = (m->clk_mode==CLK_MODE_GCK1)?1:(m->clk_mode==CLK_MODE_GCK2)?2:
                  (m->clk_mode==CLK_MODE_GCK3)?3:0;
        fprintf(f, "%s", gck_sig_name(v, idx));
    }
}

/* ===========================================================================
 * 4  Entity / architecture emission
 * =========================================================================== */

static void emit_header(FILE *f, const vctx_t *v, const char *entity, const char *sigtxt) {
    fprintf(f,
        "-- ============================================================\n"
        "-- Decompiled VHDL - %s\n"
        "-- Generated by xc9500dump (xc9500tool) " XC9500TOOL_VERSION "\n"
        "-- user signature: %s\n"
        "-- NOTE: Signals prefixed 'input' are unresolved IMUX sources\n"
        "--       (IMUX table gap or A-byte contamination). Value = '0'.\n"
        "-- ============================================================\n"
        "\n"
        "library IEEE;\n"
        "use IEEE.STD_LOGIC_1164.ALL;\n\n",
        v->dev->id, sigtxt);

    /* If the JED never named its package, the resolved row is a generic
     * family row carrying a default package's pin table, so every LOC below
     * is an assumption. Label it; the names are not changed, because the
     * default is often right and another guess would be no better founded. */
    if (device_is_generic(v->dev))
        fprintf(f,
        "-- WARNING: the source JED never named its package. Every pin name and\n"
        "--          LOC below is ASSUMED from this family's default package\n"
        "--          table and may be wrong for the real board. Macrocell names\n"
        "--          (FBn_MCm) are unaffected. Verify before trusting any LOC.\n"
        "-- ============================================================\n");

    /* -- Entity --------------------------------------------------------- */
    const device_t *dev = v->dev;
    fprintf(f, "entity %s is\n  port (\n", entity);

    /* Collect port lines (with embedded semicolon before comment), then
     * emit them -- stripping the semicolon from the last one only. */
    char ports[256][128]; int n_ports = 0;
    {   /* unresolved literals become real input ports */
        int ui;
        for (ui = 0; ui < v->n_unres && n_ports < 250; ui++)
            snprintf(ports[n_ports++], 128,
                     "    %-6s : in  std_logic;            -- P%d (unresolved FC pair, kept as a port so the fitter cannot remove it)",
                     v->unres[ui], v->unres_pin[ui]);
    }
    char sgb[16], lcb[12];   /* scratch for pin_sig()/pin_loc() */

    /* Global clocks */
    for (int g = 1; g <= 3; g++) {
        if (!(v->gck_used & (1 << (g-1)))) continue;
        if (v->gck_alias[g][0]) continue;   /* pad is driven as an output */
        int pin = gck_pin_num(dev, g);
        snprintf(ports[n_ports++], 128,
                 "    gck%d   : in  std_logic;            -- %s (GCK%d)",
                 g, (pin_loc(pin, lcb, sizeof lcb), lcb), g);
    }
    /* GTS -- XC95144/216/288 have four */
    for (int g = 1; g <= 4; g++) {
        if (!(v->gts_used & (1 << (g-1)))) continue;
        int pin = gts_pin_num(dev, g);
        snprintf(ports[n_ports++], 128,
                 "    gts%d   : in  std_logic;            -- %s (GTS%d)",
                 g, (pin_loc(pin, lcb, sizeof lcb), lcb), g);
    }
    /* GSR */
    if (v->gsr_used) {
        int pin = gsr_pin_num(dev);
        snprintf(ports[n_ports++], 128,
                 "    gsr    : in  std_logic;            -- %s (GSR)",
                 (pin_loc(pin, lcb, sizeof lcb), lcb));
    }
    /* Input pins */
    for (int pin = 1; pin < MAX_PINS; pin++) {
        if (!v->input_pin[pin]) continue;
        snprintf(ports[n_ports++], 128,
                 "    %-6s : in  std_logic;            -- %s",
                 (pin_sig(pin, sgb, sizeof sgb), sgb),
                 (pin_loc(pin, lcb, sizeof lcb), lcb));
    }
    /* Output / bidirectional pins */
    for (int pin = 1; pin < MAX_PINS; pin++) {
        const mc_t *m = v->out_mc[pin];
        if (!m) continue;
        const char *dir;
        /* An open-drain combinational output (PT-OE) is read back by other
         * logic through its pin, so it must be `inout`; VHDL-93 forbids
         * reading a port of mode `out`. */
        if (m->is_combi && m->has_oe_pt && m->oe_mode == 3) dir = "inout";
        else if (m->is_combi || m->oe_mode==1) dir = "out  ";
        else                              dir = "inout";
        /* A GCK pad whose clock is read from the port cannot be mode
         * `out`. Promote it; `inout` is already readable. */
        if (v->gck_pad_readback[pin] && dir[0] == 'o') dir = "buffer";
        char anno[64] = "";
        if (m->is_combi)        snprintf(anno,sizeof anno," (combinatorial)");
        else if (m->pwr_low && m->slew_slow) snprintf(anno,sizeof anno," [low-pwr,slow]");
        else if (m->pwr_low)    snprintf(anno,sizeof anno," [low-pwr]");
        else if (m->slew_slow)  snprintf(anno,sizeof anno," [slow]");
        snprintf(ports[n_ports++], 128,
                 "    %-6s : %s std_logic;          -- %s  FB%d_MC%d%s",
                 (pin_sig(pin, sgb, sizeof sgb), sgb), dir,
                 (pin_loc(pin, lcb, sizeof lcb), lcb), m->fb, m->mc, anno);
    }

    /* Emit: semicolon already embedded; strip it from the last port */
    for (int p = 0; p < n_ports; p++) {
        if (p < n_ports - 1) {
            fprintf(f, "%s\n", ports[p]);
        } else {
            /* Last port: remove the semicolon before the comment */
            char last[128];
            strncpy(last, ports[p], 127); last[127] = '\0';
            char *sc = strchr(last, ';');
            if (sc) memmove(sc, sc+1, strlen(sc));  /* remove ; */
            fprintf(f, "%s\n", last);
        }
    }
    fprintf(f, "  );\nend entity %s;\n\n", entity);
}

static void emit_arch_signals(FILE *f, const vctx_t *v, const char *entity) {
    fprintf(f, "architecture rtl of %s is\n\n", entity);

    /* The attribute types must be declared before the first signal that
     * carries them (the q_ block comes before the buried block); otherwise
     * ISE rejects the file (HDLParsers:3312).
     *
     * Why these attributes: macrocells are lost on recompile in two ways.
     * A buried node has no external observability, so XST absorbs it --
     * `keep`/`S` stop that. A chain of single-product-term registers is
     * collapsed into a shift register -- `shreg_extract`/`register_balancing`
     * stop that, and they apply to pinned registers too. All four are
     * synthesis directives, inert to a simulator. */
    fprintf(f, "  -- Preserve structure on recompile. Buried nodes are\n");
    fprintf(f, "  -- absorbed without keep/S; register chains are collapsed\n");
    fprintf(f, "  -- into shift registers without shreg_extract.\n");
    fprintf(f, "  attribute keep : string;\n");
    fprintf(f, "  attribute S    : string;\n");
    fprintf(f, "  attribute shreg_extract : string;\n");
    fprintf(f, "  attribute register_balancing : string;\n\n");

    /* q_ signals for registered pin-outputs */
    int any_q = 0;
    for (int pin = 1; pin < MAX_PINS; pin++) {
        const mc_t *m = v->out_mc[pin];
        if (!m || m->is_combi || !v->has_q[pin]) continue;
        if (!any_q) {
            fprintf(f, "  -- Internal registered signals (before output-enable logic)\n");
            any_q = 1;
        }
        {   /* mc_qname() so BGA parts get q_ball_XX, matching what the body
             * emits. */
            char qn[24];
            mc_qname(v->dev, m->fb, m->mc, qn, sizeof qn);
            fprintf(f, "  signal %-12s : std_logic := '%c';  -- FB%d_MC%d\n",
                    qn, m->init_set ? '1' : '0', m->fb, m->mc);
            /* Pinned registered outputs need the structural attributes too:
             * single-product-term register chains on pins are otherwise
             * inferred as shift registers. `keep` alone cannot help here: the
             * signal still exists, it is the chain structure XST rewrites. */
            fprintf(f, "  attribute shreg_extract of %-12s : signal is \"no\";\n", qn);
            fprintf(f, "  attribute register_balancing of %-12s : signal is \"no\";\n", qn);
        }
    }
    if (any_q) fprintf(f, "\n");


    /* Buried macrocell signals */
    if (v->n_buried > 0) {
        fprintf(f, "  -- Buried (non-output) macrocells\n");
        for (int i = 0; i < v->n_buried; i++) {
            const mc_t *m = v->buried[i];
            char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
            fprintf(f, "  signal %-14s : std_logic := '%c';  -- FB%d_MC%d\n",
                    lbl, m->init_set ? '1' : '0', m->fb, m->mc);
        }
        /* Keep the buried nodes. A buried node has no external
         * observability, so XST is free to absorb it into its consumers, and
         * the macrocell is lost from the rebuild.
         *
         * XST honours both `keep` and the older `S` (save) attribute on a
         * signal; both are declared so the file works with either front end.
         * They are synthesis directives, inert to a simulator and to any
         * other tool that ignores unknown attributes. */
        for (int i = 0; i < v->n_buried; i++) {
            const mc_t *m = v->buried[i];
            char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
            fprintf(f, "  attribute keep of %-14s : signal is \"true\";\n", lbl);
            fprintf(f, "  attribute S    of %-14s : signal is \"yes\";\n", lbl);
            if (!m->is_combi) {
                fprintf(f, "  attribute shreg_extract of %-14s : signal is \"no\";\n", lbl);
                fprintf(f, "  attribute register_balancing of %-14s : signal is \"no\";\n", lbl);
            }
        }
        fprintf(f, "\n");
    }

    {   /* buried combinatorial nodes carrying logic */
        int any_bc = 0;
        for (int i = 0; i < v->n_active; i++) {
            const mc_t *m = v->active[i];
            /* A buried combinatorial macrocell with zero product terms is
             * still a real node other logic reads. Left undriven, every
             * consumer would read an undefined signal. Emit it -- its empty
             * sum-of-products is '0', and a driven '0' is well-formed where an
             * undriven signal is not. Emit only when something reads it. */
            if (!m->is_combi || drives_pad(m)) continue;
            if (m->n_data_pts <= 0 && !mc_is_referenced(v->chip, v->dev, m->fb, m->mc))
                continue;
            if (!any_bc) {
                fprintf(f, "  -- Buried combinatorial nodes\n");
                any_bc = 1;
            }
            char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
            fprintf(f, "  signal %-14s : std_logic;  -- FB%d_MC%d\n",
                    lbl, m->fb, m->mc);
            fprintf(f, "  attribute keep of %-14s : signal is \"true\";\n", lbl);
            fprintf(f, "  attribute S    of %-14s : signal is \"yes\";\n", lbl);
        }
        if (any_bc) fprintf(f, "\n");
    }

    /* Intermediate condition signals */
    int any_cond = 0;
    for (int i = 0; i < v->n_active; i++) {
        const mc_t *m = v->active[i];
        if (m->is_combi) continue;
        char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);

        int need_rst = m->has_reset  && pt_is_complex(&m->rst_pt);
        int need_pst = m->has_preset && pt_is_complex(&m->pst_pt);
        int need_clk = m->clk_mode == CLK_MODE_PT;
        int need_ce  = m->has_ce    && pt_is_complex(&m->ce_pt);
        int need_oe  = m->has_oe_pt && pt_is_complex(&m->oe_pt);

        if (need_rst || need_pst || need_clk || need_ce || need_oe) {
            if (!any_cond) {
                fprintf(f, "  -- Intermediate condition signals\n");
                any_cond = 1;
            }
        }
        if (need_rst) fprintf(f, "  signal %s_rst : std_logic;\n", lbl);
        if (need_pst) fprintf(f, "  signal %s_pst : std_logic;\n", lbl);
        if (need_clk) fprintf(f, "  signal %s_clk : std_logic;\n", lbl);
        if (need_ce)  fprintf(f, "  signal %s_ce  : std_logic;\n", lbl);
        if (need_oe)  fprintf(f, "  signal %s_oe  : std_logic;\n", lbl);
    }
    if (any_cond) fprintf(f, "\n");

    /* Unresolved IMUX signals (unk_NNN_cCC): collect from all PTs, declare as '0' */
    {
        /* Simple dedup: collect unique sanitised names in a local buffer */
        char seen[512][32]; int n_seen = 0;
        /* Helper: sanitise INPUT{N}(imux=C) to unk_{N}_c{C} */
        /* We re-scan every PT to find unresolved names */
        int any_unk = 0;
        const chip_t *chip = v->chip;
        for (int ia = 0; ia < v->n_active; ia++) {
            const mc_t *m = v->active[ia];
            /* Scan all PT groups */
            const pt_t *pts[7]; int npts[7];
            pts[0]=m->data_pts;   npts[0]=m->n_data_pts;
            pts[1]=&m->rst_pt;    npts[1]=m->has_reset  ? 1 : 0;
            pts[2]=&m->pst_pt;    npts[2]=m->has_preset ? 1 : 0;
            pts[3]=&m->ce_pt;     npts[3]=m->has_ce     ? 1 : 0;
            pts[4]=&m->oe_pt;     npts[4]=m->has_oe_pt  ? 1 : 0;
            pts[5]=&m->clk_pt;   npts[5]=m->has_clk_pt ? 1 : 0;
            pts[6]=&m->ptinv_pt; npts[6]=m->has_ptinv  ? 1 : 0;
            for (int g = 0; g < 7; g++) {
                for (int p = 0; p < npts[g]; p++) {
                    for (int l = 0; l < pts[g][p].n_lits; l++) {
                        char raw[SIGNAME_MAX];
                        pair_signal_name(v->dev, chip->fuses, m->fb,
                                         pts[g][p].lits[l].pair,
                                         pts[g][p].lits[l].is_comp,
                                         raw, sizeof raw);
                        identity_override_c(v->chip, v->dev, m->fb, pts[g][p].lits[l].pair, pts[g][p].lits[l].is_comp, raw, sizeof raw);
                        /* A wired pair emits several identifiers, so each
                         * is declaration-checked on its own. Passing the
                         * composite through lit_to_expr() and sanitising the
                         * result would declare one truncated "(a or b ...)"
                         * name the body never emits. */
                        char oparts[OR_MAX_PARTS][OR_PART_LEN];
                        const char *rawpol  = (strncmp(raw,"NOT_",4)==0) ? "NOT_" : "";
                        const char *rawbase = raw + strlen(rawpol);
                        int onp = or_parts_names(rawbase, oparts, OR_MAX_PARTS), opi;
                        for (opi = 0; opi < (onp ? onp : 1); opi++) {
                        char rawbuf[SIGNAME_MAX];
                        const char *rawc;
                        /* onp == 0: the name would not split, so use it
                         * exactly as it is -- no copy, so no truncation. */
                        if (!onp) rawc = raw;
                        else {
                            snprintf(rawbuf, sizeof rawbuf, "%s%.*s",
                                     rawpol, OR_PART_LEN - 1, oparts[opi]);
                            rawc = rawbuf;
                        }
                        /* Declare every identifier the body will emit that
                         * is not already a port or a known signal. lit_to_expr()
                         * makes the name byte-identical to what the expression
                         * emitter produces; it is then tested against the same
                         * predicates the declaration emitter used. Otherwise
                         * XST fails with ERROR:HDLParsers:1209. */
                        char ex[LITEXPR_MAX];
                        lit_to_expr(v, rawc, ex, sizeof ex);
                        const char *san_p = ex;
                        if (strncmp(san_p,"not ",4)==0) san_p += 4;
                        char san[32];
                        /* Never truncate. This name is compared against the
                         * declared set and then emitted as a signal
                         * declaration, so a truncated copy would declare an
                         * identifier the body never uses. Skipping instead makes
                         * XST fail loudly (HDLParsers:1209). No name the emitter
                         * produces reaches 31 characters. */
                        {   size_t sl = strlen(san_p);
                            if (sl >= sizeof san) continue;
                            memcpy(san, san_p, sl + 1); }
                        {   /* already declared? */
                            /* parse via sig_to_pin() so ball_XN names are
                             * recognised too. */
                            int declared = 0, pn = 0;
                            if (strncmp(san,"q_",2)==0 && (pn=sig_to_pin(san+2))>0)
                                declared = (pn<MAX_PINS && v->has_q[pn]);
                            else if (strncmp(san,"c_",2)==0 && (pn=sig_to_pin(san+2))>0)
                                declared = (pn<MAX_PINS && v->comb_out_pin[pn]);
                            else if ((pn=sig_to_pin(san))>0) {
                                declared = (pn<MAX_PINS &&
                                            (v->input_pin[pn] || v->out_mc[pn]));
                                /* A global pad in use is declared as gck1/gts1/gsr,
                                 * not in input_pin[] (it is stripped from the
                                 * ordinary input list). It is still a declared
                                 * port, so a literal naming it must not become
                                 * an unresolved stub tied to '0'. */
                                if (!declared && v->dev) {
                                    int gg;
                                    for (gg = 1; gg <= 3 && !declared; gg++)
                                        if ((v->gck_used & (1 << (gg-1))) &&
                                            gck_pin_num(v->dev, gg) == pn) declared = 1;
                                    for (gg = 1; gg <= 4 && !declared; gg++)   /* GTS3/GTS4 */
                                        if ((v->gts_used & (1 << (gg-1))) &&
                                            gts_pin_num(v->dev, gg) == pn) declared = 1;
                                    if (!declared && v->gsr_used &&
                                        gsr_pin_num(v->dev) == pn) declared = 1;
                                }
                            }
                            else {
                                int bf=0,bm=0;
                                /* buried_emitted() also covers buried
                                 * combinatorial nodes; scanning v->buried alone
                                 * would re-declare them as unresolved beside
                                 * their real declaration (HDLParsers:1202). */
                                if (sscanf(san,"fb%dmc%d",&bf,&bm)==2 &&
                                    buried_emitted(v, bf, bm))
                                    declared = 1;
                            }
                            /* lit_to_expr() has already renamed a global pad to
                             * its port name, so test the port name directly. */
                            if (!declared &&
                                (strcmp(san,"gck1")==0 || strcmp(san,"gck2")==0 ||
                                 strcmp(san,"gck3")==0 || strcmp(san,"gts1")==0 ||
                                 strcmp(san,"gts2")==0 || strcmp(san,"gts3")==0 ||
                                 strcmp(san,"gts4")==0 || strcmp(san,"gsr")==0))
                                declared = 1;
                            if (declared) continue;
                        }
                        /* Check seen */
                        int dup=0;
                        for(int s=0;s<n_seen;s++) if(strcmp(seen[s],san)==0){dup=1;break;}
                        if(dup) continue;
                        if(n_seen<512) { snprintf(seen[n_seen],sizeof(seen[0]),"%s",san); n_seen++; }
                        if(!any_unk){
                            fprintf(f,"  -- Unresolved IMUX signals (IMUX table gap or A-byte contamination)\n");
                            fprintf(f,"  -- These are set to '0'; actual sources unknown.\n");
                            any_unk=1;
                        }
                        {
                            /* If this is an unresolved FastCONNECT pair (not an XL
                             * "input_N" placeholder) and a candidate-pool hint
                             * exists for it (from Xilinx's .chp architecture data,
                             * see fc_hint_t in device.h), print it as a
                             * diagnostic comment only. It lists physically
                             * possible sources, never which one is active, so it
                             * is not used to pick a signal name. */
                            int hfb=0, hpair=0;
                            char hint[256]; hint[0] = '\0';
                            if (v->dev->fc_hints &&
                                sscanf(san, "fb%d_pair%d", &hfb, &hpair) == 2 &&
                                hfb >= 1 && hfb <= v->dev->n_fbs &&
                                hpair >= 0 && hpair < FC_PAIRS_PER_FB) {
                                const fc_hint_t *hp = &v->dev->fc_hints[hfb-1][hpair];
                                if (hp->n_candidates > 0) {
                                    char tmp[32]; int hp_i;
                                    /* names its pair (an unresolved-literal port has
                                     * no signal line above it) and says what
                                     * the list is: fc_hints is a subset of the
                                     * .chp candidates. */
                                    snprintf(hint, sizeof hint,
                                             "  -- %s: possible sources, a subset of the vendor .chp list (unconfirmed which is active): ",
                                             san);
                                    for (hp_i = 0; hp_i < hp->n_candidates; hp_i++) {
                                        snprintf(tmp, sizeof tmp, "%sFB%d_MC%d",
                                                 hp_i ? ", " : "",
                                                 hp->candidates[hp_i].fb,
                                                 hp->candidates[hp_i].mc);
                                        strncat(hint, tmp, sizeof(hint)-strlen(hint)-1);
                                    }
                                }
                            }
                            {
                                /* If this literal got a real port it is declared in
                                 * the entity, so it must not also be declared as
                                 * an architecture signal. */
                                int ui, isport = 0;
                                for (ui = 0; ui < v->n_unres; ui++)
                                    if (!strcmp(v->unres[ui], san)) { isport = 1; break; }
                                if (!isport)
                                    fprintf(f,"  signal %-20s : std_logic := '0';  -- unresolved\n",san);
                            }
                            if (hint[0]) fprintf(f,"%s\n", hint);
                        }
                        }   /* end wired-pair component loop */
                    }
                }
            }
        }
        if(any_unk) fprintf(f,"\n");
    }

    {   /* internal signals for combinatorial outputs that are read */
        int any_c = 0;
        for (int p = 1; p < MAX_PINS; p++) {
            if (!v->comb_out_pin[p]) continue;
            if (!any_c) {
                fprintf(f, "  -- Internal copies of combinatorial outputs.  A port of mode\n");
                fprintf(f, "  -- `out` cannot be read in VHDL-93, and these are also used as\n");
                fprintf(f, "  -- FastCONNECT sources, so the port is driven from the signal.\n");
                any_c = 1;
            }
            char sgc[16]; pin_sig(p, sgc, sizeof sgc);
            fprintf(f, "  signal c_%-6s %*s: std_logic;\n", sgc, 12, "");
            /* Keep the combinational output net. Without it the fitter
             * may absorb the buffer into a register and re-root the pair
             * (e.g. `n_p43.T = Vcc` with `n_p24.D = !n_p43`, reversing which
             * pin toggles and which buffers it), putting one pin a clock out
             * of phase. Absorption alone is safe; re-rooting is not. */
            fprintf(f, "  attribute keep of c_%-6s %*s: signal is \"true\";\n",
                    sgc, 6, "");
        }
        if (any_c) fprintf(f, "\n");
    }

    fprintf(f, "begin\n\n");
}

static void emit_intermediates(FILE *f, const vctx_t *v) {
    int any = 0;
    for (int i = 0; i < v->n_active; i++) {
        const mc_t *m = v->active[i];
        if (m->is_combi) continue;
        char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);

        int need_rst = m->has_reset  && pt_is_complex(&m->rst_pt);
        int need_pst = m->has_preset && pt_is_complex(&m->pst_pt);
        int need_clk = m->clk_mode == CLK_MODE_PT;
        int need_ce  = m->has_ce    && pt_is_complex(&m->ce_pt);
        int need_oe  = m->has_oe_pt && pt_is_complex(&m->oe_pt);

        if (!(need_rst || need_pst || need_clk || need_ce || need_oe)) continue;
        if (!any) {
            fprintf(f, "  -- -- Intermediate combinatorial assignments --------------------\n\n");
            any = 1;
        }
        if (need_rst) {
            fprintf(f, "  %s_rst <= ", lbl);
            emit_pt_expr(f, v, m->fb, &m->rst_pt);
            fprintf(f, ";\n");
        }
        if (need_pst) {
            fprintf(f, "  %s_pst <= ", lbl);
            emit_pt_expr(f, v, m->fb, &m->pst_pt);
            fprintf(f, ";\n");
        }
        if (need_clk) {
            fprintf(f, "  %s_clk <= ", lbl);
            /* The clock term is decoded into its own field. data_pts[0] is only
             * used where slot 0's selector did not identify a dedicated clock
             * (fallback). */
            if (m->has_clk_pt) emit_pt_expr(f, v, m->fb, &m->clk_pt);
            else if (m->n_data_pts > 0) emit_pt_expr(f, v, m->fb, &m->data_pts[0]);
            else fprintf(f, "'0'");
            fprintf(f, ";\n");
        }
        if (need_ce) {
            fprintf(f, "  %s_ce <= ", lbl);
            emit_pt_expr(f, v, m->fb, &m->ce_pt);
            fprintf(f, ";\n");
        }
        if (need_oe) {
            fprintf(f, "  %s_oe <= ", lbl);
            emit_pt_expr(f, v, m->fb, &m->oe_pt);
            fprintf(f, ";\n");
        }
    }
    if (any) fprintf(f, "\n");
}

/* Emit the reset condition expression (inline when simple) */
static void emit_rst_expr(FILE *f, const vctx_t *v, const mc_t *m) {
    char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
    if (pt_is_complex(&m->rst_pt))
        fprintf(f, "%s_rst", lbl);
    else
        emit_pt_expr(f, v, m->fb, &m->rst_pt);
}

static void emit_pst_expr(FILE *f, const vctx_t *v, const mc_t *m) {
    char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
    if (pt_is_complex(&m->pst_pt))
        fprintf(f, "%s_pst", lbl);
    else
        emit_pt_expr(f, v, m->fb, &m->pst_pt);
}

static void emit_ce_expr(FILE *f, const vctx_t *v, const mc_t *m) {
    char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
    if (pt_is_complex(&m->ce_pt))
        fprintf(f, "%s_ce", lbl);
    else
        emit_pt_expr(f, v, m->fb, &m->ce_pt);
}

/* -- Sensitivity list helpers -------------------------------------------- */

/* Collect unique signal names from a PT into a sensitivity-list buffer.
 * 'seen' tracks pair indices already added. 'sigs'/'nsigs' accumulate names. */
static void senlist_collect(const vctx_t *v, int fb, const pt_t *pt,
                             int *seen, int seen_sz,
                             char sigs[][96], int *nsigs) {
    for (int l = 0; l < pt->n_lits; l++) {
        int pair = pt->lits[l].pair;
        if (pair < seen_sz && seen[pair]) continue;
        if (pair < seen_sz) seen[pair] = 1;

        char raw[SIGNAME_MAX], expr[LITEXPR_MAX];
        pair_signal_name(v->dev, v->chip->fuses, fb,
                         pair, 0 /* true polarity */, raw, sizeof raw);
        identity_override_c(v->chip, v->dev, fb, pair, 0 /* true polarity */, raw, sizeof raw);
        /* A sensitivity list holds signal names, never an expression.
         * Passing a wired composite through lit_to_expr() would put
         * "(a or b or c)" in the list, which XST rejects, and truncating it
         * would drop signals the process reads. Enumerate the components. */
        {
            char oparts[OR_MAX_PARTS][OR_PART_LEN];
            int onp = or_parts_names(raw, oparts, OR_MAX_PARTS), opi;
            for (opi = 0; opi < (onp ? onp : 1); opi++) {
                /* onp == 0: use the name whole rather than truncating it. */
                lit_to_expr(v, onp ? oparts[opi] : raw, expr, sizeof expr);
                /* Strip "not " to get the base signal name */
                const char *sig = (strncmp(expr,"not ",4)==0) ? expr+4 : expr;
                /* a truncated sensitivity-list entry is a wrong signal name, so
                 * copy only when it fits. A single identifier always does; an
                 * expression never reaches here. */
                size_t sl = strlen(sig);
                if (sl < 96 && *nsigs < 64) {
                    memcpy(sigs[*nsigs], sig, sl + 1); (*nsigs)++;
                }
            }
        }
    }
}

/* -- Process emission ---------------------------------------------------- */

/* Emit the D/T expression, including the PTINV XOR term.
 *
 * A macrocell whose slot-4 selector is dedicated feeds that product term to
 * the XOR gate ahead of the register, not to the OR gate. cpldfit writes it
 * with `$`:  q.D = p35 $ p50. One more OR term would be a different function
 * -- the two differ wherever both inputs are high.
 *
 * The parentheses are required: VHDL gives `or` and `xor` the same
 * precedence and refuses to mix them unparenthesised, so `a or b xor (c)`
 * is a syntax error. */
static void emit_data_expr(FILE *f, const vctx_t *v, const mc_t *m,
                           const pt_t *pts, int n, int indent) {
    if (m->has_ptinv) fprintf(f, "(");
    /* An unresolved PT export (never merged back into the donor)
     * becomes a deliberate, never-declared identifier, so a design containing
     * one fails synthesis instead of being silently wrong. */
    if (n == 0 && m->has_unresolved_export) {
        fprintf(f, "UNRESOLVED_PTEXPORT_FB%d_MC%d", m->fb, m->mc);
    } else if (oe_honest_rule_fires(m, n)) {
        /* Not for PT-clocked macrocells. (a) OE on, no data terms, the
         * shape matches a PT-dedicated constant (has_oe_pt) and CPTINV is a
         * defined bit -> emit the CPTINV-correct constant, polarity normalised
         * per family; (b) shape does not match -> unresolved marker. */
        /* A truly empty macrocell -- no terms of its own before the export
         * pass and on no export bus -- is the fitter's constant output (`y0 <= '0'` / `y1 <= '1'` fit as exactly this shape on
         * both families, the fitter report saying Gnd/Vcc); its value is the same CPTINV-correct constant as the PT-OE case.
         * Any other n==0 shape stays a marker. */
        int genuinely_empty = (m->pre_export_n == 0 && m->export_role == 0);
        /* An export donor whose every term went onto the bus (no term left, a donor and not a receiver, its export resolved) has
         * the same empty OR -- cpldfit writes `y = Gnd` for this shape with its output always on (fitter report: Total Pt 0,
         * Exp /\2 or \/2); the PT-OE form of the same shape takes this branch through has_oe_pt. */
        if (m->n_data_pts == 0 && m->pre_export_n > 0 && m->export_role == 1 && !m->has_unresolved_export)
            genuinely_empty = 1;
        if ((m->has_oe_pt || genuinely_empty) && m->have_dev_bits && m->devbit[22] >= 0) {
            int cptinv_true = m->dev_is_xl ? (m->devbit[22] == 1)
                                            : (m->devbit[22] == 0);
            fprintf(f, "%s", cptinv_true ? "'1'" : "'0'");
        } else {
            fprintf(f, "UNRESOLVED_PTDATA_FB%d_MC%d", m->fb, m->mc);
        }
    } else {
        emit_sop_expr(f, v, m->fb, pts, n, indent);
        if (m->has_unresolved_export) {
            fprintf(f, "\n");
            for (int i = 0; i < indent; i++) fputc(' ', f);
            fprintf(f, "or UNRESOLVED_PTEXPORT_FB%d_MC%d", m->fb, m->mc);
        }
    }
    if (m->has_ptinv) {
        fprintf(f, ") xor (");
        emit_sop_expr(f, v, m->fb, &m->ptinv_pt, 1, indent);
        fprintf(f, ")");
    }
}

static void emit_one_process(FILE *f, const vctx_t *v, const mc_t *m) {
    char lbl[24];  mc_label(m->fb, m->mc, lbl, sizeof lbl);
    char qname[32]; mc_qname(v->dev, m->fb, m->mc, qname, sizeof qname);
    const char *edge = m->clk_falling ? "falling_edge" : "rising_edge";

    /* Comment header */
    const char *fftype = m->is_toggle ? "T-FF" : "D-FF";
    const char *clkstr = (m->clk_mode==CLK_MODE_GCK1) ? "GCK1" :
                         (m->clk_mode==CLK_MODE_GCK2) ? "GCK2" :
                         (m->clk_mode==CLK_MODE_GCK3) ? "GCK3" : "PT-CLK";
    fprintf(f, "  -- -- FB%d_MC%d  %s  %s %s",
            m->fb, m->mc, fftype, clkstr,
            m->clk_falling ? "FALL" : "RISE");
    if (m->has_reset)  fprintf(f, "  async-RST");
    if (m->has_preset) fprintf(f, "  async-PST");
    if (m->has_ce)     fprintf(f, "  CE");
    if (!m->pin)       fprintf(f, "  [buried]");
    fprintf(f, "\n");

    /* Sensitivity list: collect all signals, then format on one or two lines */
    {
        char sigs[64][96]; int nsigs = 0;
        int seen[256] = {0};

        /* Clock (or PT-CLK intermediate) */
        char clk_sig[48];
        if (m->clk_mode == CLK_MODE_PT) {
            char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
            snprintf(clk_sig, 48, "%s_clk", lbl);
            {   /* mark the clock term's own pairs */
                const pt_t *cp = m->has_clk_pt ? &m->clk_pt
                               : (m->n_data_pts > 0 ? &m->data_pts[0] : NULL);
                if (cp)
                    for (int l = 0; l < cp->n_lits; l++)
                        if (cp->lits[l].pair < 256) seen[cp->lits[l].pair] = 1;
            }
        } else {
            int idx = (m->clk_mode==CLK_MODE_GCK1)?1:(m->clk_mode==CLK_MODE_GCK2)?2:
                      (m->clk_mode==CLK_MODE_GCK3)?3:0;
            snprintf(clk_sig, 48, "%s", gck_sig_name(v, idx));
        }
        snprintf(sigs[nsigs++], 96, "%s", clk_sig);

        /* Async RST/PST signals.
         *
         * When the condition was hoisted into an intermediate signal
         * (pt_is_complex -- see the `%s_rst` / `%s_pst` declarations above),
         * the sensitivity list must name that signal, not the literals it is
         * built from. A process reading `fb1mc1_rst` while sensitive to
         * `n_p32, n_p49, n_p41` does not match XST's async-reset template:
         * XST cannot infer an FDCP and drops the register on refit. With the
         * intermediate in the list cpldfit gives
         *     n_p16.CLK = gck1;
         *     n_p16.AR  = !n_p32 & n_p49 & n_p41 */
        if (m->has_reset) {
            if (pt_is_complex(&m->rst_pt)) {
                char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
                if (nsigs < 32) snprintf(sigs[nsigs++], 96, "%s_rst", lbl);
            } else {
                senlist_collect(v, m->fb, &m->rst_pt, seen, 256, sigs, &nsigs);
            }
        }
        if (m->has_preset) {
            if (pt_is_complex(&m->pst_pt)) {
                char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
                if (nsigs < 32) snprintf(sigs[nsigs++], 96, "%s_pst", lbl);
            } else {
                senlist_collect(v, m->fb, &m->pst_pt, seen, 256, sigs, &nsigs);
            }
        }
        /* GSR: the global net is asynchronous, so it belongs in the
         * sensitivity list like a per-macrocell reset. */
        if ((m->gsr_reset || m->gsr_set) && nsigs < 32)
            snprintf(sigs[nsigs++], 96, "gsr");

        /* Format: single line if <= 72 chars, else indented multi-line */
        int total = 12;
        for (int s = 0; s < nsigs; s++) total += (int)strlen(sigs[s]) + (s?2:0);
        fprintf(f, "  process(");
        for (int s = 0; s < nsigs; s++) {
            if (s) fprintf(f, total <= 72 ? ", " : ",\n           ");
            fprintf(f, "%s", sigs[s]);
        }
        fprintf(f, ")\n  begin\n");
    }

    /* Reset or set by the global net. Emitted ahead of the per-macrocell
     * chain because a macrocell driven by GSR has no reset product term of
     * its own -- its PT4/PT1 selectors read "slot unused". */
    /* GSR beside a product-term reset or preset. The register's async RESET input is the OR of its PT reset and a GSR
     * reset, its async SET input the OR of its PT preset and a GSR set; the reset arm stays first, as in the PT-only chain
     * below. GSR acts as a SET when CSET says so and as a RESET otherwise (`m->gsr_set ? '1' : '0'`). */
    int g_set = m->gsr_set != 0, g_rst = m->gsr_reset && !m->gsr_set;
    int rst_arm = m->has_reset || g_rst, set_arm = m->has_preset || g_set;
    if ((m->gsr_reset || m->gsr_set) && !m->has_reset && !m->has_preset) {
        fprintf(f, "    if gsr = '1' then\n");
        fprintf(f, "      %s <= '%c';\n", qname, m->gsr_set ? '1' : '0');
        fprintf(f, "    elsif %s(", edge);
        emit_clk_expr(f, v, m);
        fprintf(f, ") then\n");
    } else if (rst_arm || set_arm) {
        fprintf(f, "    if ");
        if (rst_arm) {
            if (m->has_reset) { emit_rst_expr(f, v, m); fprintf(f, " = '1'"); }
            if (g_rst) fprintf(f, "%sgsr = '1'", m->has_reset ? " or " : "");
            fprintf(f, " then\n");
            fprintf(f, "      %s <= '0';\n", qname);
            fprintf(f, "    ");
            if (set_arm) fprintf(f, "elsif ");
        }
        if (set_arm) {
            if (m->has_preset) { emit_pst_expr(f, v, m); fprintf(f, " = '1'"); }
            if (g_set) fprintf(f, "%sgsr = '1'", m->has_preset ? " or " : "");
            fprintf(f, " then\n");
            fprintf(f, "      %s <= '1';\n", qname);
            fprintf(f, "    ");
        }
        fprintf(f, "elsif %s(", edge);
        emit_clk_expr(f, v, m);
        fprintf(f, ") then\n");
    } else {
        fprintf(f, "    if %s(", edge);
        emit_clk_expr(f, v, m);
        fprintf(f, ") then\n");
    }

    /* Clock-enable wrapper */
    const char *inner_indent = "      ";
    if (m->has_ce) {
        fprintf(f, "      if ");
        emit_ce_expr(f, v, m);
        fprintf(f, " = '1' then\n");
        inner_indent = "        ";
    }

    /* Registered assignment */
    if (m->ce_const0) {
        /* The clock enable is the constant Gnd (decode.c, ce_const0): the register
         * never loads; only its asynchronous controls act. `q <= q` rather than an `if '0' = '1'` wrapper (XST refuses that). */
        fprintf(f, "%s%s <= %s;  -- CE = Gnd: holds\n", inner_indent, qname, qname);
    } else if (m->is_toggle) {
        /* T-FF: q <= q XOR (sum-of-products)
         * If T input is constant '1', simplify to NOT q */
        int n = m->n_data_pts;
        /* The clock term is kept out of data_pts, so for a
         * product-term-clocked macrocell the remaining terms are data terms
         * and are emitted. The fallback applies only where slot 0's selector
         * did not identify a clock, i.e. where data_pts[0] may still be the
         * clock. */
        if (m->clk_mode == CLK_MODE_PT && !m->has_clk_pt) n = 0;
        fprintf(f, "%s%s <= %s xor (", inner_indent, qname, qname);
        if (n == 0 && m->clk_mode != CLK_MODE_PT && m->n_data_pts == 0 && (m->export_role & 2) == 0
            && !m->has_unresolved_export && m->have_dev_bits && m->devbit[22] >= 0) {
            /* A GCK-clocked T register with no data term has T = the empty OR through CPTINV. CPTINV inactive by the family rule
             * (XL raw 0, non-XL raw 1) -> cpldfit writes `.T = Gnd` (the register holds); active -> `.T = Vcc` (it toggles).
             * A PT-clocked register and a receiver keep the default line. */
            int cptinv_true = m->dev_is_xl ? (m->devbit[22] == 1)
                                            : (m->devbit[22] == 0);
            fprintf(f, "%s", cptinv_true ? "'1'" : "'0'");
        } else if (n == 0) {
            fprintf(f, "'1'");
        } else if (m->d_inverted) {
            /* CPTINV=0 -- the D/T input is inverted. Expressed as a
             * `nand`-style form rather than `not (sum)`, so the fitter can
             * absorb it back into the macrocell's own inversion instead of
             * allocating a separate inverter. Logically identical. */
            fprintf(f, "not (");
            emit_data_expr(f, v, m, m->data_pts, n, 13);
            fprintf(f, ")");
        } else {
            emit_data_expr(f, v, m, m->data_pts, n,
                          (int)(strlen(inner_indent) + strlen(qname) + 14));
        }
        fprintf(f, ");\n");
    } else {
        /* D-FF: q <= sum-of-products */
        int n = m->n_data_pts;
        /* see the T-FF arm above: with a PT clock, data_pts holds only the
         * D input terms, the clock term is separate. */
        if (m->clk_mode == CLK_MODE_PT && !m->has_clk_pt) n = 0;
        fprintf(f, "%s%s <= ", inner_indent, qname);
        if (n == 0 && m->clk_mode != CLK_MODE_PT && m->n_data_pts == 0
            && m->pre_export_n == 0 && m->export_role == 0
            && m->have_dev_bits && m->devbit[22] >= 0) {
            /* A GCK-clocked D register whose own terms are empty and which
             * is on no export bus has D = the empty OR through CPTINV -- the
             * same constant emit_data_expr() emits for a combinational
             * output. cpldfit writes `q.D = Gnd` when CPTINV is inactive by the
             * family rule (XL raw 0, non-XL raw 1) and `q.D = Vcc` when it is
             * active. A constant '1' on a D=Gnd register with an async preset
             * and INIT=S would let XST replace it by logic (Xst:2679) and
             * ngdbuild then refuses the INIT constraint on the vanished net.
             * A PT-clocked register and a register emptied by an export keep
             * the default line. */
            int cptinv_true = m->dev_is_xl ? (m->devbit[22] == 1)
                                            : (m->devbit[22] == 0);
            fprintf(f, "%s", cptinv_true ? "'1'" : "'0'");
        } else if (n == 0) {
            fprintf(f, "'1'");
        } else if (m->d_inverted) {
            /* CPTINV=0 -- the D/T input is inverted. Expressed as a
             * `nand`-style form rather than `not (sum)`, so the fitter can
             * absorb it back into the macrocell's own inversion instead of
             * allocating a separate inverter. Logically identical. */
            fprintf(f, "not (");
            emit_data_expr(f, v, m, m->data_pts, n, 13);
            fprintf(f, ")");
        } else {
            emit_data_expr(f, v, m, m->data_pts, n,
                          (int)(strlen(inner_indent) + strlen(qname) + 6));
        }
        fprintf(f, ";\n");
    }

    if (m->has_ce) fprintf(f, "      end if;  -- CE\n");
    fprintf(f, "    end if;\n");
    fprintf(f, "  end process;\n\n");
}

static void emit_processes(FILE *f, const vctx_t *v) {
    /* Group by clock for clarity */
    static const struct { int mode; const char *label; } groups[] = {
        { CLK_MODE_GCK1, "GCK1 RISE" }, { CLK_MODE_GCK1, "GCK1 FALL" },
        { CLK_MODE_GCK2, "GCK2 RISE" }, { CLK_MODE_GCK2, "GCK2 FALL" },
        { CLK_MODE_GCK3, "GCK3 RISE" }, { CLK_MODE_GCK3, "GCK3 FALL" },
        { CLK_MODE_PT,   "PT-CLK"    }, { -1, NULL }
    };

    for (int g = 0; groups[g].mode >= 0; g++) {
        int fall_expected = (g % 2 == 1) && (groups[g].mode != CLK_MODE_PT);
        int found_any = 0;
        for (int i = 0; i < v->n_active; i++) {
            const mc_t *m = v->active[i];
            if (m->is_combi) continue;
            if (m->clk_mode != groups[g].mode) continue;
            if (groups[g].mode != CLK_MODE_PT && m->clk_falling != fall_expected) continue;
            if (!found_any) {
                fprintf(f,
                    "  -- ==================================================\n"
                    "  -- %s\n"
                    "  -- ==================================================\n\n",
                    groups[g].label);
                found_any = 1;
            }
            emit_one_process(f, v, m);
        }
    }
}

static void emit_oe_logic(FILE *f, const vctx_t *v) {
    int any = 0;
    for (int pin = 1; pin < MAX_PINS; pin++) {
        const mc_t *m = v->out_mc[pin];
        if (!m || m->is_combi) continue;
        if (!any) {
            fprintf(f,
                "  -- ==================================================\n"
                "  -- Output-enable logic\n"
                "  -- ==================================================\n\n");
            any = 1;
        }

        char qname[32]; mc_qname(v->dev, m->fb, m->mc, qname, sizeof qname);
        char lbl[24];   mc_label(m->fb, m->mc, lbl, sizeof lbl);
        char psg[16];   pin_sig(pin, psg, sizeof psg);

        switch (m->oe_mode) {
        case 1:  /* always driven */
            fprintf(f, "  %s <= %s;\n", psg, qname);
            break;
        /* An XL global enable can be inverted (COEINV beside OE0=1;
         * decode.c's oe_gts_invert): the pad is driven while GTSn is LOW.
         * GTS3/GTS4 are XC95144(XL)/216/288(XL). */
        case 2:  /* GOE1 (GTS1) */
        case 4:  /* GOE3 (GTS3) */
        case 5:  /* GOE4 (GTS4) */
            {   int n = m->oe_mode == 2 ? 1 : m->oe_mode == 4 ? 3 : 4, lo = gts_active_low(v, m, n);
                fprintf(f, "  %s <= %s when gts%d = '%c' else 'Z';  -- GOE%d%s%s\n", psg, qname, v->gts_src[n], lo ? '0' : '1', n,
                        m->oe_gts_invert ? " (active low: COEINV=1)" : "", gts_note(v, n)); }
            break;
        case 3:
            if (m->has_oe_pt) {
                /* PT-OE */
                fprintf(f, "  %s <= %s when ", psg, qname);
                if (pt_is_complex(&m->oe_pt))
                    fprintf(f, "%s_oe", lbl);
                else
                    emit_pt_expr(f, v, m->fb, &m->oe_pt);
                /* XL COL_OE_INVERT is an OE polarity bit (decode.c's
                 * oe_pt_invert). cpldfit writes the inverted form
                 * `!sig.OE = <term>`, i.e. the pad is driven while the term
                 * is FALSE. Compare against '0' there. */
                fprintf(f, " = '%c' else 'Z';  -- PT-OE%s\n",
                        m->oe_pt_invert ? '0' : '1',
                        m->oe_pt_invert ? " (active low: COL_OE_INVERT=1)" : "");
            } else {
                /* GOE2 (GTS2); XL polarity as above */
                fprintf(f, "  %s <= %s when gts%d = '%c' else 'Z';  -- GOE2%s%s\n", psg, qname, v->gts_src[2],
                        gts_active_low(v, m, 2) ? '0' : '1', m->oe_gts_invert ? " (active low: COEINV=1)" : "", gts_note(v, 2));
            }
            break;
        default: /* tristate: pin permanently 'Z' -- should not happen for an output */
            fprintf(f, "  %s <= 'Z';  -- NOTE: oe_mode=tristate\n", psg);
            break;
        }
    }
    if (any) fprintf(f, "\n");
}

static void emit_combinatorial(FILE *f, const vctx_t *v) {
    int any = 0;
    for (int i = 0; i < v->n_active; i++) {
        const mc_t *m = v->active[i];
        if (!m->is_combi || !drives_pad(m)) continue;
        if (!any) {
            fprintf(f,
                "  -- ==================================================\n"
                "  -- Combinatorial outputs\n"
                "  -- ==================================================\n\n");
            any = 1;
        }
        char csg[16]; pin_sig(m->pin, csg, sizeof csg);
        fprintf(f, "  -- FB%d_MC%d\n  c_%s <= ", m->fb, m->mc, csg);
        /* CPTINV-driven source inversion: same `not (...)` wrap as the
         * registered emitter (emit_registered_process, T-FF/D-FF arms).
         * Skipped when emit_data_expr()'s OE rule fires
         * (oe_honest_rule_fires()): that rule already reads the same
         * m->devbit[22] CPTINV bit and puts its polarity into the literal it
         * picks, so a second `not (...)` would apply the bit twice. */
        if (m->d_inverted && !oe_honest_rule_fires(m, m->n_data_pts)) {
            fprintf(f, "not (");
            emit_data_expr(f, v, m, m->data_pts, m->n_data_pts, 15);
            fprintf(f, ")");
        } else {
            emit_data_expr(f, v, m, m->data_pts, m->n_data_pts, 11);
        }
        fprintf(f, ";\n");
        /* A combinatorial output with a product-term OE is an open-drain
         * driver -- cpldfit writes it `od_pin = Gnd; od_pin.OE = en;` and
         * every consumer reads `od_pin.PIN`, i.e. the pin is LOW when enabled
         * and high-Z (pulled high) otherwise. Emit the tri-state; driving the
         * port unconditionally from c_ would throw the OE away. */
        if (m->has_oe_pt && m->oe_mode == 3) {
            char oeb[96]; int k;
            fprintf(f, "  %s <= c_%s when (", csg, csg);
            for (k = 0; k < m->oe_pt.n_lits; k++) {
                char raw[SIGNAME_MAX], ex[LITEXPR_MAX];
                pair_signal_name(v->dev, v->chip->fuses, m->fb,
                                 m->oe_pt.lits[k].pair,
                                 m->oe_pt.lits[k].is_comp, raw, sizeof raw);
                identity_override_c(v->chip, v->dev, m->fb,
                                    m->oe_pt.lits[k].pair,
                                    m->oe_pt.lits[k].is_comp, raw, sizeof raw);
                lit_to_expr(v, raw, ex, sizeof ex);
                if (k) fprintf(f, " and ");
                fprintf(f, "%s", ex);
            }
            (void)oeb;
            /* same OE polarity bit as the registered path above; an
             * open-drain pad whose enable cpldfit writes as
             * `!sig.OE = <term>` is driven while the term is FALSE. */
            fprintf(f, ") = '%c' else 'Z';%s\n\n",
                    m->oe_pt_invert ? '0' : '1',
                    m->oe_pt_invert ? "  -- active low: COL_OE_INVERT=1" : "");
        } else if (m->oe_mode == 2 || m->oe_mode == 4 || m->oe_mode == 5 ||
                   (m->oe_mode == 3 && !m->has_oe_pt)) {
            /* oe_mode==3 is GOE2 *or* PT-OE -- `has_oe_pt` disambiguates
             * (see decode.c). */
            /* A global output enable. cpldfit reports oe=2(GOE1) or oe=4(GOE2):
             * the pad is driven only while that global pin is asserted. Driven
             * unconditionally, the rebuild would refit the macrocell as
             * oe=1(always-on). */
            /* four global enables on XC95144/216/288, not two. */
            /* XL global-enable polarity (oe_gts_invert). */
            {   int n = (m->oe_mode == 2) ? 1 : (m->oe_mode == 4) ? 3 : (m->oe_mode == 5) ? 4 : 2;
                const char *nt = gts_note(v, n);   /* a VHDL comment: "  --" first unless the COEINV comment already opened one */
                fprintf(f, "  %s <= c_%s when gts%d = '%c' else 'Z';%s%s%s\n\n",
                        csg, csg, v->gts_src[n], gts_active_low(v, m, n) ? '0' : '1',
                        m->oe_gts_invert ? "  -- active low: COEINV=1" : "", (*nt && !m->oe_gts_invert) ? "  --" : "", nt); }
        } else {
            fprintf(f, "  %s <= c_%s;\n\n", csg, csg);
        }
    }

    /* buried combinatorial nodes, same shape but assigned to their
     * mc_label() signal since they have no pin. */
    for (int i = 0; i < v->n_active; i++) {
        const mc_t *m = v->active[i];
        /* A buried combinatorial macrocell with zero product terms is
             * still a real node other logic reads. Left undriven, every
             * consumer would read an undefined signal. Emit it -- its empty
             * sum-of-products is '0', and a driven '0' is well-formed where an
             * undriven signal is not. Emit only when something reads it. */
            if (!m->is_combi || drives_pad(m)) continue;
            if (m->n_data_pts <= 0 && !mc_is_referenced(v->chip, v->dev, m->fb, m->mc))
                continue;
        if (!any) {
            fprintf(f,
                "  -- ==================================================\n"
                "  -- Combinatorial outputs\n"
                "  -- ==================================================\n\n");
            any = 1;
        }
        char lbl[24]; mc_label(m->fb, m->mc, lbl, sizeof lbl);
        /* A buried open-drain node (combinational, PT-OE, zero data terms)
         * must be a real tri-state, not a constant, as for the pinned form.
         * A constant '0' would be removed by the fitter and every term reading
         * the node would be constant false. 'Z' gives the fitter a real
         * open-drain net to route. */
        if (m->has_oe_pt && m->oe_mode == 3 && m->n_data_pts <= 0) {
            int k;
            fprintf(f, "  -- FB%d_MC%d (buried, open drain)\n  %s <= '0' when (",
                    m->fb, m->mc, lbl);
            for (k = 0; k < m->oe_pt.n_lits; k++) {
                char raw[SIGNAME_MAX], ex[LITEXPR_MAX];
                pair_signal_name(v->dev, v->chip->fuses, m->fb,
                                 m->oe_pt.lits[k].pair,
                                 m->oe_pt.lits[k].is_comp, raw, sizeof raw);
                identity_override_c(v->chip, v->dev, m->fb,
                                    m->oe_pt.lits[k].pair,
                                    m->oe_pt.lits[k].is_comp, raw, sizeof raw);
                lit_to_expr(v, raw, ex, sizeof ex);
                if (k) fprintf(f, " and ");
                fprintf(f, "%s", ex);
            }
            /* same OE polarity bit; written for correctness, the usual
             * case for this polarity is a pinned macrocell. */
            fprintf(f, ") = '%c' else 'Z';%s\n\n",
                    m->oe_pt_invert ? '0' : '1',
                    m->oe_pt_invert ? "  -- active low: COL_OE_INVERT=1" : "");
            continue;
        }
        fprintf(f, "  -- FB%d_MC%d (buried)\n  %s <= ", m->fb, m->mc, lbl);
        /* same d_inverted handling and the same oe_honest_rule_fires()
         * exclusion as the pinned form above: a buried combinational node is
         * the same emit_data_expr() call. */
        if (m->d_inverted && !oe_honest_rule_fires(m, m->n_data_pts)) {
            fprintf(f, "not (");
            emit_data_expr(f, v, m, m->data_pts, m->n_data_pts, 15);
            fprintf(f, ")");
        } else {
            emit_data_expr(f, v, m, m->data_pts, m->n_data_pts, 11);
        }
        fprintf(f, ";\n\n");
    }
}

/* ===========================================================================
 * 5  UCF emission
 * =========================================================================== */

/* The per-macrocell UCF attribute lines.
 *
 * PWR_MODE is taken from ORPWR alone, not from `m->pwr_low`. decode.c
 * computes the non-XL flag as
 *     m->pwr_low = !dev_attr(c26) || !dev_attr(c50)
 * -- ORPWR or SAPWR0. A constraint `INST ... PWR_MODE = LOW` writes, at the
 * active polarity of device.h's dev_attr_off():
 *     non-XL  STD: ORPWR and SAPWR0..4 all ACTIVE   LOW: all INACTIVE
 *     XL      STD: ORPWR and SAPWR0..4 all ACTIVE   LOW: all INACTIVE
 * so the feature's own bit is ORPWR; the SAPWR* bits follow the
 * macrocell's product-term slot usage rather than the power mode. A
 * macrocell can read ORPWR active (STD) with SAPWR0 inactive, and must then
 * stay STD. */
static void emit_mc_attrs(FILE *fu, const vctx_t *v, const mc_t *m,
                          const char *net) {
    const device_t *dev = v->dev;
    int orpwr = m->devbit[COL_PWR_LOW];      /* raw; -1 = never read */
    if (orpwr >= 0) {
        unsigned char raw = (unsigned char)orpwr;
        if (!dev_attr_off(dev, &raw, 0))     /* ORPWR inactive == LOW power */
            fprintf(fu, "NET \"%s\" PWR_MODE=LOW;\n", net);
    }
    /* INIT: the vendor scopes it to registers and latches, so a combinational
     * macrocell's PRESET fuse has nothing to attach to. */
    if (m->init_set && !m->is_combi)
        fprintf(fu, "NET \"%s\" INIT=S;\n", net);
}

static void emit_ucf(FILE *fu, const vctx_t *v, const char *entity, const char *sigtxt) {
    const device_t *dev = v->dev;
    fprintf(fu, "# UCF constraints for %s (%s)\n", entity, dev->id);
    fprintf(fu, "# user signature: %s\n", sigtxt);
    char usg[16], ulc[12];   /* scratch for pin_sig()/pin_loc() */

    for (int g = 1; g <= 3; g++) {
        if (!(v->gck_used & (1 << (g-1)))) continue;
        if (v->gck_alias[g][0]) continue;   /* no port, so no LOC */
        int pin = gck_pin_num(dev, g);
        if (pin) { pin_loc(pin, ulc, sizeof ulc);
                   fprintf(fu, "NET \"gck%d\" LOC = \"%s\";\n", g, ulc); }
    }
    {   /* LOC every unresolved-literal port, and warn once per literal so
         * the reader knows a real device pin was used to preserve structure
         * that would otherwise have been optimised away. */
        int ui;
        for (ui = 0; ui < v->n_unres; ui++) {
            fprintf(fu, "NET \"%s\" LOC = \"P%d\";\n",
                    v->unres[ui], v->unres_pin[ui]);
            fprintf(stderr,
                    "WARNING: %s could not be identified.\n"
                    "         One FastCONNECT input of this chip feeds a product term, but the\n"
                    "         JEDEC file does not say which macrocell drives it, so its name is\n"
                    "         unknown. It has been given its own input pin (P%d) instead of being\n"
                    "         left undriven -- otherwise the ISE fitter treats it as a constant,\n"
                    "         deletes it, and rebuilds the surrounding logic differently.\n"
                    "         COST: pin P%d is used by the rebuild but not by the original chip.\n"
                    "         Use -noportfix to keep the old behaviour instead.\n",
                    v->unres[ui], v->unres_pin[ui], v->unres_pin[ui]);
        }
    }
    for (int g = 1; g <= 4; g++) {   /* GTS3/GTS4 LOC'd too */
        if (!(v->gts_used & (1 << (g-1)))) continue;
        int pin = gts_pin_num(dev, g);
        if (pin) { pin_loc(pin, ulc, sizeof ulc);
                   fprintf(fu, "NET \"gts%d\" LOC = \"%s\";\n", g, ulc); }
    }
    if (v->gsr_used) {
        int pin = gsr_pin_num(dev);
        if (pin) { pin_loc(pin, ulc, sizeof ulc);
                   fprintf(fu, "NET \"gsr\" LOC = \"%s\";\n", ulc); }
    }
    for (int pin = 1; pin < MAX_PINS; pin++) {
        if (v->input_pin[pin]) {
            pin_sig(pin, usg, sizeof usg); pin_loc(pin, ulc, sizeof ulc);
            fprintf(fu, "NET \"%s\" LOC = \"%s\";\n", usg, ulc);
        }
    }
    for (int pin = 1; pin < MAX_PINS; pin++) {
        const mc_t *m = v->out_mc[pin];
        if (!m) continue;
        pin_sig(pin, usg, sizeof usg); pin_loc(pin, ulc, sizeof ulc);
        fprintf(fu, "NET \"%s\" LOC = \"%s\";\n", usg, ulc);
        if (m->slew_slow) fprintf(fu, "NET \"%s\" SLEW = SLOW;\n", usg);
    }

    /* -- The two electrical/metadata fields that have a documented
     * per-element UCF form on this family.
     *
     * Vendor syntax (ISE CPLD Attributes page):
     *   PWR_MODE  "UCF: NET signal_name PWR_MODE=value;  Legal values: LOW,
     *              STD (default) ... Families: XC9500/XL/XV ... When attached
     *              to a net, PWR_MODE attaches to all applicable elements that
     *              drive the net."
     *   INIT      "UCF: NET signal_name INIT=value;  Legal values: S, R
     *              (default) ... Families: All CPLD ... You can attach INIT to
     *              a register or latch instance, to a net or signal driven by
     *              a register or latch."
     * PWR_MODE=LOW on one macrocell moves exactly its 5 power fuses; INIT=S
     * moves exactly 1 (PRESET); on both families.
     *
     * A line is emitted only for a net this function has emitted: ngdbuild
     * does not warn on an unassociated constraint, it fails --
     *   ERROR:NgdBuild:981 - Could not find any associations for the
     *   following constraint: '<NET "n_p42" INIT=S;>'
     * (e.g. a permanently tristated macrocell whose pin is dropped). The
     * emitted set is known by construction: out_mc[] for pads, v->buried[]
     * for internal nodes.
     *
     * Buried nodes are included: they are declared as `fb<N>mc<M>` carrying
     * `attribute keep = "true"`, and ngdbuild accepts `NET "fb1mc18" INIT=S;`.
     *
     * INIT only for a registered macrocell: a combinational one has no
     * register for the attribute to attach to, which is the vendor's own
     * scope. */
    for (int pin = 1; pin < MAX_PINS; pin++) {
        const mc_t *m = v->out_mc[pin];
        if (!m) continue;
        pin_sig(pin, usg, sizeof usg);
        emit_mc_attrs(fu, v, m, usg);
    }
    for (int i = 0; i < v->n_buried; i++) {
        const mc_t *m = v->buried[i];
        if (!m) continue;
        mc_label(m->fb, m->mc, usg, sizeof usg);
        emit_mc_attrs(fu, v, m, usg);
    }
}

/* -- The build-options file ----------------------------------------------
 *
 * Three fields have no per-element UCF form: the user signature (a
 * programming-time field), the programmable ground (`cpldfit -unused
 * ground`, global) and the XL keeper (`cpldfit -terminate`, global). The
 * emitter writes them to a file the build reads, to compose cpldfit's and
 * hprep6's command lines.
 *
 * Format: one `key value` line, `#` comments, no quoting; a shell
 * `while read k vrest` parses it.
 *
 *   signature_printable 0|1    1 when all four bytes are hprep6-legal text
 *   signature_ascii     <4ch>  present only when signature_printable is 1
 *   signature_hex       <8 hex digits>   always -- all 32 bits, MSBs included
 *   cpldfit_unused      ground|float
 *   cpldfit_terminate   keeper|float     XL only; absent on non-XL
 *
 * signature_hex is needed because the field is 8 bits per character and
 * `hprep6 -n` writes bytes >= 0x80, so a 4-character ASCII string cannot
 * express every value the device can hold. */
static void emit_opt(FILE *fo, const vctx_t *v, const JedFile *jf,
                     const char *entity) {
    const device_t *dev = v->dev;
    unsigned char sb[4];
    int i, fb, mc, ground = 0;

    fprintf(fo, "# xc9500dump build options for %s (%s)\n", entity, dev->id);
    fprintf(fo, "# One `key value` line per directive; '#' begins a comment.\n");

    if (chip_signature(jf, dev, sb)) {
        int printable = 1;
        for (i = 0; i < 4; i++) {
            int c = sb[i];
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
                  (c >= 'a' && c <= 'z') || c == ' ' || c == '_' ||
                  c == '-' || c == '.')) { printable = 0; break; }
        }
        fprintf(fo, "signature_printable %d\n", printable);
        if (printable)
            fprintf(fo, "signature_ascii %c%c%c%c\n", sb[0], sb[1], sb[2], sb[3]);
        fprintf(fo, "signature_hex %02X%02X%02X%02X\n", sb[0], sb[1], sb[2], sb[3]);
    } else {
        fprintf(fo, "signature_printable 0\n");
    }

    /* CGND is per macrocell in the fuse map and global in the fitter: the only
     * control is `-unused ground|float`, so one active macrocell decides it.
     * ACTIVE is device.h's dev_attr_off() polarity -- non-XL raw 0, XL raw 1.
     * `-unused float` leaves every CGND fuse inactive. */
    /* `m->cgnd` is assigned only on non-XL, so the raw bit is read from
     * devbit[] instead, which holds it on both families at the column the
     * vendor's name table gives: 48 non-XL, 43 XL. */
    {   int cgnd_col = dev->is_xl ? COL_XL_GROUND_PIN : 48;
        if (v->chip) {
            for (fb = 1; fb <= dev->n_fbs && fb <= 16 && !ground; fb++)
                for (mc = 1; mc <= MACROCELLS_PER_FB && !ground; mc++) {
                    int raw_i = v->chip->mc[fb-1][mc-1].devbit[cgnd_col];
                    if (raw_i < 0) continue;
                    unsigned char raw = (unsigned char)raw_i;
                    if (dev_attr_off(dev, &raw, 0)) ground = 1;
                }
        }
    }
    fprintf(fo, "cpldfit_unused %s\n", ground ? "ground" : "float");

    {   int k = chip_keeper(jf, dev);
        if (k >= 0) fprintf(fo, "cpldfit_terminate %s\n", k ? "keeper" : "float");
    }
}

/* ===========================================================================
 * 6  Entry point
 * =========================================================================== */

/* A pad whose only reader is a concurrent KEEP alias.
 * XST names such a pad's net after the KEEP alias (`c_n_p1 <= n_p89;`,
 * `fb1mc5 <= n_p40;`), so the UCF's `NET "<port>" LOC` names a net that no longer exists and NGDBUILD refuses
 * (ConstraintSystem:59) -- on an INOUT pad and on an INPUT whose alias is dead. The port object keeps
 * its name, so the pin is given as a VHDL attribute on the port and the UCF NET line is dropped, for exactly the ports
 * of that shape. A text pass over the finished emission: it changes no logic line. */
static int ca_is_ident(int c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_'; }
static int ca_count_reads(const char *body, const char *name) {
    int n = 0; size_t L = strlen(name);
    for (const char *q = body; (q = strstr(q, name)) != NULL; q += L) {
        if ((q > body && ca_is_ident((unsigned char)q[-1])) || ca_is_ident((unsigned char)q[L])) continue;
        const char *t = q + L; while (*t == ' ' || *t == '\t') t++;
        if (t[0] == '<' && t[1] == '=') continue;            /* an assignment target, not a read */
        n++;
    }
    return n;
}
static char *ca_slurp(FILE *f, size_t *len) {
    long n; char *b; rewind(f); fseek(f, 0, SEEK_END); n = ftell(f); rewind(f);
    b = malloc((size_t)n + 1); if (!b) return NULL;
    *len = fread(b, 1, (size_t)n, f); b[*len] = 0; return b;
}
static void alias_loc_fix_write(const char *v, const char *u, FILE *fv, FILE *fu) {
    enum { MAXP = 512 };
    char moved[MAXP][64], pins[MAXP][12]; int nm = 0;
    const char *body = strstr(v, "\nbegin\n"); const char *ent_end = strstr(v, "\nend entity");
    if (body && ent_end) {
        int depth = 0;
        for (const char *ln = body + 1; *ln; ) {
            const char *eol = strchr(ln, '\n'); size_t ll = eol ? (size_t)(eol - ln) : strlen(ln);
            char line[512]; if (ll >= sizeof line) ll = sizeof line - 1; memcpy(line, ln, ll); line[ll] = 0;
            char *t = line; while (*t == ' ') t++;
            if (!strncmp(t, "end process", 11)) depth--;
            else if (strstr(t, "process") == t || (strstr(t, ": process") && !strncmp(strstr(t, ": process"), ": process", 9))) depth++;
            else if (depth == 0) {
                char lhs[64], rhs[64]; int k = 0;
                if (sscanf(t, "%63[A-Za-z0-9_] <= %63[A-Za-z0-9_] ;%n", lhs, rhs, &k) == 2 && k > 0) {
                    char key[160], dec[96], loc[96];
                    snprintf(key, sizeof key, "attribute keep of %s ", lhs);
                    const char *kp = strstr(v, key);
                    snprintf(dec, sizeof dec, "\n    %s ", rhs);
                    const char *pd = strstr(v, dec);
                    int is_in = 0;
                    if (pd && pd < ent_end) { const char *c = strchr(pd + 1, ':');
                        if (c) { c++; while (*c == ' ') c++; is_in = !strncmp(c, "in ", 3) || !strncmp(c, "inout", 5); } }
                    snprintf(loc, sizeof loc, "NET \"%s\" LOC = \"", rhs);
                    const char *lp = u ? strstr(u, loc) : NULL;
                    if (kp && strstr(kp, "\"true\"") && is_in && lp && ca_count_reads(body, rhs) == 1 && nm < MAXP) {
                        const char *pv = lp + strlen(loc); size_t pl = strcspn(pv, "\"");
                        snprintf(moved[nm], sizeof moved[nm], "%s", rhs);
                        snprintf(pins[nm], sizeof pins[nm], "%.*s", (int)pl, pv); nm++;
                    }
                }
            }
            if (!eol) break;
            ln = eol + 1;
        }
    }
    if (nm == 0) { fputs(v, fv); if (fu && u) fputs(u, fu); return; }
    fwrite(v, 1, (size_t)(ent_end - v), fv);
    fprintf(fv, "\n  -- These pads' only reader is a KEEP alias, which takes the pad\n"
                "  -- net's name in XST's netlist; the pin is given on the PORT so NGDBUILD can place it\n");
    fprintf(fv, "  attribute LOC : string;\n");
    for (int i = 0; i < nm; i++) fprintf(fv, "  attribute LOC of %s : signal is \"%s\";\n", moved[i], pins[i]);
    fputs(ent_end, fv);
    if (fu && u) {
        for (const char *ln = u; *ln; ) {
            const char *eol = strchr(ln, '\n'); size_t ll = eol ? (size_t)(eol - ln + 1) : strlen(ln);
            int drop = -1;
            for (int i = 0; i < nm && drop < 0; i++) {
                char loc[96]; snprintf(loc, sizeof loc, "NET \"%s\" LOC = \"", moved[i]);
                if (!strncmp(ln, loc, strlen(loc))) drop = i;
            }
            if (drop >= 0) fprintf(fu, "# NET \"%s\" LOC = \"%s\" is a VHDL port attribute (cause C)\n", moved[drop], pins[drop]);
            else fwrite(ln, 1, ll, fu);
            if (!eol) break;
            ln = eol + 1;
        }
    }
}

int run_vhdl(const char *jed_path, const char *vhd_path, const char *ucf_path,
             const char *opt_path, const char *want_dev, const char *want_pkg) {
    JedFile jf;
    if (jed_load(jed_path, &jf) != 0) { fprintf(stderr,"vhdl: cannot load %s\n",jed_path); return 2; }

    /* The operator names the target; the tool reports what the file
     * claims beside it. This is how the operator states the real package. */
    const device_t *dev = NULL;
    if (want_dev || want_pkg) {
        int conflict = 0;
        dev = target_resolve(&jf, want_dev, want_pkg, &conflict);
        if (!dev) { jed_free(&jf); return 2; }
    }
    if (!dev) dev = (jf.device[0] ? device_find(jf.device) : NULL);
    /* Refuse a JED naming a device this tool does not support, as decode,
     * verify and coverage do (otherwise e.g. XC9999-15-PC44 would be emitted
     * with XC9536-PC44's tables). */
    if (!dev && !want_dev && device_named_but_unknown(jf.device)) {
        fprintf(stderr, "unsupported device \"%s\" (named in the JED header); "
                        "refusing to guess from fuse count\n", jf.device);
        jed_free(&jf);
        return 2;
    }
    if (!dev) dev = device_detect((int)jf.count);
    if (dev && !device_fits_jed(dev, jf.count)) {
        fprintf(stderr,
            "vhdl: %s expects %lu fuses, this JED has %lu.\n"
            "  Refusing -- decoding one device's fuse map with another's\n"
            "  geometry reads past the end of the data.\n",
            dev->id,
            (unsigned long)dev->n_fbs * (unsigned long)dev->jed_fuses_per_fb,
            (unsigned long)jf.count);
        jed_free(&jf); return 2;
    }
    if (!dev) {
        fprintf(stderr, "vhdl: unrecognised device (%d fuses)\n", (int)jf.count);
        jed_free(&jf);
        return 2;
    }

    /* Refuse to emit pin assignments from a guessed package.
     *
     * A JED that never names its package resolves to the generic "XCxxxx-"
     * row, whose pin table is one package's, chosen by default. A UCF from
     * it may name pins of a different package, and cpldfit then cannot place
     * the design with its global LOCs. A UCF is a physical claim about a
     * board; an error the user can act on is better than a file that looks
     * right.
     *
     * XC9500TOOL_PACKAGE=<PKG> overrides it when the package is known. */
    if (device_is_generic(dev)) {
        const char *want = getenv("XC9500TOOL_PACKAGE");
        const device_t *exact = NULL;
        if (want && *want) {
            char id[64];
            size_t n = strlen(dev->id);
            if (n && dev->id[n-1] == '-') {
                snprintf(id, sizeof id, "%s%s", dev->id, want);
                exact = device_find(id);
                /* device_find()'s last loop falls back to the generic "XCxxxx-"
                 * row, so it would "resolve" a package that does not exist
                 * (e.g. ZZ99). Demand a row that is package-specific, i.e. one
                 * whose id does not end in '-'. */
                if (exact) {
                    size_t el = strlen(exact->id);
                    if (!el || exact->id[el-1] == '-') exact = NULL;
                }
            }
            if (!exact) {
                fprintf(stderr,
                    "vhdl: XC9500TOOL_PACKAGE=\"%s\" is not a known package "
                    "for %s\n", want, dev->id);
                jed_free(&jf);
                return 2;
            }
            dev = exact;
        } else {
            /* The suggestion names --package (works on every platform) and
             * is taken from the device table, so it never advises a package
             * the device does not have. */
            {
                char devbase[32];
                char cand[256];
                const char *dash;
                size_t nb;
                int n;
                dash = strchr(dev->id, '-');
                nb = dash ? (size_t)(dash - dev->id) : strlen(dev->id);
                if (nb >= sizeof devbase) nb = sizeof devbase - 1;
                memcpy(devbase, dev->id, nb);
                devbase[nb] = '\0';
                /* names every candidate package, via the shared
                 * device_generic_candidates(). */
                n = device_generic_candidates(dev, cand, sizeof cand);
                fprintf(stderr,
                "vhdl: REFUSING to emit -- this JED never names its package.\n"
                "      Device resolved to %s, whose pin table is a DEFAULT and\n"
                "      may belong to a different package. Every LOC in the UCF\n"
                "      and every global pin would be a guess.\n"
                "      Name the package on the command line:\n"
                "          xc9500dump vhdl --device %s --package <PKG> \\\n"
                "              %s out.vhd out.ucf\n"
                "      Candidate package%s for this device: %s\n"
                "      If you do not know it, READ THE CHIP -- the\n"
                "      package code is in the part number printed on it\n"
                "      (e.g. XC95108 PQ100AEM -> PQ100).\n"
                "      `decode` refuses the same way for the same reason --\n"
                "      neither command prints a guessed pin table.\n",
                dev->id, devbase, jed_path,
                n == 1 ? "" : "s", n ? cand : "(none known for this family)");
            }
            jed_free(&jf);
            return 2;
        }
    }

    chip_t chip;
    decode_chip(&jf, dev, &chip);

    /* Derive entity name from JED filename (strip path and extension) */
    char entity[64] = "cpld_design";
    if (jed_path) {
        const char *base = strrchr(jed_path, '/');
        base = base ? base + 1 : jed_path;
        const char *dot = strrchr(base, '.');
        int len = dot ? (int)(dot - base) : (int)strlen(base);
        if (len > 0 && len < 60) {
            memcpy(entity, base, len);
            entity[len] = '\0';
            /* Make safe VHDL identifier: replace non-alnum with '_' */
            for (char *p = entity; *p; p++)
                if (!isalnum((unsigned char)*p)) *p = '_';
            /* Must start with a letter */
            if (!isalpha((unsigned char)entity[0])) {
                memmove(entity+1, entity, strlen(entity)+1);
                entity[0] = 'x';
            }
        }
    }

    /* A macrocell that does not drive its pad but carries logic is a
     * buried node whose pad happens to also be an input. Its own output is a
     * different signal from the pad, and the fuses distinguish them: the
     * identity path (select 30) selects the macrocell output, the FastCONNECT
     * path (select < 30) selects the pad. Reclassifying it as buried makes the
     * buried machinery emit it as an internal signal; the pad still becomes an
     * input port if any FastCONNECT literal reads it. */
    for (int rf = 1; rf <= chip.dev->n_fbs; rf++)
        for (int rm = 1; rm <= MACROCELLS_PER_FB; rm++) {
            mc_t *mm = &chip.mc[rf-1][rm-1];
            /* A register that does not drive its pad is a buried node
             * whether or not it has data terms: a zero-product-term macrocell
             * is real and readable (a T-FF with no toggle term holds). */
            if (mm->pin > 0 && mm->oe_mode == 0 && !mm->is_combi) {
                mm->tri_logic_pin = mm->pin;
                mm->pin = 0;
            }
        }
    g_reclass_chip = &chip;
    {   {
            }
        }

    /* Build context */
    vctx_t ctx;
    build_ctx(&ctx, &chip, entity);

    /* Open output files */
    FILE *fv_final = vhd_path ? fopen(vhd_path, "w") : stdout;
    FILE *fv = fv_final ? tmpfile() : NULL;   /* emit into a buffer, then the KEEP-alias pass */
    /* 3 = I/O failure, distinct from 2 = deliberate refusal. */
    if (!fv_final || !fv) { perror(vhd_path); jed_free(&jf); return 3; }
    FILE *fu_final = ucf_path ? fopen(ucf_path, "w") : NULL;
    FILE *fu = fu_final ? tmpfile() : NULL;
    if (ucf_path && !fu) perror(ucf_path);
    /* The build-options file. Optional, like the UCF -- `vhdl` with no
     * output paths prints the VHDL to stdout and creates no files. */
    FILE *fo = opt_path ? fopen(opt_path, "w") : NULL;
    if (opt_path && !fo) perror(opt_path);

    /* Emit VHDL */
    char sigtxt[96]; chip_signature_text(&jf, dev, sigtxt, sizeof sigtxt);
    emit_header(fv, &ctx, entity, sigtxt);
    /* A non-vendor value in a vendor control cell becomes a VHDL comment (nothing is written when there is none,
     * so a vendor-generated JED's emission is unchanged); the emitted logic is unchanged either way. */
    vendor_cells_report(dev, jf.fuses, (unsigned long)jf.count, fv, 1);
    emit_arch_signals(fv, &ctx, entity);
    emit_intermediates(fv, &ctx);
    emit_processes(fv, &ctx);
    emit_oe_logic(fv, &ctx);
    emit_combinatorial(fv, &ctx);
    fprintf(fv, "end architecture rtl;\n");

    /* Emit UCF */
    if (fu) emit_ucf(fu, &ctx, entity, sigtxt);
    {   size_t vl = 0, ul = 0; char *vb = ca_slurp(fv, &vl); char *ub = fu ? ca_slurp(fu, &ul) : NULL;
        alias_loc_fix_write(vb ? vb : "", ub, fv_final, fu_final);
        free(vb); free(ub); fclose(fv); if (fu) fclose(fu); }
    if (fu_final) fclose(fu_final);
    if (fo) { emit_opt(fo, &ctx, &jf, entity); fclose(fo); }
    if (fv_final != stdout) fclose(fv_final);
    jed_free(&jf);
    return 0;
}
