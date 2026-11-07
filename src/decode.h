#ifndef DECODE_H
#include <stdio.h>
#define DECODE_H

#include "jed.h"
#include "device.h"

#define MAX_PTS  36   /* max product terms per MC (generous upper bound) */
#define MAX_LITS 72   /* max literals per product term */

/* One literal in a product term */
typedef struct {
    int pair;      /* FC pair index (0..fc_pairs-1) */
    int is_comp;   /* 1=complement/inverted, 0=true/non-inverted */
} lit_t;

/* One product term */
typedef struct {
    int    col_offset;  /* offset within column (0..107) identifying the slot */
    int    is_tail;     /* 0=front section (offset<72), 1=tail (offset>=72) */
    lit_t  lits[MAX_LITS];
    int    n_lits;
    /* XL data slots only: the physical PT slot (0..4, CPT0..CPT4) this term
     * was read from; -1 for control PTs and on non-XL. Lets the PT-export
     * pass find which data_pts[] entry a (0,1)/EXP slot produced. */
    int    slot;
} pt_t;

/* Decoded macrocell */
typedef struct {
    /* Identity */
    int fb, mc;            /* 1-based */
    int pin;               /* physical pin; 0=buried */
    int K, r;              /* slot attribution: K=(mc-1)/3, r=(mc-1)%3 */

    /* Clock source */
    int clk_mode;          /* CLK_MODE_GCK1/2/3/COMB/PT */
    int clk_falling;       /* 0=rising, 1=falling (from col0 global invert) */
    int d_inverted;        /* CPTINV=0 -> the D/T input is inverted
                            * (cpldfit writes `!sig.D = <terms>`). */
    int clk_pt_col;        /* if CLK_MODE_PT: column of clock literal, else -1 */

    /* FF type */
    int is_toggle;         /* T-type FF (col40=0) */
    int is_combi;          /* combinatorial output (CLK_MODE_COMB) */
    int init_set;          /* initial state: 0=RESET, 1=SET (col43=0 -> SET) */

    /* Output */
    int oe_mode;           /* 0=tristate, 1=always-on, 2=GOE1, 3=GOE2/PT-OE */
    int pwr_low;           /* low power mode */
    int slew_slow;         /* slow slew rate */
    /* Non-XL per-macrocell control bits named by the vendor device file,
     * at column c, row mc_off(mc). Reported as read. */
    int oe4;               /* col 31  OE4    */
    int fboe0, fboe1;      /* col 44/45 FBOE0/FBOE1 */
    int fbinv;             /* col 46  FBINV  */
    int cgnd;              /* col 48  CGND   */
    int sapwr[4];          /* col 51..54 SAPWR1..4 (sense-amp power) */
    int dev_is_xl;         /* which .dev name table applies */
    int devbit[72];        /* every NO_MACROCELL-named column */
    int have_dev_bits;     /* set for non-XL only */

    /* Control functions detected */
    int has_reset;         /* async reset PT present */
    int has_preset;        /* async preset PT present */
    int gsr_reset;         /* FF is reset by the global GSR net */
    int gsr_set;           /* FF is set   by the global GSR net */
    int has_ce;            /* clock enable PT present (XL: PST slot of (r+2)%3 sibling) */
    int ce_const0;         /* XL: the CKEN pair (0,1) selects a clock enable but the shared
                            * PST/CE slot is not dedicated (CPT3 != SPC): the enable is constant Gnd
                            * (cpldfit writes `y.CE = Gnd;`) and the register never loads. has_ce stays 0. */
    int has_oe_pt;         /* product-term OE (oe_mode==3 AND OE slot occupied) */
    int oe_pt_invert;      /* XL COL_OE_INVERT, OE polarity. 1 => the pad is
                            * driven while the OE product term is FALSE
                            * (cpldfit writes `!sig.OE = <term>`); 0 => driven
                            * while TRUE. Only set together with has_oe_pt; an
                            * empty OE slot with COL_OE_INVERT=1 is still
                            * oe_mode=1, always on. Never set on non-XL, where
                            * column 30 is OE3, a different field. */
    int oe_gts_invert;     /* XL COEINV beside a global enable (OE0=1).
                            * 1 => the pad is driven while GTSn is LOW (cpldfit
                            * writes `!sig.OE = g; // GTS`). Only set on XL with
                            * oe_mode 2..5 and no PT-OE. */

    /* PT forwarding */
    int imports_from_lower; /* col24=0 */
    int exports_to_upper;   /* col25=0 */

    /* Product terms (separated by function) */
    pt_t data_pts[MAX_PTS];  /* D/T input PTs */
    int  n_data_pts;

    /* PTINV -- product-term controlled inversion. When slot 4
     * (COL_PT2_MUX_B1/B0, both asserted) is dedicated, its term is XORed
     * with the sum of the remaining data terms, not ORed into it. cpldfit
     * writes that with its `$` operator: q.D = p35 $ p50. Non-XL raw 1,1 and
     * XL raw 0,0 both mean dedicated; dev_attr_off() normalises the polarity. */
    int  has_ptinv;
    pt_t ptinv_pt;
    pt_t rst_pt;             /* async reset condition */
    pt_t pst_pt;             /* async preset condition */
    pt_t ce_pt;              /* clock enable condition (XL: stored at sibling PST slot) */
    pt_t oe_pt;              /* output enable condition (PT-OE mode) */
    /* A product-term clock lives in PT slot 0, the same slot that otherwise
     * carries the first data term. It is kept here, apart from data_pts, so
     * data_pts holds only data terms. has_clk_pt is set only when slot 0's
     * two-bit selector says the slot is dedicated to its control function
     * (the clock); clk_mode comes from the separate CCK columns. When it is 0
     * the clock term was not identified and the emitter falls back. */
    pt_t clk_pt;             /* product-term clock condition (CLK_MODE_PT) */
    int  tri_logic_pin;      /* pad number of a reclassified buried MC */
    int  has_clk_pt;         /* clk_pt holds a decoded, dedicated clock term */
    /* The PT-CLK selector is set but slot 0 holds no literal: the clock is a
     * constant (Gnd in the fitter's equations), not a product term, and is not
     * counted as one. "Not identified" stays for a non-empty, non-dedicated slot. */
    int  clk_slot_empty;

    int is_real_output;      /* oe_mode!=0 or is_toggle */

    /* XL only: this macrocell had a (0,1)/EXP slot that no neighbour's
     * CAS0/CAS1 claimed. Reported as unresolved, never merged into data_pts. */
    int has_unresolved_export;

    /* What the export pass saw. pre_export_n = data terms before any EXP
     * term was lifted; export_role = 1 donor, 2 receiver, 3 both, 0 neither.
     * emit_data_expr() uses them to recognise a truly empty macrocell (no
     * terms of its own, no bus) -- the fitter's constant output. */
    int pre_export_n;
    int export_role;
} mc_t;

/* Decoded full chip */
typedef struct {
    const device_t       *dev;
    const unsigned char  *fuses;   /* raw JED fuses (for XL IMUX readout) */
    /* mc[fb-1][mc-1], indexed by FB and MC within FB */
    mc_t mc[16][18]; /* up to 16 FBs, 18 MCs each (XC95288XL) */
} chip_t;

/* Decode the full chip from a JED file */
void decode_chip(const JedFile *jf, const device_t *dev, chip_t *chip);

/* The two chip-level fields a faithful rebuild must carry. Shared by
 * `decode` (prints the signature) and `vhdl` (hands it to the build).
 *
 * chip_signature: the 32-bit user signature, eight bits per character.
 *   hprep6 accepts bytes >= 0x80; the high bit of each byte is SI7/15/23/31.
 *   Returns 1 when all four bytes were in range, 0 otherwise.
 *
 * chip_keeper: the XL I/O bus-hold keeper, `cpldfit -terminate float|keeper`.
 *   One global fuse at device_col(dev,1,2) + xl_attr_mc_off(n,1,9)
 *   (566 / 1126 / 2246 / 4486 on the four XL densities). Its polarity is not
 *   dev_attr_off()'s: raw 0 = keeper (the default), raw 1 = float.
 *   Returns 1 = keeper, 0 = float, -1 = this family has no such field. */
int chip_signature(const JedFile *jf, const device_t *dev, unsigned char out[4]);
/* The one text of the signature that `decode`, the VHDL header and the UCF
 * header print: the raw 32 bits always, the ASCII form beside them only when
 * all four bytes are 0x20..0x7E, "none" for 0x00000000. Writes e.g.
 * `0x43323937 "C297"` into buf (n >= 96). */
void chip_signature_text(const JedFile *jf, const device_t *dev, char *buf, size_t n);
int chip_keeper(const JedFile *jf, const device_t *dev);

/* Print decoded chip (text summary) */
void decode_print(const chip_t *chip);

/* Wired-AND FastCONNECT pairs (non-XL). A pair can be driven by several
 * sources at once: the vendor's bitmap writer (SetUIMRowZeros) zeroes a UIM
 * line only when it is all-ones, so a second source on the same line leaves
 * the first source's bit standing. N survivors in an identity-plane row are
 * N members of a wired-AND. pair_signal_name() returns a composite name:
 *
 *     n_FB2MC15|n_FB2MC16          the AND of both
 *     NOT_n_FB2MC15|n_FB2MC16      the complement of the AND
 *
 * '|' is a member-list separator, not an OR (see or_expand_expr in vhdl.c).
 * Buffers holding these must be SIGNAME_MAX: a row can carry up to 107
 * survivors of 11 chars each ("n_FB16MC18|"). decode.c refuses to emit a
 * composite that does not fit and keeps the placeholder instead. */
#define SIGNAME_MAX  1280
#define LITEXPR_MAX  4096

/* Signal name for a given FB/MC */
const char *signal_name(const device_t *dev, int fb, int mc, char *buf, int buflen);
int pair_via_identity(const device_t *dev, const unsigned char *fuses,
                      int fb_dest, int pair, int *out_fb, int *out_mc);
const char *pair_signal_name(const device_t *dev, const unsigned char *fuses, int fb_dest, int pair, int is_comp, char *buf, int buflen);
/* a survivor of a multi-hit identity-plane row (a FastCONNECT wired-AND input) */
int mc_is_wired_and_member(const device_t *dev, const unsigned char *fuses, int f, int m);
void compute_fb_unique_unresolved_pairs(const device_t *dev, const chip_t *chip);


/* BGA pin encoding: 1000 + (row-'A'+1)*100 + col  (A1=1101, G7=1707).
 * Two column digits, so CS144 (up to 13) and BG256 (up to 20) balls do not
 * collide. The encoding is internal and never printed as a raw number. */
static inline int  pin_is_bga(int p)                       { return p >= 1000; }
/* Rows 27..32 are the two-letter BGA rows AA..AF (BG352: the vendor's ball
 * names run A..Y then AA..AF). */
static inline void bga_pin_name(int p, char *b, int sz)    {
    int c=p-1000, r=c/100;
    if (r <= 26) snprintf(b,sz,"ball_%c%d",(char)('A'+r-1),c%100);
    else         snprintf(b,sz,"ball_A%c%d",(char)('A'+r-27),c%100); }

/* A BGA pin is spelled ball_XN everywhere, also in pair_signal_name(), so
 * the decode report never carries two spellings for one pin (e.g. "n_p1022"
 * against the fitter's "ball_B2"). sig_to_pin() parses both. Defined here,
 * next to bga_pin_name(), so it precedes its first use in decode.c. */
static inline void pin_lit_name(const char *pol, int pin, char *b, int sz) {
    if (pin_is_bga(pin)) {
        char n[16];
        bga_pin_name(pin, n, sizeof n);
        snprintf(b, sz, "%s%s", pol, n);
    } else {
        snprintf(b, sz, "%sn_p%d", pol, pin);
    }
}


/* -noportfix: keep unresolved FastCONNECT literals as undriven stub signals
 * instead of allocating each a real input pin. */
extern int vhdl_no_port_fix;

#endif /* DECODE_H */
