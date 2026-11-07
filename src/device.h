#ifndef DEVICE_H
#define DEVICE_H

#include <stddef.h>  /* size_t */

/* XC9500 family device descriptors.
 * XC9500 (non-XL) and XC9500XL are both supported.
 * They differ in:
 *   - JED layout:    non-XL = FB-major; XL = band-major (interleaved)
 *   - JED default:   non-XL = F1 (1=default); XL = F0 (0=default)
 *   - Fuse polarity: non-XL = 0 activates attributes; XL = 1 activates
 *   - Attribute cols: some differ (T-FF, CLK MUX, INIT_SET, slew)
 *   - AND-array: non-XL uses fixed wired-OR crossbar; XL uses 54-input IMUX
 */

/* Universal page/column constants */
#define COLUMN_WIDTH         108   /* fuses per AND-array column */
#define COLUMN_FRONT_WIDTH    72   /* 9 groups x 8 bits */
#define COLUMN_TAIL_WIDTH     36   /* tail slots */
/* Single definition of the version, used by the banner and the .vhd
 * header. */
#define XC9500TOOL_VERSION    "v6.136"

#define MACROCELLS_PER_FB     18   /* always 18 MCs per FB */
/* Largest part in the family is XC95288/XC95288XL, 16 FBs. */
#define MAX_FBS               16

/* XC9500XL specific: 54 inputs per FB (FastCONNECT II IMUX) */
#define XL_INPUTS_PER_FB     54
#define XL_COLS_PER_FB       108   /* 54 pairs x 2 = 108 AND-array columns */
#define XL_FUSES_PER_FB      11664 /* 108 bands x 108 fuses/page */

/* XC9500 non-XL FC crossbar constants
 *
 *   - Each destination FB has 36 AND-array pair slots (columns 0-35 x 2 =
 *     cols 0-71).
 *   - Each pair slot is a wired-OR bus connecting up to FC_MAX_SOURCES
 *     macrocell home pins.
 *   - Source sets are per destination FB (pair 0 in FB1 != pair 0 in FB2).
 *   - When a pair has 2+ sources, a select-code fuse field in the same
 *     destination FB's own AND-array column encodes which source is active
 *     (one code per source, no two sources share a code). See
 *     pair_signal_name() in decode.c.
 *   - XC9572 non-XL: every destination FB sees signals from all 4 FBs,
 *     including its own feedback.
 */
#define FC_PAIRS_PER_FB   36   /* pairs (= AND-array columns / 2) per dest FB */
#define FC_MAX_SOURCES    19   /* max wired-OR MC sources per pair slot (XC95288
                                   lists up to 19). A table entry with more
                                   would be truncated by the compiler; check
                                   the build for "excess elements" when a
                                   table is added. */
#define FC_MAX_SELECT_BITS 5   /* max select-code fuse width. The vendor
                                   bitmap writer names the select bits
                                   MR<role>B0..MR<role>B4 (role = pair%3),
                                   MR0B0=(55,0) through MR2B4=(55,16).
                                   Entries that give 4 values leave the
                                   5th at zero. */

/* ---- One source MC in a FC pair's wired-OR set ----
 * select_code[0..n_select_bits-1] (from the owning fc_entry_t) is this
 * candidate's expected value at each of the entry's select_bits[] fuse
 * positions -- unused when the entry's n_sources == 1. */
typedef struct {
    signed char fb;   /* source function block (1-based); 0 = end-of-list sentinel */
    signed char mc;   /* source macrocell within FB (1-based) */
    unsigned char select_code[FC_MAX_SELECT_BITS];
} fc_source_t;

/* ---- One FC pair entry: multiple MC sources share the pair via wired-OR ----
 *
 * n_sources == 0: pair slot not characterised (unknown)
 * n_sources == 1: single hardwired source, no select code
 * n_sources >= 2: n_sources MC home pins wired-OR'd; select_bits[0..
 *                 n_select_bits-1] are absolute JED fuse indices within
 *                 this destination FB's own AND-array column. Read fuses[]
 *                 at each position and compare the pattern with each
 *                 src[i].select_code[]; a match identifies the active
 *                 source. No match (or fuses == NULL, or n_select_bits == 0)
 *                 means: do not guess -- report all candidates. See
 *                 pair_signal_name() in decode.c.
 *
 * Addressing: col = 55 + pair/3 within the destination FB's own
 * FUSES_PER_FB block, role-based row offsets by pair%3 -- the same column
 * the vendor bitmap writer computes. The vendor column numbering matches
 * the COL_* constants in xc9500.h (e.g. QSREN=49 = COL_SLEW_SLOW).
 */
typedef struct {
    int         n_sources;
    int         n_select_bits;
    int         select_bits[FC_MAX_SELECT_BITS];
    fc_source_t src[FC_MAX_SOURCES];
} fc_entry_t;

/* ---- XC9500XL IMUX source entry: input N -> possible signal source ----
 * For XL devices, each of the 54 FB inputs is selected from a small set
 * of possible sources via a 5-bit MUX. */
typedef struct {
    unsigned char mux_code; /* 5-bit code that selects this source */
    signed char   fb;       /* source FB (1-based); -1 = not an MC source */
    signed char   mc;       /* source MC (1-based); 0 = external pin only */
    int           pin;      /* physical pin (0 = internal-only/buried) */
} imux_entry_t;

/* ---- FC candidate-pool hint (diagnostic only, not a resolution) ----
 * From the vendor .chp architecture description files. Lists every
 * physical candidate that can reach a (dest_fb, pair) slot. Does not say
 * which one a design selects -- that needs the select code. Only meaningful
 * where fc_table has n_sources==0; n_candidates==0 means no hint data. */
#define FC_HINT_MAX_CANDIDATES 12  /* observed max across all 3 devices: 11 */
typedef struct {
    int n_candidates;
    struct { signed char fb; signed char mc; } candidates[FC_HINT_MAX_CANDIDATES];
} fc_hint_t;

/* ---- Device descriptor ---- */
typedef struct {
    const char    *id;               /* e.g. "XC9572XL-VQ44" */
    const char    *family;           /* "XC9500" or "XC9500XL" */
    int            is_xl;            /* 1 = XC9500XL (F0, band-major, 54 IMUX) */
    int            n_fbs;            /* number of function blocks */
    int            fc_pairs_per_fb;  /* non-XL: FC_PAIRS_PER_FB; XL: XL_INPUTS_PER_FB */
    int            jed_fuses_per_fb; /* total JED fuses per FB */

    /* Non-XL FC pair table: fc_table[dest_fb-1][pair] -> set of wired-OR sources.
     * Indexed as a C pointer-to-array: fc_table[i] is row i (dest_fb = i+1).
     * NULL for XL devices (use imux_table instead).
     *
     * XC9536 non-XL: both rows carry the same mapping (symmetric crossbar).
     * XC9572 non-XL: all 4 rows differ. */
    const fc_entry_t (*fc_table)[FC_PAIRS_PER_FB];

    /* XL IMUX table */
    const imux_entry_t *const *imux_table;  /* [n_fbs*54] slot->source arrays */
    const int           *imux_count;        /* [n_fbs*54] counts */

    /* mc_pin[(fb-1)*18 + mc-1] = physical pin (0=buried). NULL if unverified. */
    const int        *mc_pin;

    /* Candidate-pool hints for unresolved fc_table slots (diagnostic only --
     * see fc_hint_t above). NULL if no hint data exists for this device.
     * Kept as the last field: the device table rows use positional
     * initialisation. */
    const fc_hint_t (*fc_hints)[FC_PAIRS_PER_FB];
} device_t;

/* ---- Device lookup ---- */
const device_t *device_detect(int total_fuses);
/* 1 if this device's total fuse count equals the JED's. */
int device_fits_jed(const device_t *dev, unsigned long jed_fuses);
/* see fc_lfb_missing() in device.c. */
int fc_lfb_missing(const device_t *dev, int fb, int pair, int *out);
/* defined in device.c, used in decode.c */
int xl_imux2_lookup(const device_t *dev, int input_n, int code,
                    int *out_fb, int *out_mc, int *out_is_fb);
int device_named_but_unknown(const char *name);
int device_is_generic(const device_t *d);
int device_generic_candidates(const device_t *dev, char *buf, size_t buflen);
const device_t *device_nth(int i);
const device_t *device_find(const char *name);
void device_list(void);

/* ---- Fuse addressing helpers ---- */

static inline int mc_off(int mc) {
    return (mc <= 9) ? 8*(mc-1)+6 : 8*(mc-10)+7;
}

static inline int pt_slot_offset(int r, int K, int pt_func) {
    static const int r2_t_base[] = {6, 12, 18, 24, 0};  /* XC9500 non-XL */
    if (r == 0) return pt_func * 8 + K;
    if (r == 1) return (pt_func + 5) * 8 + K;
    return 72 + r2_t_base[pt_func] + K;
}

static inline int pt_slot_offset_xl(int r, int K, int pt_func) {
    static const int r2_t_base_xl[] = {8, 14, 20, 26, 2};
    if (r == 0) return pt_func * 8 + K;
    if (r == 1) return (pt_func + 5) * 8 + K;
    return 72 + r2_t_base_xl[pt_func] + K;
}

static inline const unsigned char *
device_col(const device_t *dev, const unsigned char *fuses, int fb, int col) {
    if (dev->is_xl) {
        return fuses + (size_t)col * dev->n_fbs * COLUMN_WIDTH
                     + (size_t)(fb-1) * COLUMN_WIDTH;
    } else {
        return fuses + (size_t)(fb-1) * dev->jed_fuses_per_fb
                     + (size_t)col * COLUMN_WIDTH;
    }
}

static inline int dev_attr(const device_t *dev,
                            const unsigned char *col_108, int mc) {
    int raw = col_108[mc_off(mc)];
    return dev->is_xl ? (raw == 1) : (raw == 0);
}

/* xl_attr_mc_off: attribute byte position of a macrocell inside its
 * attribute band, as an absolute position (0-215); callers derive the page
 * from it (>=108 means page 2, offset -= 108).
 *
 * Every XL density (n_fbs 2, 4, 8, 16) uses one closed form:
 *     6 + 8*n_fbs*r + 8*(fb-1) + 24*n_fbs*(K%3) + (K/3)
 * with r = (mc-1)%3 and K = (mc-1)/3. The FB term is a flat 8 and does not
 * scale with n_fbs. The per-case positions after it are not reached by any
 * device in the table. */
static inline int xl_attr_mc_off(int n_fbs, int fb, int mc) {
    if (n_fbs == 2 || n_fbs == 4 || n_fbs == 8 || n_fbs == 16) {
        int r = (mc-1) % 3;
        int K = (mc-1) / 3;
        return 6 + 8*n_fbs*r + 8*(fb-1) + 24*n_fbs*(K%3) + (K/3);
    }
    int pair_base = ((fb-1)/2)*2;
    if (pair_base == 2 && n_fbs == 4) {
        /* FB3/FB4 at n_fbs==4: 22 + 32*r + 8*fb2 + 96*(K%3) + (K/3),
         * fb2=(fb-1)%2 (equal to the closed form above). */
        int r = (mc-1) % 3;
        int K = (mc-1) / 3;
        int fb2 = (fb-1) % 2;
        return 22 + 32*r + 8*fb2 + 96*(K%3) + (K/3);
    }
    if (pair_base == 2) {
        /* shared byte per K group */
        static const int tbl[2][2] = { {54, 55}, {62, 63} };
        int fb2     = (fb-1) % 2;        /* 0=FB3, 1=FB4 */
        int K       = (mc-1) / 3;
        int K_group = (K < 3) ? 0 : 1;   /* 0 for K=0..2, 1 for K=3..5 */
        return tbl[fb2][K_group];
    }
    int r = (mc-1) % 3;
    if (n_fbs < 8 && r == 1) {
        int fb2 = (fb-1) % 2;
        int K   = (mc-1) / 3;
        return 22 + fb2*8 + 48*(K%3) + (K/3);  /* may be >=108 -- page 2 */
    }
    /* r=0: 6 + fb2*8 + 24*n_fbs*(K%3) + (K/3), fb2=(fb-1)%2. */
    if ((n_fbs == 2 || n_fbs == 4) && r == 0) {
        int fb2 = (fb-1) % 2;
        int K = (mc-1) / 3;
        return 6 + fb2*8 + 24*n_fbs*(K%3) + (K/3);
    }
    /* MC18 (fb==2, r==2, K==5): fixed position per device. */
    if (fb == 2 && r == 2 && ((mc-1)/3) == 5) {
        if (n_fbs == 2) return 143;  /* page 2, offset 35 */
        if (n_fbs == 4) return 271;  /* page 3, offset 55 */
    }
    return mc_off(mc);  /* pair_base=0, r=0 (FB1, or FB2 on other n_fbs), or r=2 (all other K) */
}

/* dev_attr_off: read attribute using a pre-computed byte offset.
 * Use when xl_attr_mc_off(fb,mc) differs from mc_off(mc) (pair_base=2). */
static inline int dev_attr_off(const device_t *dev,
                                const unsigned char *col_108, int off) {
    int raw = col_108[off];
    return dev->is_xl ? (raw == 1) : (raw == 0);
}

static inline int col_is_idle(const device_t *dev, const unsigned char *col_108) {
    if (dev->is_xl) {
        for (int i = 0; i < COLUMN_WIDTH; i++)
            if (col_108[i]) return 0;
        return 1;
    }
    for (int g = 0; g < COLUMN_WIDTH; g++)        /* non-XL: every byte except the 18 mc_off rows */
        if ((g >= 72 || g % 8 < 6) && col_108[g] != 1) return 0;
    return 1;
}

static inline int fuse_connected(const unsigned char *col_108, int offset) {
    return col_108[offset] == 1;
}

/* Locate one product-term slot in the XL AND-array.
 *
 * Returns a half-band index (page_out, 1-based) and a byte offset inside it.
 * device_col() turns those into  fuses + col*n_fbs*108 + (page-1)*108, so the
 * absolute position inside a band is  band_off = (page-1)*108 + off.
 *
 * One formula for every density:
 *
 *   front rows 0..8 : band_off = row*8*n_fbs + (fb-1)*8 + K
 *   tail  rows 9..14: band_off = 72*n_fbs + (row-9)*6*n_fbs + (fb-1)*6 + K
 *   row = r*5 + pf ;  r=0 -> rows 0..4, r=1 -> rows 5..9, r=2 -> rows 10..14
 *   row order inside a group: PT3, PT5, PT4, PT1, PT2
 *
 * The per-case statements after each block are not reached. */
static inline void xl_and_loc(int n_fbs, int fb, int r, int K, int pf,
                               int *page_out, int *off_out) {
    int pair_base = ((fb - 1) / 2) * 2;
    int fb2       = (fb - 1) % 2;
    int abs;

    if (n_fbs >= 8) {
        int row = r * 5 + pf;
        int band_off = (row <= 8)
            ? row * 8 * n_fbs + (fb - 1) * 8 + K
            : 72 * n_fbs + (row - 9) * 6 * n_fbs + (fb - 1) * 6 + K;
        *page_out = band_off / COLUMN_WIDTH + 1;
        *off_out  = band_off % COLUMN_WIDTH;
        return;
    }

    if (pair_base == 0) {
        {
            int row = r * 5 + pf;
            int band_off = (row <= 8)
                ? row * 8 * n_fbs + (fb - 1) * 8 + K
                : 72 * n_fbs + (row - 9) * 6 * n_fbs + (fb - 1) * 6 + K;
            *page_out = band_off / COLUMN_WIDTH + 1;
            *off_out  = band_off % COLUMN_WIDTH;
            return;
        }
        if (r == 0) { *page_out = 1; *off_out = pf*16 + fb2*8 + K; return; }
        if (r == 1) {
            abs = (pf==4) ? (144+fb2*6+K) : (80+pf*16+fb2*8+K);
            if (abs<108) { *page_out=1; *off_out=abs; }
            else          { *page_out=2; *off_out=abs-108; }
            return;
        }
        *page_out=2; *off_out=48+pf*12+fb2*6+K-(pf>=3?1:0); return;
    }
    /* pair_base==2: XC9572XL FB3/FB4. */
    {
        int row = r*5 + pf;
        int band_off = (row <= 8)
            ? row * 8 * n_fbs + (fb - 1) * 8 + K
            : 72 * n_fbs + (row - 9) * 6 * n_fbs + (fb - 1) * 6 + K;
        *page_out = band_off / COLUMN_WIDTH + 1;
        *off_out  = band_off % COLUMN_WIDTH;
        return;
    }
    if (pf==2) { *page_out=4; *off_out=48+fb2*6+K; return; }
    if (1) {
        int row = r*5 + pf;
        int band_off = 72*n_fbs + (row-9)*6*n_fbs + (fb-1)*6 + K;
        *page_out = band_off / COLUMN_WIDTH + 1;
        *off_out  = band_off % COLUMN_WIDTH;
        return;
    }
    *page_out=4; *off_out=pf*12+fb2*6+K-(pf>=3?1:0);
}


#endif /* DEVICE_H */
