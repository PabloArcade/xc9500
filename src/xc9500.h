#ifndef XC9500_H
#define XC9500_H

#include "jed.h"
#include "device.h"

/* ==========================================================================
 * Attribute column indices -- XC9500 (non-XL), confirmed by differential tests
 * ==========================================================================
 *
 * All attributes: read fb_bits[col * COLUMN_WIDTH + mc_off(mc)]
 * Non-XL polarity: 0 = feature ACTIVE, 1 = inactive (F1 default).
 * Use dev_attr(dev, col_ptr, mc) to get a family-independent 0/1 result.
 *
 * GCK GLOBAL INVERT (col=0, non-per-MC positions):
 *   offset 22 (mc_off MC3): GCK1 invert (0=falling)  [XC9536, confirmed]
 *   offset 30 (mc_off MC4): GCK2 invert (0=falling)  [confirmed]
 *   offset 38 (mc_off MC5): GCK3 invert (0=falling)  [confirmed]
 *
 * XC9500 non-XL CLK mode (3-bit at cols 35/36/37):
 *   (col35<<2)|(col36<<1)|col37: 6=GCK1, 7=GCK2, 5=GCK3, 3=COMB, 4=PT-CLK
 *
 * GCK pins (XC9536-PC44, confirmed): P5=GCK1, P6=GCK2, P7=GCK3
 * ========================================================================== */

/* PT MUX attribute columns (SAME for both families) */
#define COL_PT3_MUX_B1    12
#define COL_PT3_MUX_B0    13
#define COL_PT5_MUX_B1    14   /* OE/CE slot */
#define COL_PT5_MUX_B0    15
#define COL_PT4_MUX_B1    16   /* RESET slot */
#define COL_PT4_MUX_B0    17
#define COL_PT1_MUX_B1    18   /* PRESET slot */
#define COL_PT1_MUX_B0    19
#define COL_PT2_MUX_B1    20   /* 5th data slot (r=0) */
#define COL_PT2_MUX_B0    21
#define COL_INVERT_OUT    22   /* OR-gate output inversion (NOR synthesis) */
#define COL_FB_FROM_UPPER 23
#define COL_FB_FROM_LOWER 24
#define COL_FB_TO_UPPER   25
#define COL_PWR_LOW       26   /* non-XL: 1=LOW power; XL: "STD power" = 1=STD */
#define COL_OE_GTS        27   /* OE via GTS bit */
#define COL_OE_MUX_B0     28
#define COL_OE_MUX_B1     29
#define COL_OE_INVERT     30   /* OE invert (both families; fixed value on non-XL) */

/* XC9500 non-XL specific: CLK 3-bit at cols 35/36/37 */
#define COL_CLK_B2        35   /* XC9536: CLK mode bit-2 */
#define COL_CLK_B1        36   /* XC9536: CLK mode bit-1 */
#define COL_CLK_B0        37   /* XC9536: CLK mode bit-0 */
#define COL_IS_TOGGLE     40   /* XC9536: T-type FF (0=yes) */
#define COL_INIT_SET      43   /* XC9536: initial state (0=SET, 1=RST) */
/* Non-XL global set/reset enables. A reset mapped onto the GSR net sets
 * these columns; an ordinary per-macrocell reset uses COL_PT4_MUX_B1/B0
 * instead. Mirrors the XL pair COL_XL_RESET_GSR/COL_XL_SET_GSR, offset by
 * one. Active low, like every other non-XL attribute (dev_attr is raw==0). */
#define COL_GSR_RESET     41   /* reset via GSR (0=yes) */
#define COL_GSR_SET       42   /* set   via GSR (0=yes) */
#define COL_SLEW_SLOW     49   /* XC9536: slew rate (0=FAST, 1=SLOW) */
#define COL_PWR_LOW_ALT   50   /* XC9536: pwr_low alternate */
/* col45 is in use on XC9536. Columns 39/41/42/46/47/48 hold fixed values;
 * the bits at mc_off 41/42/46/48 are read, 39/47 are not. */

/* XC9500XL specific attribute columns (band N = col N-1)
 * XL polarity: 1 = feature ACTIVE (F0 default = 0 = inactive). */
#define COL_XL_BYPASS      32   /* band 33: bypass FF -> combinatorial output */
#define COL_XL_CLK_B1      33   /* band 34: CLK MUX bit-1 */
#define COL_XL_CLK_B0      34   /* band 35: CLK MUX bit-0 */
#define COL_XL_CLK_INVERT  35   /* band 36: per-MC CLK invert (1=falling edge) */
#define COL_XL_CE_OR_R     37   /* band 38: Clock Enable or Reset */
#define COL_XL_CE_OR_S     38   /* band 39: Clock Enable or Set */
#define COL_XL_IS_TOGGLE   39   /* band 40: T-type FF (1=yes) */
#define COL_XL_RESET_GSR   40   /* band 41: Reset via GSR (1=yes) */
#define COL_XL_SET_GSR     41   /* band 42: Set via GSR (1=yes) */
#define COL_XL_INIT_SET    42   /* band 43: preset FF with '1' = INIT=SET */
#define COL_XL_GROUND_PIN  43   /* band 44: ground unused pin */
#define COL_XL_FAST_SLEW   44   /* band 45: fast slew rate (1=FAST) */
#define COL_XL_PT3_STD_PWR 45   /* band 46: PT3 STD power */
#define COL_XL_PT5_STD_PWR 46   /* band 47: PT5 STD power */
#define COL_XL_PT4_STD_PWR 47   /* band 48: PT4 STD power */
#define COL_XL_PT1_STD_PWR 48   /* band 49: PT1 STD power */
#define COL_XL_PT2_STD_PWR 49   /* band 50: PT2 STD power */

/* XC9500XL CLK MUX 2-bit encoding at {COL_XL_CLK_B1, COL_XL_CLK_B0}:
 *   GCK1:   col33=0, col34=1 -> code 1
 *   GCK2:   col33=0, col34=0 -> code 0 (col35=1 for falling edge)
 *   GCK3:   col33=1, col34=0 -> code 2
 *   PT-CLK: col33=1, col34=1 -> code 3 */
#define XL_CLK_GCK2    0   /* {B1=0, B0=0} = GCK2 */
#define XL_CLK_GCK1    1   /* {B1=0, B0=1} = GCK1 */
#define XL_CLK_GCK3    2   /* {B1=1, B0=0} = GCK3 */
#define XL_CLK_PT      3   /* {B1=1, B0=1} = PT-CLK */

/* XC9500 non-XL CLK mode codes (3-bit, confirmed) */
#define CLK_MODE_GCK1   6   /* 0b110 */
#define CLK_MODE_GCK2   7   /* 0b111 */
#define CLK_MODE_GCK3   5   /* 0b101 */
#define CLK_MODE_COMB   3   /* 0b011 */
#define CLK_MODE_PT     4   /* 0b100 */

/* GCK pin numbers for XC9536-PC44 (confirmed) */
#define GCK1_PIN  5
#define GCK2_PIN  6
#define GCK3_PIN  7

/* GCK global invert offsets in col0 (non-XL only) */
#define GCK1_INVERT_OFF 22
#define GCK2_INVERT_OFF 30
#define GCK3_INVERT_OFF 38

/* ---- CLK mode decode ---- */

/* Non-XL: 3-bit code at cols 35/36/37 */
static inline int decode_clk_mode_non_xl(const unsigned char *col35,
                                          const unsigned char *col36,
                                          const unsigned char *col37, int mc) {
    int o = mc_off(mc);
    return (col35[o] << 2) | (col36[o] << 1) | col37[o];
}

/* XL: bypass bit at col32, 2-bit MUX at col33/34 */
static inline int decode_xl_clk_mux(const unsigned char *col33,
                                     const unsigned char *col34, int mc) {
    int o = mc_off(mc);
    return (col33[o] << 1) | col34[o];  /* {B1, B0} */
}

/* ---- OE mode decode (SAME columns for both families) ---- */
/* The global-enable selector is three bits (OE2,OE3,OE4). XC95144, XC95216
 * and XC95288 have GTS1-GTS4; the smaller parts only GTS1-GTS2:
 *
 *     (OE2,OE3,OE4)  0,0,1 -> GTS3      0,1,1 -> GTS1
 *                    1,1,0 -> GTS4      1,0,1 -> GTS2
 *
 * oe_mode 2 = GTS1, 3 = GTS2 or PT-OE, 4 = GTS3, 5 = GTS4. */
static inline int decode_oe_mode4(const unsigned char *col27,
                                   const unsigned char *col28,
                                   const unsigned char *col29,
                                   const unsigned char *col30,
                                   const unsigned char *col31, int mc) {
    int o = mc_off(mc);
    int b0 = col27[o], b1 = col28[o], b2 = col29[o];
    if (b0==1 && b1==0 && b2==1) return 1; /* always-on */
    if (b0==0 && b1==1) {
        int e3 = col30 ? col30[o] : 1;
        int e4 = col31 ? col31[o] : 1;
        /* (OE2,OE3,OE4) */
        if (b2==0 && e3==0 && e4==1) return 4; /* GTS3 */
        if (b2==1 && e3==1 && e4==0) return 5; /* GTS4 */
        if (b2==0) return 2;                   /* 011 -> GTS1 */
        return 3;                              /* 101 -> GTS2, or PT-OE */
    }
    return 0; /* tristate */
}

static inline int decode_oe_mode(const unsigned char *col27,
                                  const unsigned char *col28,
                                  const unsigned char *col29, int mc) {
    return decode_oe_mode4(col27, col28, col29, NULL, NULL, mc);
}

/* ---- GCK invert (non-XL only, col0 global bits) ---- */
static inline int gck_inverted(const unsigned char *col0, int gck_num) {
    int off = (gck_num==1) ? GCK1_INVERT_OFF :
              (gck_num==2) ? GCK2_INVERT_OFF : GCK3_INVERT_OFF;
    return (col0[off] == 0);
}

/* ---- Column idle (family-aware) ---- */
static inline int xc9500_col_is_idle(const device_t *dev, const unsigned char *col108) {
    return col_is_idle(dev, col108);
}

/* ---- Core commands ---- */
void xc9500_dump(const JedFile *jf, const device_t *dev);
void xc9500_diff(const JedFile *a, const JedFile *b);

#endif /* XC9500_H */
