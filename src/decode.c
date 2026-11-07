#include "decode.h"
#include "xc9500.h"
#include "device.h"
#include "jed.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ---- Signal naming ---- */

const char *signal_name(const device_t *dev, int fb, int mc,
                         char *buf, int buflen) {
    if (!dev || !dev->mc_pin) {
        snprintf(buf, buflen, "FB%d_MC%d", fb, mc);
        return buf;
    }
    int pin = dev->mc_pin[(fb-1)*18 + mc-1];
    if (pin == 0)          snprintf(buf, buflen, "n_FB%dMC%d", fb, mc);
    else if (pin_is_bga(pin)) bga_pin_name(pin, buf, buflen);
    else                   snprintf(buf, buflen, "n_p%d", pin);
    return buf;
}

/* Name for a signal coming from FC pair N (non-XL) or IMUX input N (XL) */

/* xl_imux_abyte: FastCONNECT II input-mux code for input_n (1..54) as seen by
 * destination FB fb_dest.
 *
 * The five mux fuses sit on the even A-rows 0,2,4,6,8. Odd A-rows 1 and 3
 * are often programmed but are not mux bits.
 *
 * The code is read from the destination FB's own page of the band. */
static int xl_imux_abyte(const device_t *dev, const unsigned char *fuses,
                          int fb_dest, int input_n) {
    int col  = (input_n <= 27) ? (49 + input_n) : (22 + input_n);
    int side = (input_n > 27) ? 7 : 6;
    /* A band is n_fbs*108 fuses. Its first 9 rows are n_fbs groups of 8:
     * six AND-array fuses then the A-bit and the B-bit for that function
     * block. So the five mux fuses for input_n as seen by fb_dest are
     * consecutive rows: stride 8*n_fbs, offset (fb-1)*8. */
    size_t base = (size_t)col * dev->n_fbs * COLUMN_WIDTH;
    size_t lim  = (size_t)dev->n_fbs * XL_FUSES_PER_FB;
    int code = 0;
    for (int bit = 0; bit < 5; bit++) {
        size_t o = base + (size_t)bit * 8 * dev->n_fbs
                        + (size_t)(fb_dest - 1) * 8 + side;
        if (o < lim && fuses[o]) code |= (1 << bit);
    }
    return code;
}

/* read_xl_abyte_pb2: thin wrapper over xl_imux_abyte, so the label on an
 * unresolved FB#_PAIR# placeholder reports the same code an imux_table
 * lookup for that fb_dest would use.
 * N is 0-based (pair index); xl_imux_abyte takes 1-based input_n. */
static int read_xl_abyte_pb2(const device_t *dev, const unsigned char *fuses,
                              int fb_dest, int N) {
    return xl_imux_abyte(dev, fuses, fb_dest, N + 1);
}





/* -------------------------------------------------------------------
 * XC9572XL A-byte codes, derived from the vendor .chp crossbar and indexed
 * by ISE imux slot.
 *
 * Where the table has no entry -- notably code 0, the unprogrammed default,
 * which the .chp does not describe -- the decoder emits a placeholder
 * instead of a name.
 */
static const unsigned char xl9572_derived[54][20][2] = {
  { { 0, 0},{ 2, 3},{ 2, 7},{ 1, 5},{ 1,14},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 1},{ 3, 1},{ 2, 1},{ 4, 1} },  /* slot 1 */
  { { 0, 0},{ 4,17},{ 4, 7},{ 4, 1},{ 3, 7},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 2},{ 3, 2},{ 2, 2},{ 4, 2} },  /* slot 2 */
  { { 0, 0},{ 2,18},{ 2,11},{ 1, 6},{ 1,10},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 3},{ 3, 3},{ 2, 3},{ 4, 3} },  /* slot 3 */
  { { 0, 0},{ 4,15},{ 4, 6},{ 3,16},{ 3, 6},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 4},{ 3, 4},{ 2, 4},{ 4, 4} },  /* slot 4 */
  { { 0, 0},{ 2, 4},{ 2,12},{ 1, 1},{ 1,15},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 5},{ 3, 5},{ 2, 5},{ 4, 5} },  /* slot 5 */
  { { 0, 0},{ 2, 1},{ 4,11},{ 3,13},{ 3,11},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 6},{ 3, 6},{ 2, 6},{ 4, 6} },  /* slot 6 */
  { { 0, 0},{ 2, 2},{ 2,13},{ 1, 8},{ 1,17},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 7},{ 3, 7},{ 2, 7},{ 4, 7} },  /* slot 7 */
  { { 0, 0},{ 4,16},{ 4, 4},{ 3,12},{ 3, 4},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 8},{ 3, 8},{ 2, 8},{ 4, 8} },  /* slot 8 */
  { { 0, 0},{ 2, 5},{ 2,14},{ 1, 3},{ 3, 2},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 9},{ 3, 9},{ 2, 9},{ 4, 9} },  /* slot 9 */
  { { 0, 0},{ 4,13},{ 4, 3},{ 3,10},{ 3, 3},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,10},{ 3,10},{ 2,10},{ 4,10} },  /* slot 10 */
  { { 0, 0},{ 2, 6},{ 2,16},{ 1, 4},{ 1,12},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,11},{ 3,11},{ 2,11},{ 4,11} },  /* slot 11 */
  { { 0, 0},{ 4,12},{ 4, 8},{ 3,18},{ 3, 9},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,12},{ 3,12},{ 2,12},{ 4,12} },  /* slot 12 */
  { { 0, 0},{ 2, 8},{ 2,15},{ 1, 9},{ 3, 5},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,13},{ 3,13},{ 2,13},{ 4,13} },  /* slot 13 */
  { { 0, 0},{ 4,10},{ 4, 5},{ 3,17},{ 3, 1},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,14},{ 3,14},{ 2,14},{ 4,14} },  /* slot 14 */
  { { 0, 0},{ 2, 9},{ 2,17},{ 1,11},{ 1,13},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,15},{ 3,15},{ 2,15},{ 4,15} },  /* slot 15 */
  { { 0, 0},{ 4,18},{ 4, 2},{ 3,15},{ 1,18},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,16},{ 3,16},{ 2,16},{ 4,16} },  /* slot 16 */
  { { 0, 0},{ 2,10},{ 1, 2},{ 1, 7},{ 3, 8},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,17},{ 3,17},{ 2,17},{ 4,17} },  /* slot 17 */
  { { 0, 0},{ 4,14},{ 4, 9},{ 3,14},{ 1,16},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,18},{ 3,18},{ 2,18},{ 4,18} },  /* slot 18 */
  { { 0, 0},{ 2, 4},{ 2,15},{ 3,16},{ 3, 9},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 5},{ 3, 5},{ 2, 4},{ 4, 2} },  /* slot 19 */
  { { 0, 0},{ 2, 2},{ 2,17},{ 1, 8},{ 3, 1},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,15},{ 3, 8},{ 2,14},{ 4, 6} },  /* slot 20 */
  { { 0, 0},{ 4,10},{ 4, 6},{ 1, 5},{ 3, 3},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,11},{ 3,15},{ 2, 5},{ 4, 3} },  /* slot 21 */
  { { 0, 0},{ 4,12},{ 2,16},{ 1, 4},{ 3, 5},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 6},{ 3, 4},{ 2,11},{ 4, 5} },  /* slot 22 */
  { { 0, 0},{ 4,16},{ 2,14},{ 1, 6},{ 3, 2},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,15},{ 3, 6},{ 2, 7},{ 4, 5} },  /* slot 23 */
  { { 0, 0},{ 2, 5},{ 4,11},{ 4, 1},{ 1,18},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,18},{ 3,16},{ 2,15},{ 4,15} },  /* slot 24 */
  { { 0, 0},{ 2, 3},{ 4, 9},{ 3,15},{ 1,14},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 7},{ 3, 5},{ 2, 8},{ 4, 8} },  /* slot 25 */
  { { 0, 0},{ 2, 6},{ 2,13},{ 1, 4},{ 3, 8},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 4},{ 3,14},{ 2, 3},{ 4,14} },  /* slot 26 */
  { { 0, 0},{ 4,13},{ 2, 7},{ 1, 3},{ 3, 8},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,13},{ 3,12},{ 2, 6},{ 4,16} },  /* slot 27 */
  { { 0, 0},{ 4,15},{ 4, 8},{ 3,18},{ 1,15},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 1},{ 3,17},{ 2,17},{ 4,15} },  /* slot 28 */
  { { 0, 0},{ 2, 6},{ 4, 4},{ 3,14},{ 3, 7},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 9},{ 3, 9},{ 2, 8},{ 4,13} },  /* slot 29 */
  { { 0, 0},{ 4,17},{ 4, 3},{ 1, 5},{ 1,10},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 3},{ 3, 7},{ 2,14},{ 4,14} },  /* slot 30 */
  { { 0, 0},{ 4,14},{ 1, 2},{ 3,15},{ 3, 1},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,14},{ 3, 1},{ 2,16},{ 4, 6} },  /* slot 31 */
  { { 0, 0},{ 4,13},{ 4, 5},{ 1, 7},{ 1,14},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,14},{ 3, 8},{ 2, 6},{ 4, 4} },  /* slot 32 */
  { { 0, 0},{ 2, 8},{ 4, 7},{ 3,10},{ 1,17},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,17},{ 3,10},{ 2, 9},{ 4, 8} },  /* slot 33 */
  { { 0, 0},{ 2, 2},{ 2,12},{ 1, 6},{ 3, 6},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 1},{ 3,18},{ 2, 1},{ 4, 9} },  /* slot 34 */
  { { 0, 0},{ 2,18},{ 4, 2},{ 3,18},{ 3, 3},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 8},{ 3,13},{ 2, 4},{ 4,12} },  /* slot 35 */
  { { 0, 0},{ 4,12},{ 4, 2},{ 3,12},{ 3,11},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 2},{ 3, 1},{ 2,15},{ 4,16} },  /* slot 36 */
  { { 0, 0},{ 2, 1},{ 4, 5},{ 3,17},{ 1,13},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,16},{ 3,16},{ 2,13},{ 4, 9} },  /* slot 37 */
  { { 0, 0},{ 2, 1},{ 2,16},{ 4, 1},{ 3, 4},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 3},{ 3,11},{ 2, 1},{ 4, 1} },  /* slot 38 */
  { { 0, 0},{ 4,14},{ 4,11},{ 3,10},{ 1,16},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,12},{ 3, 9},{ 2,17},{ 4,10} },  /* slot 39 */
  { { 0, 0},{ 2,10},{ 4, 6},{ 3,16},{ 1,12},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 7},{ 3, 4},{ 2, 9},{ 4, 7} },  /* slot 40 */
  { { 0, 0},{ 2, 9},{ 1, 2},{ 1, 8},{ 3, 4},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,10},{ 3, 3},{ 2, 5},{ 4, 1} },  /* slot 41 */
  { { 0, 0},{ 4,18},{ 4, 8},{ 1, 7},{ 3, 7},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 9},{ 3, 2},{ 2,13},{ 4,11} },  /* slot 42 */
  { { 0, 0},{ 4,15},{ 2,14},{ 3,12},{ 3, 2},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,13},{ 3,13},{ 2,12},{ 4,11} },  /* slot 43 */
  { { 0, 0},{ 2, 8},{ 4, 4},{ 3,17},{ 1,18},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,18},{ 3,10},{ 2,11},{ 4, 3} },  /* slot 44 */
  { { 0, 0},{ 2,18},{ 2,13},{ 1, 3},{ 1,17},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 2},{ 3,17},{ 2,16},{ 4,12} },  /* slot 45 */
  { { 0, 0},{ 4,17},{ 2,15},{ 3,13},{ 1,16},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,10},{ 3, 3},{ 2, 7},{ 4,17} },  /* slot 46 */
  { { 0, 0},{ 4,10},{ 4, 9},{ 3,14},{ 3, 5},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 4},{ 3, 6},{ 2, 2},{ 4,17} },  /* slot 47 */
  { { 0, 0},{ 2, 5},{ 2,17},{ 3,13},{ 3,11},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 5},{ 3, 2},{ 2,10},{ 4,18} },  /* slot 48 */
  { { 0, 0},{ 2,10},{ 2,12},{ 1,11},{ 1,13},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,17},{ 3,14},{ 2, 3},{ 4, 4} },  /* slot 49 */
  { { 0, 0},{ 4,16},{ 2,11},{ 1, 1},{ 1,10},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 8},{ 3,11},{ 2,18},{ 4, 7} },  /* slot 50 */
  { { 0, 0},{ 2, 3},{ 2,11},{ 1,11},{ 3, 6},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,11},{ 3,12},{ 2,12},{ 4,10} },  /* slot 51 */
  { { 0, 0},{ 2, 4},{ 4, 7},{ 1, 9},{ 3, 9},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,12},{ 3,15},{ 2, 2},{ 4,13} },  /* slot 52 */
  { { 0, 0},{ 2, 9},{ 4, 3},{ 1, 1},{ 1,12},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1,16},{ 3,18},{ 2,10},{ 4, 2} },  /* slot 53 */
  { { 0, 0},{ 4,18},{ 2, 7},{ 1, 9},{ 1,15},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 0, 0},{ 1, 6},{ 3, 7},{ 2,18},{ 4,18} },  /* slot 54 */
};

/* Returns 1 and fills out_fb/out_mc when the derived table covers this
 * (ISE slot, A-byte code). Code 0 is never covered. */
static int xl9572_derived_src(int ise_slot, int code, int *out_fb, int *out_mc) {
    if (ise_slot < 1 || ise_slot > 54) return 0;
    if (code < 1 || code > 19) return 0;
    if (xl9572_derived[ise_slot - 1][code][0] == 0) return 0;
    *out_fb = xl9572_derived[ise_slot - 1][code][0];
    *out_mc = xl9572_derived[ise_slot - 1][code][1];
    if (*out_fb < 1 || *out_fb > 4) return 0;
    if (*out_mc < 1 || *out_mc > 18) return 0;
    return 1;
}

/* ---------------------------------------------------------------------
 * XC9536XL imux codes 1, 16 and 17, derived from the vendor .chp:
 * external position p -> code p+1, feedback position q -> code 16+q.
 *
 * (fb,mc) is package-independent here, so one table serves VQ44, PC44,
 * CS48 and VQ64; the pin comes from dev->mc_pin as usual.
 *
 * Code 0 is the unprogrammed default and has no source in the .chp;
 * those inputs keep their placeholder. */
/* Fourth column: imux code 2. Filled only for the slots where every
 * package's table and the .chp agree; the other slots stay 0 and give a
 * placeholder. */
static const unsigned char xl9536_derived[54][4][2] = {
    {{ 2, 9},{ 1, 1},{ 2, 1},{ 0, 0}},  /* slot 1  c1/c16/c17/c2 */
    {{ 2,10},{ 1, 2},{ 2, 2},{ 1, 1}},  /* slot 2  c1/c16/c17/c2 */
    {{ 2, 8},{ 1, 3},{ 2, 3},{ 1,17}},  /* slot 3  c1/c16/c17/c2 */
    {{ 2,11},{ 1, 4},{ 2, 4},{ 1, 2}},  /* slot 4  c1/c16/c17/c2 */
    {{ 2, 7},{ 1, 5},{ 2, 5},{ 1,18}},  /* slot 5  c1/c16/c17/c2 */
    {{ 2,12},{ 1, 6},{ 2, 6},{ 1, 4}},  /* slot 6  c1/c16/c17/c2 */
    {{ 2, 6},{ 1, 7},{ 2, 7},{ 1,16}},  /* slot 7  c1/c16/c17/c2 */
    {{ 2,13},{ 1, 8},{ 2, 8},{ 1, 3}},  /* slot 8  c1/c16/c17/c2 */
    {{ 2, 5},{ 1, 9},{ 2, 9},{ 1,15}},  /* slot 9  c1/c16/c17/c2 */
    {{ 2,18},{ 1,10},{ 2,10},{ 1, 5}},  /* slot 10  c1/c16/c17/c2 */
    {{ 2, 3},{ 1,11},{ 2,11},{ 1,14}},  /* slot 11  c1/c16/c17/c2 */
    {{ 2,14},{ 1,12},{ 2,12},{ 1, 7}},  /* slot 12  c1/c16/c17/c2 */
    {{ 2, 4},{ 1,13},{ 2,13},{ 1,13}},  /* slot 13  c1/c16/c17/c2 */
    {{ 2,15},{ 1,14},{ 2,14},{ 1, 6}},  /* slot 14  c1/c16/c17/c2 */
    {{ 2, 2},{ 1,15},{ 2,15},{ 1,12}},  /* slot 15  c1/c16/c17/c2 */
    {{ 2,16},{ 1,16},{ 2,16},{ 1, 8}},  /* slot 16  c1/c16/c17/c2 */
    {{ 2, 1},{ 1,17},{ 2,17},{ 1,11}},  /* slot 17  c1/c16/c17/c2 */
    {{ 2,17},{ 1,18},{ 2,18},{ 1, 9}},  /* slot 18  c1/c16/c17/c2 */
    {{ 2, 7},{ 1, 4},{ 2,12},{ 1,10}},  /* slot 19  c1/c16/c17/c2 */
    {{ 2, 6},{ 1, 7},{ 2,14},{ 0, 0}},  /* slot 20  c1/c16/c17/c2 */
    {{ 2,15},{ 1, 1},{ 2,10},{ 0, 0}},  /* slot 21  c1/c16/c17/c2 */
    {{ 2,14},{ 1,11},{ 2,13},{ 1,18}},  /* slot 22  c1/c16/c17/c2 */
    {{ 2,13},{ 1, 3},{ 2, 9},{ 1, 7}},  /* slot 23  c1/c16/c17/c2 */
    {{ 2, 5},{ 1, 2},{ 2,16},{ 0, 0}},  /* slot 24  c1/c16/c17/c2 */
    {{ 2, 9},{ 1,16},{ 2, 1},{ 1,16}},  /* slot 25  c1/c16/c17/c2 */
    {{ 2, 3},{ 1,11},{ 2,17},{ 0, 0}},  /* slot 26  c1/c16/c17/c2 */
    {{ 2,18},{ 1, 9},{ 2,17},{ 0, 0}},  /* slot 27  c1/c16/c17/c2 */
    {{ 2,11},{ 1,12},{ 2, 5},{ 0, 0}},  /* slot 28  c1/c16/c17/c2 */
    {{ 2, 3},{ 1,18},{ 2, 2},{ 0, 0}},  /* slot 29  c1/c16/c17/c2 */
    {{ 2,10},{ 1, 1},{ 2, 3},{ 0, 0}},  /* slot 30  c1/c16/c17/c2 */
    {{ 2,17},{ 1,16},{ 2,14},{ 1,14}},  /* slot 31  c1/c16/c17/c2 */
    {{ 2,18},{ 1,17},{ 2, 1},{ 0, 0}},  /* slot 32  c1/c16/c17/c2 */
    {{ 2, 4},{ 1,10},{ 2, 7},{ 1,12}},  /* slot 33  c1/c16/c17/c2 */
    {{ 2, 6},{ 1, 3},{ 2, 4},{ 0, 0}},  /* slot 34  c1/c16/c17/c2 */
    {{ 2, 8},{ 1,12},{ 2,10},{ 1, 4}},  /* slot 35  c1/c16/c17/c2 */
    {{ 2,14},{ 1, 8},{ 2, 6},{ 1,11}},  /* slot 36  c1/c16/c17/c2 */
    {{ 2,12},{ 1,14},{ 2,15},{ 0, 0}},  /* slot 37  c1/c16/c17/c2 */
    {{ 2,12},{ 1, 2},{ 2, 8},{ 0, 0}},  /* slot 38  c1/c16/c17/c2 */
    {{ 2,17},{ 1,10},{ 2,18},{ 0, 0}},  /* slot 39  c1/c16/c17/c2 */
    {{ 2, 1},{ 1, 4},{ 2,11},{ 0, 0}},  /* slot 40  c1/c16/c17/c2 */
    {{ 2, 2},{ 1, 7},{ 2, 8},{ 1,18}},  /* slot 41  c1/c16/c17/c2 */
    {{ 2,16},{ 1,17},{ 2, 2},{ 1, 9}},  /* slot 42  c1/c16/c17/c2 */
    {{ 2,11},{ 1, 8},{ 2, 9},{ 0, 0}},  /* slot 43  c1/c16/c17/c2 */
    {{ 2, 4},{ 1,14},{ 2,16},{ 1, 5}},  /* slot 44  c1/c16/c17/c2 */
    {{ 2, 8},{ 1, 9},{ 2, 7},{ 0, 0}},  /* slot 45  c1/c16/c17/c2 */
    {{ 2,10},{ 1, 6},{ 2,18},{ 0, 0}},  /* slot 46  c1/c16/c17/c2 */
    {{ 2,15},{ 1,18},{ 2,13},{ 0, 0}},  /* slot 47  c1/c16/c17/c2 */
    {{ 2, 5},{ 1, 6},{ 2, 6},{ 0, 0}},  /* slot 48  c1/c16/c17/c2 */
    {{ 2, 1},{ 1,15},{ 2,15},{ 0, 0}},  /* slot 49  c1/c16/c17/c2 */
    {{ 2,13},{ 1, 5},{ 2, 3},{ 1, 4}},  /* slot 50  c1/c16/c17/c2 */
    {{ 2, 9},{ 1,15},{ 2, 4},{ 0, 0}},  /* slot 51  c1/c16/c17/c2 */
    {{ 2, 7},{ 1,13},{ 2,12},{ 1, 2}},  /* slot 52  c1/c16/c17/c2 */
    {{ 2, 2},{ 1, 5},{ 2,11},{ 1,17}},  /* slot 53  c1/c16/c17/c2 */
    {{ 2,16},{ 1,13},{ 2, 5},{ 0, 0}},  /* slot 54  c1/c16/c17/c2 */
};

/* Resolve an XC9536XL A-byte code from the derived table above.
 * Code 2 is answered only on the agreed slots. */
/* XC9536XL imux sources indexed by destination FB, input and code
 * (codes 1, 2, 16, 17), from the vendor input-choice lists.
 * Codes 1, 16 and 17 name the same source for both destination FBs; code 2
 * depends on the destination FB. fb1 has a code-2 source for only some
 * inputs, fb2 for all 54. A 0 entry means the vendor defines no source
 * for that (fb, input, code) and the tool names nothing. */
/* fb1 code-2 entries for inputs 29, 32 and 34 come from the raw
 * xc9536xl.chp: its <imux> <out> list carries four entries per input
 * (label 'G1 G2 G16 G17'), element [1] is the code-2 source, and <pads>
 * maps that imux line to a macrocell:
 *     input 29 -> FB1_MC15   input 32 -> FB1_MC12   input 34 -> FB1_MC4
 * A macrocell reading its own output this way with positive polarity
 * (q <= A and B and q) is a latch or a hold, not an oscillator. */
static const unsigned char xl9536_fbimux[2][54][4][2] = {
  {
    {{ 2, 9},{ 1, 1},{ 1, 1},{ 2, 1}},  /* fb1 input1  */
    {{ 2,10},{ 1,17},{ 1, 2},{ 2, 2}},  /* fb1 input2  */
    {{ 2, 8},{ 1, 2},{ 1, 3},{ 2, 3}},  /* fb1 input3  */
    {{ 2,11},{ 1,18},{ 1, 4},{ 2, 4}},  /* fb1 input4  */
    {{ 2, 7},{ 1, 4},{ 1, 5},{ 2, 5}},  /* fb1 input5  */
    {{ 2,12},{ 1,16},{ 1, 6},{ 2, 6}},  /* fb1 input6  */
    {{ 2, 6},{ 1, 3},{ 1, 7},{ 2, 7}},  /* fb1 input7  */
    {{ 2,13},{ 1,15},{ 1, 8},{ 2, 8}},  /* fb1 input8  */
    {{ 2, 5},{ 1, 5},{ 1, 9},{ 2, 9}},  /* fb1 input9  */
    {{ 2,18},{ 1,14},{ 1,10},{ 2,10}},  /* fb1 input10 */
    {{ 2, 3},{ 1, 7},{ 1,11},{ 2,11}},  /* fb1 input11 */
    {{ 2,14},{ 1,13},{ 1,12},{ 2,12}},  /* fb1 input12 */
    {{ 2, 4},{ 1, 6},{ 1,13},{ 2,13}},  /* fb1 input13 */
    {{ 2,15},{ 1,12},{ 1,14},{ 2,14}},  /* fb1 input14 */
    {{ 2, 2},{ 1, 8},{ 1,15},{ 2,15}},  /* fb1 input15 */
    {{ 2,16},{ 1,11},{ 1,16},{ 2,16}},  /* fb1 input16 */
    {{ 2, 1},{ 1, 9},{ 1,17},{ 2,17}},  /* fb1 input17 */
    {{ 2,17},{ 1,10},{ 1,18},{ 2,18}},  /* fb1 input18 */
    {{ 2, 7},{ 0, 0},{ 1, 4},{ 2,12}},  /* fb1 input19 */
    {{ 2, 6},{ 0, 0},{ 0, 0},{ 2,14}},  /* fb1 input20 */
    {{ 2,15},{ 0, 0},{ 1, 1},{ 2,10}},  /* fb1 input21 */
    {{ 2,14},{ 0, 0},{ 0, 0},{ 2,13}},  /* fb1 input22 */
    {{ 2,13},{ 0, 0},{ 1, 3},{ 0, 0}},  /* fb1 input23 */
    {{ 2, 5},{ 0, 0},{ 1, 2},{ 2,16}},  /* fb1 input24 */
    {{ 2, 9},{ 0, 0},{ 1,16},{ 0, 0}},  /* fb1 input25 */
    {{ 2, 3},{ 0, 0},{ 1,11},{ 2,17}},  /* fb1 input26 */
    {{ 2,18},{ 0, 0},{ 1, 9},{ 0, 0}},  /* fb1 input27 */
    {{ 2,11},{ 0, 0},{ 0, 0},{ 2, 5}},  /* fb1 input28 */
    {{ 2, 3},{ 1,15},{ 1,18},{ 2, 2}},  /* fb1 input29 */
    {{ 2,10},{ 0, 0},{ 1, 1},{ 2, 3}},  /* fb1 input30 */
    {{ 2,17},{ 0, 0},{ 1,16},{ 2,14}},  /* fb1 input31 */
    {{ 2,18},{ 1,12},{ 1,17},{ 0, 0}},  /* fb1 input32 */
    {{ 2, 4},{ 0, 0},{ 1,10},{ 0, 0}},  /* fb1 input33 */
    {{ 2, 6},{ 1, 4},{ 0, 0},{ 2, 4}},  /* fb1 input34 */
    {{ 2, 8},{ 0, 0},{ 1,12},{ 2,10}},  /* fb1 input35 */
    {{ 2,14},{ 0, 0},{ 1, 8},{ 2, 6}},  /* fb1 input36 */
    {{ 2,12},{ 0, 0},{ 0, 0},{ 2,15}},  /* fb1 input37 */
    {{ 2,12},{ 0, 0},{ 1, 2},{ 2, 8}},  /* fb1 input38 */
    {{ 2,17},{ 0, 0},{ 1,10},{ 2,18}},  /* fb1 input39 */
    {{ 2, 1},{ 0, 0},{ 0, 0},{ 2,11}},  /* fb1 input40 */
    {{ 2, 2},{ 0, 0},{ 0, 0},{ 2, 8}},  /* fb1 input41 */
    {{ 2,16},{ 0, 0},{ 1,17},{ 2, 2}},  /* fb1 input42 */
    {{ 2,11},{ 0, 0},{ 1, 8},{ 0, 0}},  /* fb1 input43 */
    {{ 2, 4},{ 0, 0},{ 1,14},{ 2,16}},  /* fb1 input44 */
    {{ 2, 8},{ 0, 0},{ 0, 0},{ 0, 0}},  /* fb1 input45 */
    {{ 2,10},{ 0, 0},{ 0, 0},{ 2,18}},  /* fb1 input46 */
    {{ 2,15},{ 0, 0},{ 0, 0},{ 2,13}},  /* fb1 input47 */
    {{ 2, 5},{ 0, 0},{ 1, 6},{ 2, 6}},  /* fb1 input48 */
    {{ 2, 1},{ 0, 0},{ 1,15},{ 2,15}},  /* fb1 input49 */
    {{ 2,13},{ 0, 0},{ 1, 5},{ 0, 0}},  /* fb1 input50 */
    {{ 2, 9},{ 0, 0},{ 1,15},{ 2, 4}},  /* fb1 input51 */
    {{ 2, 7},{ 0, 0},{ 1,13},{ 2,12}},  /* fb1 input52 */
    {{ 2, 2},{ 0, 0},{ 1, 5},{ 2,11}},  /* fb1 input53 */
    {{ 2,16},{ 0, 0},{ 1,13},{ 2, 5}},  /* fb1 input54 */
  },
  {
    {{ 2, 9},{ 1, 1},{ 1, 1},{ 2, 1}},  /* fb2 input1  */
    {{ 2,10},{ 1,17},{ 1, 2},{ 2, 2}},  /* fb2 input2  */
    {{ 2, 8},{ 1, 2},{ 1, 3},{ 2, 3}},  /* fb2 input3  */
    {{ 2,11},{ 1,18},{ 1, 4},{ 2, 4}},  /* fb2 input4  */
    {{ 2, 7},{ 1, 4},{ 1, 5},{ 2, 5}},  /* fb2 input5  */
    {{ 2,12},{ 1,16},{ 1, 6},{ 2, 6}},  /* fb2 input6  */
    {{ 2, 6},{ 1, 3},{ 1, 7},{ 2, 7}},  /* fb2 input7  */
    {{ 2,13},{ 1,15},{ 1, 8},{ 2, 8}},  /* fb2 input8  */
    {{ 2, 5},{ 1, 5},{ 1, 9},{ 2, 9}},  /* fb2 input9  */
    {{ 2,18},{ 1,14},{ 1,10},{ 2,10}},  /* fb2 input10 */
    {{ 2, 3},{ 1, 7},{ 0, 0},{ 2,11}},  /* fb2 input11 */
    {{ 2,14},{ 1,13},{ 1,12},{ 2,12}},  /* fb2 input12 */
    {{ 2, 4},{ 1, 6},{ 1,13},{ 2,13}},  /* fb2 input13 */
    {{ 2,15},{ 1,12},{ 1,14},{ 2,14}},  /* fb2 input14 */
    {{ 2, 2},{ 1, 8},{ 1,15},{ 2,15}},  /* fb2 input15 */
    {{ 2,16},{ 1,11},{ 1,16},{ 2,16}},  /* fb2 input16 */
    {{ 2, 1},{ 1, 9},{ 1,17},{ 2,17}},  /* fb2 input17 */
    {{ 2,17},{ 1,10},{ 1,18},{ 2,18}},  /* fb2 input18 */
    {{ 0, 0},{ 1, 6},{ 1, 4},{ 2,12}},  /* fb2 input19 */
    {{ 0, 0},{ 1, 8},{ 0, 0},{ 2,14}},  /* fb2 input20 */
    {{ 0, 0},{ 1,18},{ 1, 1},{ 2,10}},  /* fb2 input21 */
    {{ 0, 0},{ 1, 7},{ 1,11},{ 2,13}},  /* fb2 input22 */
    {{ 0, 0},{ 1, 5},{ 1, 3},{ 2, 9}},  /* fb2 input23 */
    {{ 0, 0},{ 1,16},{ 1, 2},{ 2,16}},  /* fb2 input24 */
    {{ 0, 0},{ 1,10},{ 1,16},{ 2, 1}},  /* fb2 input25 */
    {{ 0, 0},{ 1, 3},{ 1,11},{ 2,17}},  /* fb2 input26 */
    {{ 0, 0},{ 1, 1},{ 1, 9},{ 2,17}},  /* fb2 input27 */
    {{ 0, 0},{ 1,13},{ 1,12},{ 2, 5}},  /* fb2 input28 */
    {{ 0, 0},{ 1,15},{ 1,18},{ 2, 2}},  /* fb2 input29 */
    {{ 0, 0},{ 1,14},{ 1, 1},{ 2, 3}},  /* fb2 input30 */
    {{ 0, 0},{ 1, 9},{ 1,16},{ 2,14}},  /* fb2 input31 */
    {{ 0, 0},{ 1,12},{ 1,17},{ 0, 0}},  /* fb2 input32 */
    {{ 0, 0},{ 1,17},{ 1,10},{ 0, 0}},  /* fb2 input33 */
    {{ 0, 0},{ 1, 4},{ 0, 0},{ 2, 4}},  /* fb2 input34 */
    {{ 0, 0},{ 1,11},{ 1,12},{ 2,10}},  /* fb2 input35 */
    {{ 0, 0},{ 1,11},{ 1, 8},{ 2, 6}},  /* fb2 input36 */
    {{ 0, 0},{ 1,12},{ 1,14},{ 2,15}},  /* fb2 input37 */
    {{ 0, 0},{ 1, 7},{ 1, 2},{ 2, 8}},  /* fb2 input38 */
    {{ 0, 0},{ 1,16},{ 1,10},{ 0, 0}},  /* fb2 input39 */
    {{ 0, 0},{ 1,18},{ 1, 4},{ 2,11}},  /* fb2 input40 */
    {{ 0, 0},{ 1, 9},{ 0, 0},{ 2, 8}},  /* fb2 input41 */
    {{ 0, 0},{ 1,13},{ 1,17},{ 2, 2}},  /* fb2 input42 */
    {{ 0, 0},{ 1, 5},{ 1, 8},{ 2, 9}},  /* fb2 input43 */
    {{ 0, 0},{ 1,15},{ 1,14},{ 2,16}},  /* fb2 input44 */
    {{ 0, 0},{ 1, 3},{ 1, 9},{ 2, 7}},  /* fb2 input45 */
    {{ 0, 0},{ 1, 6},{ 1, 6},{ 2,18}},  /* fb2 input46 */
    {{ 0, 0},{ 1,10},{ 1,18},{ 2,13}},  /* fb2 input47 */
    {{ 0, 0},{ 1, 8},{ 1, 6},{ 2, 6}},  /* fb2 input48 */
    {{ 0, 0},{ 1, 4},{ 1,15},{ 2,15}},  /* fb2 input49 */
    {{ 0, 0},{ 1, 2},{ 1, 5},{ 2, 3}},  /* fb2 input50 */
    {{ 0, 0},{ 1, 2},{ 1,15},{ 2, 4}},  /* fb2 input51 */
    {{ 0, 0},{ 1,17},{ 1,13},{ 0, 0}},  /* fb2 input52 */
    {{ 0, 0},{ 1,14},{ 1, 5},{ 2,11}},  /* fb2 input53 */
    {{ 0, 0},{ 1, 1},{ 1,13},{ 2, 5}},  /* fb2 input54 */
  },
};

static int xl9536_fbimux_src(int fb_dest, int ise_slot, int code,
                             int *out_fb, int *out_mc) {
    int ci;
    if (fb_dest < 1 || fb_dest > 2) return 0;
    if (ise_slot < 1 || ise_slot > 54) return 0;
    if      (code == 1)  ci = 0;
    else if (code == 2)  ci = 1;
    else if (code == 16) ci = 2;
    else if (code == 17) ci = 3;
    else return 0;
    if (xl9536_fbimux[fb_dest-1][ise_slot-1][ci][0] == 0) return 0;
    *out_fb = xl9536_fbimux[fb_dest-1][ise_slot-1][ci][0];
    *out_mc = xl9536_fbimux[fb_dest-1][ise_slot-1][ci][1];
    return 1;
}

/* Marked-guess fallback for XC9536XL imux code 2 on destination FB1, where
 * the vendor defines no fb1 source: borrow fb2's code-2 entry for the same
 * input. Code 2 is the one code that can differ by destination FB, so this
 * is a guess and the caller marks it with '?'.
 *
 * Self-loop guard: if the guess names the macrocell that is reading the
 * literal, a combinational macrocell would feed its own product term (an
 * oscillator), so the guess is refused and the placeholder stays. */
/* The macrocell whose product term is currently being named. Set by
 * decode_print()'s macrocell loop and used only by the self-loop guard
 * below. Zero when unknown, which disables the guess. */
static int g_consumer_fb = 0, g_consumer_mc = 0;

static int xl9536_code2_guess(int fb_dest, int ise_slot,
                              int consumer_fb, int consumer_mc,
                              int *out_fb, int *out_mc) {
    int other = (fb_dest == 1) ? 2 : 1;
    if (fb_dest < 1 || fb_dest > 2) return 0;
    if (!xl9536_fbimux_src(other, ise_slot, 2, out_fb, out_mc)) return 0;
    if (*out_fb == consumer_fb && *out_mc == consumer_mc) return 0;  /* self-loop */
    return 1;
}

static int xl9536_derived_src(int ise_slot, int code, int *out_fb, int *out_mc) {
    int col;
    if (ise_slot < 1 || ise_slot > 54) return 0;
    if (code == 1)       col = 0;
    else if (code == 16) col = 1;
    else if (code == 17) col = 2;
    else if (code == 2)  col = 3;       /* agreed slots only */
    else return 0;                      /* code 0 excluded */
    if (xl9536_derived[ise_slot - 1][col][0] == 0) return 0;
    *out_fb = xl9536_derived[ise_slot - 1][col][0];
    *out_mc = xl9536_derived[ise_slot - 1][col][1];
    if (*out_fb < 1 || *out_fb > 2) return 0;
    if (*out_mc < 1 || *out_mc > 18) return 0;
    return 1;
}

/* ---------------------------------------------------------------------
 * The FastCONNECT select index space begins with the pair's local
 * feedback entries.
 *
 *     select_value = 13 - index        (LSB-first over select_bits[])
 *     index 0 .. n_feedback-1  ->  this pair's feedback list
 *     index n_feedback ..      ->  the external candidate list
 *
 * Feedback entry Fn is the destination function block's own macrocell n+1.
 *
 * The feedback lists are the same on XC95108 and XC9572 (one shared
 * FastCONNECT matrix), so one table serves both. -1 marks an absent entry. */
static const signed char nonxl_pair_feedback[36][3] = {
    {  0, -1, -1},   /* pair 0  : F0 */
    {  0,  1, -1},   /* pair 1  : F0,F1 */
    {  0,  1,  2},   /* pair 2  : F0,F1,F2 */
    {  1,  2,  3},   /* pair 3  : F1,F2,F3 */
    {  3,  4, -1},   /* pair 4  : F3,F4 */
    {  2,  4,  5},   /* pair 5  : F2,F4,F5 */
    {  3,  5,  6},   /* pair 6  : F3,F5,F6 */
    {  6,  7, -1},   /* pair 7  : F6,F7 */
    {  4,  7,  8},   /* pair 8  : F4,F7,F8 */
    {  5,  8,  9},   /* pair 9  : F5,F8,F9 */
    {  9, 10, -1},   /* pair 10 : F9,F10 */
    {  6, 10, 11},   /* pair 11 : F6,F10,F11 */
    {  7, 11, 12},   /* pair 12 : F7,F11,F12 */
    { 12, 13, -1},   /* pair 13 : F12,F13 */
    {  8, 13, 14},   /* pair 14 : F8,F13,F14 */
    {  9, 14, 15},   /* pair 15 : F9,F14,F15 */
    { 15, 16, -1},   /* pair 16 : F15,F16 */
    { 10, 16, 17},   /* pair 17 : F10,F16,F17 */
    { 11, 17, -1},   /* pair 18 : F11,F17 */
    { -1, -1, -1},   /* pair 19 :  */
    { 12, -1, -1},   /* pair 20 : F12 */
    { 13, -1, -1},   /* pair 21 : F13 */
    { -1, -1, -1},   /* pair 22 :  */
    { 14, -1, -1},   /* pair 23 : F14 */
    { 15, -1, -1},   /* pair 24 : F15 */
    { -1, -1, -1},   /* pair 25 :  */
    { 16, -1, -1},   /* pair 26 : F16 */
    { 17, -1, -1},   /* pair 27 : F17 */
    { -1, -1, -1},   /* pair 28 :  */
    { -1, -1, -1},   /* pair 29 :  */
    { -1, -1, -1},   /* pair 30 :  */
    { -1, -1, -1},   /* pair 31 :  */
    { -1, -1, -1},   /* pair 32 :  */
    { -1, -1, -1},   /* pair 33 :  */
    { -1, -1, -1},   /* pair 34 :  */
    { -1, -1, -1},   /* pair 35 :  */
};

/* Returns 1 and fills *out_mc when the observed select value falls in
 * this pair's feedback index range. The source is always in the
 * DESTINATION function block, so no source-FB is returned. */
static int nonxl_feedback_source(const fc_entry_t *fe,
                                 const unsigned char *fuses,
                                 int pair, int *out_mc) {
    int bi, v = 0, idx;
    if (!fe || !fuses) return 0;
    if (pair < 0 || pair >= 36) return 0;
    /* 5-bit select devices (XC95144, XC95216, XC95288) use 29 - v instead
     * of 13 - v. The Fn -> MC(n+1) mapping and the per-pair list are
     * properties of the shared FastCONNECT matrix and do not change. */
    if (fe->n_select_bits != 4 && fe->n_select_bits != 5) return 0;
    for (bi = 0; bi < fe->n_select_bits; bi++)
        v |= (fuses[fe->select_bits[bi]] ? 1 : 0) << bi;
    idx = (fe->n_select_bits == 5) ? (29 - v) : (13 - v);
    if (idx < 0 || idx > 2) return 0;
    if (nonxl_pair_feedback[pair][idx] < 0) return 0;
    *out_mc = nonxl_pair_feedback[pair][idx] + 1;
    if (*out_mc < 1 || *out_mc > 18) return 0;
    return 1;
}

/* Formats the feedback source into buf. Returns 1 on success. */
static int nonxl_feedback_name(const device_t *dev, const fc_entry_t *fe,
                               const unsigned char *fuses, int fb_dest,
                               int pair, const char *pol,
                               char *buf, size_t buflen) {
    int smc = 0;
    if (!dev || dev->is_xl) return 0;
    /* nonxl_pair_feedback[] is the same on XC9572, XC95108, XC95144, XC95216
     * and XC95288. XC9536 declares no feedback entries on any pair, so the
     * table is never applied there. XC9536 is the only non-XL part with
     * fewer than 4 function blocks. */
    if (dev->n_fbs < 4) return 0;
    if (!nonxl_feedback_source(fe, fuses, pair, &smc)) return 0;
    /* A feedback source is the FB output, not the pad. */
    snprintf(buf, buflen, "%sn_FB%dMC%d", pol, fb_dest, smc);
    return 1;
}

/* ---------------------------------------------------------------------
 * The local-feedback source macrocell is recorded in the AND array's
 * identity region.
 *
 * An FB is jed_fuses_per_fb/108 columns wide -- 108 for XC95108, 96 for
 * XC9572, 120 for XC95144, 84 for XC9536. The AND array is the first 72;
 * the rest is the identity region, and (fb_cols-72)*3 == n_fbs*18: one
 * position per macrocell in the device. Never hardcode 72..107 -- that is
 * XC95108 only and reads past the FB boundary on narrower parts.
 *
 * The region holds 36 parallel planes indexed by row offset r = 0..35;
 * plane r occupies rows {r, r+36, r+72} across the region's columns.
 * One plane per FastCONNECT pair, encoding the global macrocell number
 *     idx = (src_fb-1)*18 + (src_mc-1).
 *
 * pair -> plane follows
 *     plane = (pair/5) + {0,8,15,22,29}[pair%5]
 * a bijection over 0..35. */
static const int nonxl_pair_plane[36] = {
     0,  8, 15, 22, 29,  1,  9, 16, 23, 30,  2, 10,
    17, 24, 31,  3, 11, 18, 25, 32,  4, 12, 19, 26,
    33,  5, 13, 20, 27, 34,  6, 14, 21, 28, 35,  7
};

/* Returns 1 and fills *out_fb and *out_mc when the identity region names a
 * same-FB local-feedback source for `pair`; 0 otherwise, in which case
 * the caller keeps its placeholder. */
static int nonxl_plane_source_n(const device_t *dev, const unsigned char *fuses,
                              int fb_dest, int pair, int *out_fb, int *out_mc,
                              int *out_n_cands) {
    int fb_cols, n_extra, c, j, hits = 0, zeros = 0, idx = -1, plane, sfb, smc;

    if (!dev || !fuses || dev->is_xl) return 0;
    if (pair < 0 || pair >= 36) return 0;
    plane = nonxl_pair_plane[pair];
    if (plane < 0) return 0;

    if (dev->jed_fuses_per_fb <= 0) return 0;
    fb_cols = dev->jed_fuses_per_fb / COLUMN_WIDTH;
    n_extra = fb_cols - 72;
    if (n_extra <= 0) return 0;
    if (n_extra * 3 != dev->n_fbs * 18) return 0;   /* (fb_cols-72)*3 == n_fbs*18 */

    /* The plane row is one-cold: the source macrocell's bit is left
     * unprogrammed (1) and every other bit in the row is driven to 0.
     * A row that carries no local feedback is still all ones, so a row is
     * meaningful only if it contains at least one 0. */
    for (c = 72; c < fb_cols; c++) {
        const unsigned char *col = device_col(dev, fuses, fb_dest, c);
        for (j = 0; j < 3; j++) {
            if (col[plane + 36 * j] == 1) { idx = 3 * (c - 72) + j; hits++; }
            else                          { zeros++; }
        }
    }
    if (zeros == 0) return 0;      /* row never written: no local feedback */
    if (hits != 1) {
        /* Several bits left at 1. Report the first candidate and how many
         * there were, so the caller can mark it. `idx` holds the last hit,
         * so re-scan for the first (a deterministic pick). */
        if (hits > 1 && out_n_cands) {
            int c2, j2, first = -1;
            for (c2 = 72; c2 < fb_cols && first < 0; c2++) {
                const unsigned char *col2 = device_col(dev, fuses, fb_dest, c2);
                for (j2 = 0; j2 < 3; j2++)
                    if (col2[plane + 36 * j2] == 1) { first = 3 * (c2 - 72) + j2; break; }
            }
            if (first >= 0 && first < dev->n_fbs * 18) {
                *out_fb = first / 18 + 1;
                *out_mc = first % 18 + 1;
                *out_n_cands = hits;
                return 1;
            }
        }
        return 0;
    }
    if (out_n_cands) *out_n_cands = 1;
    if (idx < 0 || idx >= dev->n_fbs * 18) return 0;

    sfb = idx / 18 + 1;
    smc = idx % 18 + 1;
    /* The identity region holds a global macrocell number and can name a
     * source in any function block. `smc` is bounded by the n_fbs*18 check
     * above, so a corrupt row cannot name a macrocell the device lacks. */

    *out_fb = sfb; *out_mc = smc;
    return 1;
}
/* Callers that want strict one-hot behaviour pass NULL, and a multi-hit
 * returns 0 for them. */
static int nonxl_plane_source(const device_t *dev, const unsigned char *fuses,
                              int fb_dest, int pair, int *out_fb, int *out_mc) {
    return nonxl_plane_source_n(dev, fuses, fb_dest, pair, out_fb, out_mc, NULL);
}

/* Which path does this pair take? The two are different signals of the
 * same macrocell:
 *   FastCONNECT (select < 30) -> the source macrocell's pad
 *   identity    (select 30)   -> the source macrocell's own output
 * Returns 1 and fills out_fb/out_mc when the identity path is in use. */
int pair_via_identity(const device_t *dev, const unsigned char *fuses,
                      int fb_dest, int pair, int *out_fb, int *out_mc) {
    static const int rrow[3][5] = {{6,14,22,30,38},{54,62,70,7,15},{31,39,47,55,63}};
    int col, role, b, v = 0;
    if (!dev || dev->is_xl || !fuses || !dev->jed_fuses_per_fb) return 0;
    if (pair < 0 || pair >= 36) return 0;
    /* Only the identity path names the macrocell output. A select value
     * below 30 is a real FastCONNECT index and names the pad, which keeps
     * its pin name. */
    col = 55 + pair/3; role = pair%3;
    for (b = 0; b < 5; b++) {
        long idx = (long)(fb_dest-1)*dev->jed_fuses_per_fb
                 + (long)col*108 + rrow[role][b];
        if (idx < 0) return 0;
        if (fuses[idx]) v |= (1 << b);
    }
    if (v < 30) return 0;                 /* FastCONNECT -> the PAD */
    return nonxl_plane_source(dev, fuses, fb_dest, pair, out_fb, out_mc);
}


/* Formats the resolved source into buf. Returns 1 on success. */
static int nonxl_plane_name(const device_t *dev, const unsigned char *fuses,
                            int fb_dest, int pair, const char *pol,
                            char *buf, size_t buflen) {
    int sfb = 0, smc = 0, pin;
    if (!nonxl_plane_source(dev, fuses, fb_dest, pair, &sfb, &smc)) return 0;
    pin = dev->mc_pin ? dev->mc_pin[(sfb - 1) * 18 + smc - 1] : 0;
    if (pin_is_bga(pin)) {
        char bg[32]; bga_pin_name(pin, bg, sizeof bg);
        snprintf(buf, buflen, "%s%s", pol, bg);
    } else if (pin) {
        pin_lit_name(pol, pin, buf, buflen);
    } else {
        snprintf(buf, buflen, "%sn_FB%dMC%d", pol, sfb, smc);
    }
    return 1;
}

/* Wired-AND UIM lines.
 *
 * The vendor bitmap writer zeroes a UIM line only if it is currently all
 * ones. The first source assigned to a line clears it and sets its own bit;
 * a second source on the same line finds a zero already there, so the line
 * is not cleared again and both bits stand.
 *
 *     N survivors in a written plane row == N members of a wired-AND line
 *
 * vhdl.c joins the members with AND. A member whose FBINV is programmed
 * enters complemented and is marked with '!'.
 *
 * Returns 1 and fills buf with "polA|B|C" when the row is written and has
 * more than one survivor; '|' separates the members. Returns 0 otherwise --
 * a single survivor (nonxl_plane_name()'s job) or a row that was never
 * written, which keeps its placeholder.
 *
 * Each survivor is spelled n_FB<f>MC<m>, the FB output, never the pad: the
 * identity path names the source macrocell's own output, which is what
 * lit_to_expr() produces from this spelling. */
static int nonxl_plane_wired_or_name(const device_t *dev,
                                     const unsigned char *fuses,
                                     int fb_dest, int pair, const char *pol,
                                     char *buf, size_t buflen) {
    int fb_cols, n_extra, c, j, hits = 0, zeros = 0, plane;
    int idx[3 * 108];
    size_t used;

    if (!dev || !fuses || dev->is_xl) return 0;
    if (pair < 0 || pair >= 36) return 0;
    plane = nonxl_pair_plane[pair];
    if (plane < 0) return 0;
    if (dev->jed_fuses_per_fb <= 0) return 0;
    fb_cols = dev->jed_fuses_per_fb / COLUMN_WIDTH;
    n_extra = fb_cols - 72;
    if (n_extra <= 0) return 0;
    if (n_extra * 3 != dev->n_fbs * 18) return 0;   /* (fb_cols-72)*3 == n_fbs*18 */

    for (c = 72; c < fb_cols; c++) {
        const unsigned char *col = device_col(dev, fuses, fb_dest, c);
        for (j = 0; j < 3; j++) {
            if (col[plane + 36 * j] == 1) {
                if (hits < (int)(sizeof idx / sizeof idx[0]))
                    idx[hits] = 3 * (c - 72) + j;
                hits++;
            } else {
                zeros++;
            }
        }
    }

    /* A row is meaningful only once it has been written. An all-ones row
     * carries no local feedback and would name every macrocell as a source. */
    if (zeros == 0) return 0;
    if (hits < 2) return 0;                 /* single survivor: existing path */
    if (hits > (int)(sizeof idx / sizeof idx[0])) return 0;

    /* Every survivor must name a macrocell the device actually has, exactly
     * as the single-survivor path requires. A corrupt row names nothing. */
    for (c = 0; c < hits; c++)
        if (idx[c] < 0 || idx[c] >= dev->n_fbs * 18) return 0;

    used = strlen(pol);
    if (used >= buflen) return 0;
    memcpy(buf, pol, used + 1);
    for (c = 0; c < hits; c++) {
        char one[32]; int mf = idx[c] / 18 + 1, mm = idx[c] % 18 + 1; /* '!': FBINV member */
        int n = snprintf(one, sizeof one, "%s%sn_FB%dMC%d", c ? "|" : "",
                         device_col(dev, fuses, mf, 46)[mc_off(mm)] == 0 ? "!" : "", mf, mm);
        if (n < 0 || (size_t)n >= sizeof one) return 0;
        if (used + (size_t)n + 1 > buflen) return 0;   /* never truncate */
        memcpy(buf + used, one, (size_t)n + 1);
        used += (size_t)n;
    }
    return 1;
}

/* xc9572xl_fb34_lookup: pair -> signal table for XC9572XL, keyed by
 * AND-array pair index alone (independent of destination FB).
 * A separate function so it can also serve as a fallback for devices with
 * an imux_table: those tables list only the selectable mux codes (1-4
 * external, 16-19 internal) and say nothing about an unprogrammed input
 * (code 0). Returns 1 and fills *out_fb, *out_mc, *out_pin if `pair` is a
 * known entry, 0 otherwise. */
static int xc9572xl_fb34_lookup(int pair, int *out_fb, int *out_mc, int *out_pin) {
    static const struct { int pair; int fb; int mc; int pin; } xc9572xl_fb34_pairs[] = {
        /* primary slots */
        {  1, 3, 2,  5 },   /* FB3_MC2/P5   (also FB4_MC2/P19; same slot) */
        {  2, 1, 6, 41 },   /* FB1_MC6/P41  CE routing slot                  */
        {  4, 1,15,  2 },   /* FB1_MC15/P2  (also FB3_MC5/P6, FB4_MC5/P20)  */
        {  4, 3, 5,  6 },   /* FB3_MC5/P6  Q feedback */
        {  6, 1, 8, 42 },   /* FB1_MC8/P42  multi-src: also FB2_MC2/P29      */
        {  7, 3, 8,  7 },   /* FB3_MC8/P7   (also FB4_MC8/P21; same slot) */
        {  8, 2,14, 36 },   /* FB2_MC14/P36 GTS1                             */
        {  8, 4, 9,  0 },   /* FB4_MC9(bur r=2 K=2) multi-src->pair8 */
        { 10, 2, 6, 31 },   /* FB2_MC6/P31  (also FB3_MC11/P12)              */
        { 12, 2,15, 37 },   /* FB2_MC15/P37 multi-src: also FB2_MC8/P32      */
        { 13, 3,14, 13 },   /* FB3_MC14/P13 (also FB4_MC14/P23; same slot) */
        { 13, 4, 5, 20 },   /* FB4_MC5/P20  INPUT path */
        { 13, 2,14,  0 },   /* FB2_MC14(bur r=0 K=4) alt-13 */
        { 15, 3,16, 18 },   /* FB3_MC16/P18                                  */
        { 15, 2,16,  0 },   /* FB2_MC16(bur r=0 K=5) multi-src->pair15 */
        { 16, 1, 2, 39 },   /* FB1_MC2/P39  (also FB3_MC17/P16, FB4_MC17/P28)*/
        { 19, 2,17, 38 },   /* FB2_MC17/P38                                  */
        { 20, 1, 5, 40 },   /* FB1_MC5/P40                                   */
        { 20, 2, 5, 30 },   /* FB2_MC5/P30 multi-src->pair20 */
        { 23, 2, 5, 30 },   /* FB2_MC5/P30                                   */
        { 23, 3,16, 18 },   /* FB3_MC16/P18 alt-23 (RST fallback) */
        { 27, 4,15, 27 },   /* FB4_MC15/P27                                  */
        { 31, 1,14,  1 },   /* FB1_MC14/P1  GCK3                             */
        { 33, 1, 6, 41 },   /* FB1_MC6/P41  DATA routing slot                */
        { 33, 1, 1,  0 },   /* FB1_MC1(bur r=0 K=0) */
        { 38, 3, 9,  8 },   /* FB3_MC9/P8                                    */
        { 41, 4,11, 22 },   /* FB4_MC11/P22 */
        { 41, 1, 9, 43 },   /* FB1_MC9/P43(GCK1) multi-src->pair41 */
        { 41, 2,13,  0 },   /* FB2_MC13(bur r=0 K=4) alt-41 */
        /* pair43: FB3_MC16/P18 alt-43 (primary=pair15), unconfirmed */
        { 43, 3,16, 18 },   /* FB3_MC16/P18 alt-43 hypothesis                 */
        /* TQ100 entries */
        {  6, 4,  7,  0 },   /* FB4_MC7  Q r=0,K=2 TQ100-active P77 */
        { 30, 3,  1,  0 },   /* FB3_MC1  Q r=0,K=0 TQ100-active P41 */
        { 31, 4,  4,  0 },   /* FB4_MC4  Q r=0,K=1 TQ100-active P72 */
        { 53, 3,  7,  0 },   /* FB3_MC7  Q r=0,K=2 TQ100-active P54 */
        /* (pair21/FB3_MC4 and pair37/FB4_MC1 already in table above) */
        /* more TQ100 entries */
        {  9, 3, 10,  0 },   /* FB3_MC10 r=0,K=3 */
        { 12, 3, 13,  0 },   /* FB3_MC13 r=0,K=4 */
        { 38, 4, 10,  0 },   /* FB4_MC10 r=0,K=3 */
        { 12, 4, 13,  0 },   /* FB4_MC13 r=0,K=4 */
        { 26, 4, 16,  0 },   /* FB4_MC16 r=0,K=5 */
        {  2, 3,  3,  0 },   /* FB3_MC3  r=2,K=0 */
        /* FB3_MC6 pair46 and FB3_MC12 pair11 already in table */
        { 20, 4,  3,  0 },   /* FB4_MC3  r=2,K=0 */
        {  5, 4,  6,  0 },   /* FB4_MC6  r=2,K=1 */
        {  8, 4,  9,  0 },   /* FB4_MC9  r=2,K=2 */
        { 17, 3, 18,  0 },   /* FB3_MC18 r=2,K=5 */
        { 11, 4, 12,  0 },   /* FB4_MC12 r=2,K=3 */
        { 53, 4, 18,  0 },   /* FB4_MC18 r=2,K=5 */
        { 44, 1,17,  3 },   /* FB1_MC17/P3                                   */
        { 45, 2,15, 37 },   /* FB2_MC15/P37 alt slot */
        { 48, 1,11, 44 },   /* FB1_MC11/P44 GCK2                             */
        { 49, 2,11, 34 },   /* FB2_MC11/P34 GTS2                             */
        { 51, 3,15, 14 },   /* FB3_MC15/P14                                  */
        { 52, 2, 9, 33 },   /* FB2_MC9/P33  GSR                              */
        /* buried macrocell outputs (no external pin) */
        { 21, 3, 5,  6 },   /* FB3_MC5/P6   INPUT path */
        { 21, 3, 4,  0 },   /* FB3_MC4 buried r=0 K=1 (PAIR21) */
        { 29, 1, 3,  0 },   /* FB1_MC3 buried r=2 K=0 */
        { 37, 4, 1,  0 },   /* FB4_MC1 buried r=0 K=0 */
        { 37, 2, 1,  0 },   /* FB2_MC1(bur r=0 K=0) */
        {  4, 4, 8, 21 },   /* FB4_MC8/P21 r=1 K=2 */
        { 39, 1, 7,  0 },   /* FB1_MC7 buried r=0 K=2 */
        { 34, 3,13,  0 },   /* FB3_MC13 buried r=0 K=4 */
        {  9, 3,10,  0 },   /* FB3_MC10 buried r=0 K=3 */
        {  5, 4, 6,  0 },   /* FB4_MC6  buried r=2 K=1 (PAIR5!) */
        { 11, 3,12,  0 },   /* FB3_MC12 buried r=2 K=3 */
        { 17, 2,18,  0 },   /* FB2_MC18 buried r=2 K=5 */
        { 25, 2, 3,  0 },   /* FB2_MC3  buried r=2 K=0 (PAIR25!) */
        { 26, 1,13,  0 },   /* FB1_MC13 buried r=0 K=4 */
        { 36, 2,13,  0 },   /* FB2_MC13 buried r=0 K=4 */
        { 46, 3, 6,  0 },   /* FB3_MC6  buried r=2 K=1 */
        /* pair 30: FB1_MC2/P39 alt-30 and FB3_MC1 buried */
        /* alternate slots -- same signal, different ISE routing choice */
        { 28, 2, 6, 31 },   /* FB2_MC6/P31  alt-28 (primary=pair 10)         */
        { 30, 1, 2, 39 },   /* FB1_MC2/P39  alt-30 (primary=pair 16)         */
        { 32, 1,17,  3 },   /* FB1_MC17/P3  alt-32 (primary=pair 44)         */
        { 14, 1,16,  0 },   /* FB1_MC16(bur r=0 K=5) alt-14 */
        { 18, 2,17, 38 },   /* FB2_MC17/P38 alt-18 */
        { 53, 1,15,  2 },   /* FB1_MC15/P2  alt-53 (primary=pair 4)          */
        { 22, 1,18,  0 },   /* FB1_MC18 buried r=2 K=5 */
        /* FB1/FB2 buried -> FB3/FB4 cross-pair, 1 literal */
        {  3, 1, 4,  0 },   /* FB1_MC4  bur r=0 K=1 */
        {  9, 1,10,  0 },   /* FB1_MC10 bur r=0 K=3 */
        { 38, 1,12,  0 },   /* FB1_MC12 bur r=2 K=3 */
        {  1, 2, 2, 29 },   /* FB2_MC2/P29 r=1 K=0 */
        { 34, 2, 4,  0 },   /* FB2_MC4  bur r=0 K=1 */
        { 22, 2, 7,  0 },   /* FB2_MC7  bur r=0 K=2 */
        {  7, 2, 8, 32 },   /* FB2_MC8/P32 r=1 K=2 */
        {  9, 2,10,  0 },   /* FB2_MC10 bur r=0 K=3 */
        { 11, 2,12,  0 },   /* FB2_MC12 bur r=2 K=3 */
        {  0, 3,17, 16 },   /* FB3_MC17/P16 r=1 K=5 */
        { 45, 4, 7,  0 },   /* FB4_MC7(bur) r=0 K=2 */
        {  3, 4,14, 23 },   /* FB4_MC14/P23 r=1 K=4 (COMP polarity) */
        { 24, 1,14,  1 },   /* FB1_MC14/P1(GCK3) comb-path */
        { 35, 4, 2, 19 },   /* FB4_MC2/P19 comb-path */
        { 42, 1, 4,  0 },   /* FB1_MC4(bur)  intra-pair slot (cross-pair=pair3) */
        { 47, 1,18,  0 },   /* FB1_MC18(bur) intra-pair self-fb (cross-pair=pair22) */
        { 50, 1, 9, 43 },   /* FB1_MC9/P43   intra-pair self-fb (cross-pair=pair41) */
        { 40, 1, 5, 40 },   /* FB1_MC5/P40   comb intra r=1 slot  (cross=pair20) */
        { -1, 0, 0,  0 },   /* sentinel */
    };
    for (int i = 0; xc9572xl_fb34_pairs[i].pair >= 0; i++) {
        if (xc9572xl_fb34_pairs[i].pair == pair) {
            *out_fb = xc9572xl_fb34_pairs[i].fb;
            *out_mc = xc9572xl_fb34_pairs[i].mc;
            *out_pin = xc9572xl_fb34_pairs[i].pin;
            return 1;
        }
    }
    return 0;
}

/* Per-FB pre-scan for local-feedback disambiguation.
 *
 * The col45/49/50 local-source check answers "which MC is an active local
 * source in this FB" but not which pair it belongs to. It is applied only
 * when an FB has exactly one pair number that fails external resolution
 * across every literal in every macrocell of that FB; then the reference
 * it belongs to is unambiguous.
 *
 * g_suppress_local, when set, makes the local-feedback block skip, so
 * pair_signal_name() can be asked what a pair resolves to by external
 * means alone during the pre-scan. */
static int g_suppress_local = 0;
static int g_fb_unique_unresolved_pair[17]; /* index 1..16; -1 = none/ambiguous */
static int g_fb_prescan_done[17];

static void scan_pt_for_unresolved_pairs(const device_t *dev, const unsigned char *fuses,
                                          int fb, const pt_t *pt, int seen[/*fc_pairs_per_fb*/]) {
    /* SIGNAME_MAX: a wired-AND composite name is much longer than a single
     * name, and this buffer decides which pairs count as unresolved -- a
     * truncated name would change the local-feedback gate for every other
     * pair in the same FB. */
    char buf[SIGNAME_MAX];
    for (int l = 0; l < pt->n_lits; l++) {
        int pair = pt->lits[l].pair;
        if (pair < 0 || pair >= dev->fc_pairs_per_fb) continue;
        if (seen[pair]) continue; /* already classified this pair for this FB */
        g_suppress_local = 1;
        pair_signal_name(dev, fuses, fb, pair, pt->lits[l].is_comp, buf, sizeof buf);
        g_suppress_local = 0;
        seen[pair] = strstr(buf, "_PAIR") ? 2 /* externally unresolved */ : 1 /* resolved */;
    }
}

void compute_fb_unique_unresolved_pairs(const device_t *dev, const chip_t *chip) {
    if (!dev || dev->is_xl || !dev->fc_table) return;
    for (int fb = 1; fb <= dev->n_fbs && fb <= 16; fb++) {
        int seen[64] = {0}; /* 0=unclassified, 1=externally resolved, 2=unresolved */
        for (int mc = 1; mc <= 18; mc++) {
            const mc_t *m = &chip->mc[fb-1][mc-1];
            scan_pt_for_unresolved_pairs(dev, chip->fuses, fb, &m->rst_pt, seen);
            scan_pt_for_unresolved_pairs(dev, chip->fuses, fb, &m->pst_pt, seen);
            scan_pt_for_unresolved_pairs(dev, chip->fuses, fb, &m->ce_pt, seen);
            scan_pt_for_unresolved_pairs(dev, chip->fuses, fb, &m->oe_pt, seen);
            /* The PT clock is held apart from data_pts[] and must be scanned
             * explicitly, or a pair used only as a clock is not counted. */
            if (m->has_clk_pt)
                scan_pt_for_unresolved_pairs(dev, chip->fuses, fb, &m->clk_pt, seen);
            for (int p = 0; p < m->n_data_pts; p++)
                scan_pt_for_unresolved_pairs(dev, chip->fuses, fb, &m->data_pts[p], seen);
        }
        int unique_pair = -1, count = 0;
        for (int p = 0; p < dev->fc_pairs_per_fb && p < 64; p++) {
            if (seen[p] == 2) { unique_pair = p; count++; }
        }
        g_fb_unique_unresolved_pair[fb] = (count == 1) ? unique_pair : -1;
        g_fb_prescan_done[fb] = 1;
    }
}

static const chip_t *g_named_chip = NULL;   /* chip being named, for the identity-path test below */

/* Does this pair take the identity path to a macrocell that carries
 * logic but does not drive its pad? Then the signal is that macrocell's
 * own output, not the pad; naming it n_p<pin> would point every consumer
 * at an input port instead of the register. */
static int identity_names_mc_output(const device_t *dev,
                                    const unsigned char *fuses,
                                    int fb_dest, int pair, int *gf, int *gm) {
    if (!g_named_chip || !pair_via_identity(dev, fuses, fb_dest, pair, gf, gm))
        return 0;
    if (*gf < 1 || *gf > dev->n_fbs || *gm < 1 || *gm > MACROCELLS_PER_FB)
        return 0;
    /* The identity path wins outright: select >= 30 is not a FastCONNECT
     * index, so fc_table is not consulted. This covers buried sources too. */
    return 1;
}

static const char *pair_signal_name_raw(const device_t *dev, const unsigned char *fuses,
                                     int fb_dest, int pair, int is_comp,
                                     char *buf, int buflen) {
    {   int gf = 0, gm = 0;
        if (!dev->is_xl && fuses
            && identity_names_mc_output(dev, fuses, fb_dest, pair, &gf, &gm)) {
            snprintf(buf, buflen, "%sn_FB%dMC%d", is_comp ? "NOT_" : "", gf, gm);
            return buf;
        }
    }
    if (!dev->is_xl && dev->fc_table && pair < dev->fc_pairs_per_fb) {
        /* Non-XL: look up pair in the per-destination-FB fc_table.
         * Each slot may have several wired sources (n_sources >= 1).
         *   1. n_sources == 1 -> that is the signal.
         *   2. n_sources > 1  -> select-code match, or the clk-mode
         *      heuristic below; otherwise report all sources as
         *      "MC_a|MC_b|..." for diagnostics.
         *   3. n_sources == 0 -> pair not characterised; emit "PAIRn". */
        /* The bound is the table's own first dimension, which device.c sizes
         * per device (up to 16 FBs). */
        int dest_fb_idx = (fb_dest >= 1 && fb_dest <= dev->n_fbs) ? fb_dest-1 : 0;
        const fc_entry_t *fe = &dev->fc_table[dest_fb_idx][pair];
        const char *pol = is_comp ? "NOT_" : "";

        /* Local (same-FB) feedback.
         *
         * Each FB has local feedback paths that let its outputs drive its
         * own AND array without going through FastCONNECT.
         *
         * Each candidate MC has a one-hot enable fuse pair at column 49/50
         * (complement/true), row mc_off(candidate), and column 45 must be 0
         * at that row as well (the MC's output is routed via local
         * feedback). Same columns and mc_off() on XC9572, XC95108 and
         * XC95144. Pair 0 is the local-feedback slot for every macrocell
         * in an FB, and several same-FB destinations may read different
         * local sources at once.
         *
         * Applied only when the destination FB has exactly one unresolved
         * pair reference (compute_fb_unique_unresolved_pairs above).
         * XC9536 (n_fbs==2) is excluded: this mechanism gives wrong names
         * there. */
        if (fuses && !g_suppress_local &&
            (dev->n_fbs == 4 || dev->n_fbs == 6 || dev->n_fbs == 8) &&
            g_fb_prescan_done[fb_dest] && g_fb_unique_unresolved_pair[fb_dest] == pair) {
            int local_mc = 0, local_hits = 0;
            for (int mc = 1; mc <= 18; mc++) {
                const unsigned char *c45 = device_col(dev, fuses, fb_dest, 45);
                const unsigned char *c49 = device_col(dev, fuses, fb_dest, 49);
                const unsigned char *c50 = device_col(dev, fuses, fb_dest, 50);
                if (c45[mc_off(mc)] == 0 && c49[mc_off(mc)] == 0 && c50[mc_off(mc)] == 0) {
                    local_mc = mc; local_hits++;
                }
            }
            if (local_hits == 1) {
                /* Local feedback is an FB output too. */
                snprintf(buf, buflen, "%sn_FB%dMC%d", pol, fb_dest, local_mc);
                return buf;
            }
            /* local_hits == 0: no local source active, fall through to
             * external resolution below. local_hits > 1: ambiguous/
             * impossible -- also fall through rather than guess. */
        }

        if (fe->n_sources == 0) {
            /* Try the identity-region table before giving up. */
            if (nonxl_plane_name(dev, fuses, fb_dest, pair, pol, buf, buflen))
                return buf;
            /* A written plane row with several survivors is several sources
             * wired onto this pair (wired-AND). */
            if (nonxl_plane_wired_or_name(dev, fuses, fb_dest, pair, pol,
                                          buf, (size_t)buflen))
                return buf;
            snprintf(buf, buflen, "%sFB%d_PAIR%d", pol, fb_dest, pair);
        } else if (fe->n_sources == 1) {
            /* Unambiguous: single source */
            int sfb = fe->src[0].fb, smc = fe->src[0].mc;
            int pin = dev->mc_pin ? dev->mc_pin[(sfb-1)*18+smc-1] : 0;
            if (pin_is_bga(pin)) { char _bg[32]; bga_pin_name(pin,_bg,sizeof _bg); snprintf(buf,buflen,"%s%s",pol,_bg); }
            else if (pin) pin_lit_name(pol, pin, buf, buflen);
            else snprintf(buf, buflen, "%sn_FB%dMC%d", pol, sfb, smc);
        } else {
            /* Several sources. Two resolution paths:
             *
             * (1) fe->n_select_bits > 0: a select-code fuse field exists
             *     for this pair. Read fuses[] at each select_bits[]
             *     position and compare with every candidate's
             *     select_code[]; a unique match identifies the active
             *     source.
             *
             * (2) fe->n_select_bits == 0: no select code for this entry.
             *     Falls back to the clk-mode heuristic below, which is
             *     unverified -- see the fc_entry_t comment in device.h. */
            if (fuses && fe->n_select_bits > 0) {
                unsigned char observed[FC_MAX_SELECT_BITS];
                int bi;
                for (bi = 0; bi < fe->n_select_bits; bi++)
                    observed[bi] = fuses[fe->select_bits[bi]];

                int match_idx = -1, n_matches = 0;
                for (int si = 0; si < fe->n_sources; si++) {
                    int all_match = 1;
                    for (bi = 0; bi < fe->n_select_bits; bi++)
                        if (fe->src[si].select_code[bi] != observed[bi]) { all_match = 0; break; }
                    if (all_match) { match_idx = si; n_matches++; }
                }

                if (n_matches == 1) {
                    int sfb = fe->src[match_idx].fb, smc = fe->src[match_idx].mc;
                    int pin = dev->mc_pin ? dev->mc_pin[(sfb-1)*18+smc-1] : 0;
                    if (pin_is_bga(pin)) { char _bg[32]; bga_pin_name(pin,_bg,sizeof _bg); snprintf(buf,buflen,"%s%s",pol,_bg); }
                    else if (pin) pin_lit_name(pol, pin, buf, buflen);
                    else snprintf(buf, buflen, "%sn_FB%dMC%d", pol, sfb, smc);
                } else {
                    /* No match among the known candidates (an unlisted source is
                     * active), or more than one match (data error): no
                     * guess. The identity region can still name a same-FB
                     * local-feedback source; try that first. */
                    /* The select value may name one of this pair's local feedback
                     * entries, which the candidate list does not cover. */
                    if (nonxl_feedback_name(dev, fe, fuses, fb_dest, pair,
                                            pol, buf, buflen))
                        return buf;
                    if (nonxl_plane_name(dev, fuses, fb_dest, pair, pol, buf, buflen))
                        return buf;
                                        {
                        /* Marked guess for a multi-hit local-feedback pair: select value
                         * 30 (local feedback) but the identity plane names 2..14
                         * candidates. Emits the first candidate with "?NofM";
                         * the VHDL emitter refuses any name containing '?'. */
                        int gf = 0, gm = 0, nc = 0;
                        /* Disabled: one of 2..14 equally supported candidates is not a decode. */
        if (0 && !dev->is_xl
            && nonxl_plane_source_n(dev, fuses, fb_dest, pair, &gf, &gm, &nc)
            && nc > 1) {
                            char sg[64];
                            signal_name(dev, gf, gm, sg, sizeof sg);
                            snprintf(buf, buflen, "%s%s?1of%d", pol, sg, nc);
                        } else {
                            /* No resolution by elimination here. The select value is read
                             * LSB-first; 14 is local feedback, which carries no
                             * source identity, so "matches no known source"
                             * does not mean "the one source not listed".
                             *
                             * fc_lfb_xc95108[] is kept as data: a subset
                             * constraint on local feedback, not a resolution.
                             *
                             * A plane row with several survivors lists sources
                             * that all drive this pair; the wired combination of
                             * every survivor is emitted above. */
                            if (nonxl_plane_wired_or_name(dev, fuses, fb_dest,
                                                          pair, pol, buf,
                                                          (size_t)buflen))
                                return buf;
                            snprintf(buf, buflen, "%sFB%d_PAIR%d",
                                     pol, fb_dest, pair);
                        }
                    }
                }
            } else {
            /* --- clk-mode heuristic, for entries without select-code data --- */
            int best_sfb = 0, best_smc = 0, best_pin = 0;
            int n_active = 0;
            char active_names[128] = "";
            char tmp[64];

            for (int si = 0; si < fe->n_sources; si++) {
                int sfb = fe->src[si].fb, smc = fe->src[si].mc;
                /* Check if this MC has a registered clock in the JED.
                 * Non-XL CLK mode: cols 35,36,37 (B0,B1,B2). Active when any bit == 0.
                 * We test col 37 (CLK_B0): if any GCK bit cleared, it's registered. */
                int is_reg = 0;
                if (fuses) {
                    /* Quick check: CLK_B2 (col35) and CLK_B1 (col36): both 1 = COMB.
                     * If either is 0, the MC has a clocked (registered) output. */
                    const unsigned char *cb2 = device_col(dev, fuses, sfb, 35);
                    const unsigned char *cb1 = device_col(dev, fuses, sfb, 36);
                    int b2 = cb2[mc_off(smc)]; /* 0 = active (non-XL F1 default) */
                    int b1 = cb1[mc_off(smc)];
                    is_reg = (b2 == 0 || b1 == 0); /* at least one CLK bit active */
                }
                int pin = dev->mc_pin ? dev->mc_pin[(sfb-1)*18+smc-1] : 0;
                if (pin_is_bga(pin)) { char _bg[32]; bga_pin_name(pin,_bg,sizeof _bg); snprintf(tmp,sizeof tmp,"%s%s",pol,_bg); }
                else if (pin) pin_lit_name(pol, pin, tmp, (int)sizeof tmp);
                else snprintf(tmp, sizeof tmp, "%sn_FB%dMC%d", pol, sfb, smc);
                if (is_reg && n_active == 0) { best_sfb=sfb; best_smc=smc; best_pin=pin; }
                if (is_reg) n_active++;
                if (active_names[0]) strncat(active_names, "|",
                                             sizeof(active_names)-strlen(active_names)-1);
                strncat(active_names, tmp,
                        sizeof(active_names)-strlen(active_names)-1);
            }

            if (n_active == 1) {
                /* Exactly one active registered MC -> unambiguous */
                if (pin_is_bga(best_pin)) { char _bg[32]; bga_pin_name(best_pin,_bg,sizeof _bg); snprintf(buf,buflen,"%s%s",pol,_bg); }
                else if (best_pin) pin_lit_name(pol, best_pin, buf, buflen);
                else
                    snprintf(buf, buflen, "%sn_FB%dMC%d", pol, best_sfb, best_smc);
            } else {
                /* 0 active (all combinatorial or fuses==NULL) or >1 active:
                 * emit all source names separated by '|' for diagnostics */
                snprintf(buf, buflen, "%s", active_names);
            }
            }
        }
    } else if (dev->is_xl) {
        /* The vendor's own <imux> table resolves any XL FB input outright, and it
         * is the only source that tells a pad (low G codes) from the macrocell
         * output (G16+). It sits ahead of every device-specific path below:
         * those are gated on n_fbs >= 4 and on a hand-built imux_table, so
         * XC9536XL never reaches them. */
        {   int vfb = 0, vmc = 0, vis = 0;
            int v_input = pair + 1;
            int v_code  = (v_input >= 1 && v_input <= 54)
                        ? xl_imux_abyte(dev, fuses, fb_dest, v_input) : -1;
            if (getenv("XC_DUMP_IMUX") && v_code >= 0)
                fprintf(stderr, "IMUX fb=%d input=%d code=%d\n",
                        fb_dest, v_input, v_code);
            if (v_code >= 0
                && xl_imux2_lookup(dev, v_input, v_code, &vfb, &vmc, &vis)
                && vfb >= 1 && vfb <= dev->n_fbs
                && vmc >= 1 && vmc <= MACROCELLS_PER_FB) {
                const char *vpol = is_comp ? "NOT_" : "";
                /* G16+ selects the macrocell output; naming its pad would point
                 * every consumer at an input port. The pad is named only for
                 * the low G codes, which select the pad. */
                int vp = (!vis && dev->mc_pin)
                       ? dev->mc_pin[(vfb-1)*18 + vmc-1] : 0;
                if (vp > 0) { pin_lit_name(vpol, vp, buf, buflen); return buf; }
                /* G16+ selects the macrocell output. signal_name() returns the
                 * pin name for any macrocell that owns a pad, which would
                 * merge the output with the pad. Where the source does not
                 * drive its pad, the two are different signals and the output
                 * is named explicitly. */
                /* G16+ is the macrocell output, unconditionally -- that is what
                 * <imux label> says and what the feedback path carries,
                 * whatever the decoded oe_mode. State the source exactly here
                 * and let the emitter choose the spelling. */
                if (vis) {
                    snprintf(buf, buflen, "%sn_FB%dMC%d", vpol, vfb, vmc);
                    return buf;
                }
                { char sb[64]; signal_name(dev, vfb, vmc, sb, sizeof sb);
                  snprintf(buf, buflen, "%s%s", vpol, sb); }
                return buf;
            }
        }
        /* XL: pair = column/2 = (AND-array band - 1)/2 (true/odd bands).
         *
         * On XC9536XL FB2 the AND-array input N (= pair+1) does not map
         * directly to the ISE imux slot M: the routing hardware maps each
         * slot M (with a given choice code) to a fixed AND-array band B.
         * So invert that mapping: given band B (= 2*N-1, N = pair+1), find
         * slot M, then read the code from the JED at A-byte col = 49+M.
         *
         * For FB1, input = pair+1 and col = 49+input. */
        int input = pair + 1;   /* AND-array input N (1-based) */
        int ise_slot = input;   /* ISE IMUX slot M (may differ from N for FB2) */

        /* XC9536XL FB2: remap AND-array input N -> ISE imux slot M.
         * Index = pair (N-1); value = slot M (1-based); 0 = no mapping. */
        static const int xl9536_fb2_n_to_m[54] = {
            /*N= 1*/ 9,  /*N= 2*/ 11, /*N= 3*/ 17, /*N= 4*/ 0,
            /*N= 5*/ 13, /*N= 6*/ 12, /*N= 7*/ 0,  /*N= 8*/ 8,
            /*N= 9*/ 14, /*N=10*/ 7,  /*N=11*/ 15, /*N=12*/ 6,
            /*N=13*/ 10, /*N=14*/ 5,  /*N=15*/ 1,  /*N=16*/ 4,
            /*N=17*/ 2,  /*N=18*/ 3,  /*N=19*/ 16,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
        };
        /* XC9536XL FB2: (ISE slot M, ISE code) -> physical pin for PC44.
         * Row format: {ise_code, pin_number}; pin_number=0 means buried or
         * unknown. code=0 is the hardwired default (no A-byte fuse written). */
        static const struct { int code; int pin; } xl9536_fb2_m_code_pin[18][4] = {
            /* M= 1 */ {{8,11},{2,1}, {0,1},{0,0}},  /* P11 (c=8) or P1 (c=2); c=0 default=P1 */
            /* M= 2 */ {{8,12},{2,22},{0,0},{0,0}},  /* P12 (c=8) or P22 (c=2) */
            /* M= 3 */ {{8,13},{2,2}, {0,0},{0,0}},  /* P13 (c=8) or P2  (c=2) */
            /* M= 4 */ {{0,14},{2,24},{0,0},{0,0}},  /* P14 default; P24 (c=2) */
            /* M= 5 */ {{8,18},{2,5}, {0,0},{0,0}},  /* P18 (c=8) or P5  (c=2) */
            /* M= 6 */ {{8,19},{2,20},{0,0},{0,0}},  /* P19 (c=8) or P20 (c=2) */
            /* M= 7 */ {{0,20},{2,3}, {0,0},{0,0}},  /* P20 default; P3  (c=2) */
            /* M= 8 */ {{8,22},{2,19},{0,0},{0,0}},  /* P22 (c=8) or P19 (c=2) */
            /* M= 9 */ {{8,24},{1,43},{2,4},{0,0}},  /* P24 (c=8), P43 (c=1), P4 (c=2) */
            /* M=10 */ {{8,2}, {2,18},{0,0},{0,0}},  /* P2  (c=8) or P18 (c=2) */
            /* M=11 */ {{8,43},{2,8}, {0,8},{0,0}},  /* P43 (c=8) or P8 (c=2); c=0 default=P8 */
            /* M=12 */ {{8,4}, {2,14},{0,0},{0,0}},  /* P4  (c=8) or P14 (c=2) */
            /* M=13 */ {{2,6}, {8,0}, {0,6},{0,0}},  /* P6 (c=2,c=0 = GCK2); c=8 unknown */
            /* M=14 */ {{8,7}, {2,13},{0,0},{0,0}},  /* P7  (c=8, GCK3) or P13 (c=2) */
            /* M=15 */ {{8,8}, {2,7}, {0,0},{0,0}},  /* P8  (c=8) or P7  (c=2) */
            /* M=16 */ {{8,9}, {2,12},{0,0},{0,0}},  /* P9  (c=8) or P12 (c=2) */
            /* M=17 */ {{8,3}, {2,9}, {0,0},{0,0}},  /* P3  (c=8, RST) or P9 (c=2) */
            /* M=18 */ {{8,0}, {2,11},{0,0},{0,0}},  /* P11 (c=2); c=8 unknown */
        };

        /* XC9572XL FB3/FB4: direct pair -> pin table.
         * Pair N is AND-array col 2N (complement) and 2N+1 (true).
         * Pairs not in the table are labelled FB3_PAIR_N. */
        if (dev->n_fbs >= 4 && !dev->imux_table && fb_dest >= 1 && pair < 54) {
            const char *pol = is_comp ? "NOT_" : "";
            int lu_fb, lu_mc, lu_pin;
            if (xc9572xl_fb34_lookup(pair, &lu_fb, &lu_mc, &lu_pin)) {
                /* lu_pin is specific to the VQ44 package and gives pins that do
                 * not exist on VQ64 or TQ100. signal_name() returns the right
                 * pin for whichever package dev is. */
                char sigbuf[64];
                (void)lu_pin;
                signal_name(dev, lu_fb, lu_mc, sigbuf, sizeof sigbuf);
                snprintf(buf, buflen, "%s%s", pol, sigbuf);
                return buf;
            }
            /* Unknown or ambiguous pair: emit label with A-byte code for future disambiguation */
            {
                int abcode = read_xl_abyte_pb2(dev, fuses, fb_dest, pair);
                if (abcode)
                    snprintf(buf, buflen, "%sFB%d_PAIR%d(ab=%d)", pol, fb_dest, pair, abcode);
                else
                    snprintf(buf, buflen, "%sFB%d_PAIR%d", pol, fb_dest, pair);
            }
            return buf;
        }

        if (dev->n_fbs >= 4 && dev->imux_table && fb_dest >= 1 && pair < 54) {
            /* XC9572XL (and any n_fbs>=4 device with a real imux_table):
             * general destination-FB path, using the band-interleave A-byte
             * formula (xl_imux_abyte) and the device's imux table.
             * Not the XC9536XL 2-FB block below: its self_code retry and
             * xl9536_fb2_n_to_m/m_code_pin tables are specific to the PC44
             * package of a 2-FB device.
             *
             * The imux table has no code 0 (default) entries. When nothing
             * resolves, the input is reported as INPUT_N(imux=C) -- the same
             * format XC9536XL uses -- rather than an FB#_PAIR# placeholder. */
            const char *pol = is_comp ? "NOT_" : "";
            int input_n = pair + 1;
            int imux_code = xl_imux_abyte(dev, fuses, fb_dest, input_n);
            int idx = (fb_dest - 1) * XL_INPUTS_PER_FB + (input_n - 1);
            const imux_entry_t *entries = dev->imux_table[idx];
            int cnt = dev->imux_count ? dev->imux_count[idx] : 0;
            int resolved = 0;
            for (int ei = 0; ei < cnt && entries; ei++) {
                if (entries[ei].mux_code == (unsigned char)imux_code) {
                    if (entries[ei].pin > 0) {
                        pin_lit_name(pol, entries[ei].pin, buf, buflen);
                    } else if (entries[ei].mc > 0) {
                        char sigbuf[64];
                        signal_name(dev, entries[ei].fb, entries[ei].mc,
                                    sigbuf, sizeof sigbuf);
                        snprintf(buf, buflen, "%s%s", pol, sigbuf);
                    } else {
                        snprintf(buf, buflen, "%sfb%dmc%d",
                                 pol, entries[ei].fb, entries[ei].mc);
                    }
                    resolved = 1;
                    break;
                }
            }
            if (!resolved) {
                /* the .chp-derived table */
                int d_fb = 0, d_mc = 0;
                if (xl9572_derived_src(input_n, imux_code, &d_fb, &d_mc)) {
                    char sigbuf2[64];
                    signal_name(dev, d_fb, d_mc, sigbuf2, sizeof sigbuf2);
                    snprintf(buf, buflen, "%s%s", pol, sigbuf2);
                    resolved = 1;
                }
            }
            /* xc9572xl_fb34_lookup() is not consulted here; a placeholder
             * is emitted instead. */
            if (0) {
                int lu_fb, lu_mc, lu_pin;
                if (xc9572xl_fb34_lookup(pair, &lu_fb, &lu_mc, &lu_pin)) {
                    /* lu_pin is VQ44-specific; see the same note on the
                     * (!dev->imux_table) branch above. */
                    char sigbuf[64];
                    (void)lu_pin;
                    signal_name(dev, lu_fb, lu_mc, sigbuf, sizeof sigbuf);
                    snprintf(buf, buflen, "%s%s", pol, sigbuf);
                    resolved = 1;
                }
            }
            /* Marked guess disabled: fb2's code-2 entry would be borrowed for a
             * code the vendor does not define on fb1, and a guess can be
             * wrong. Kept behind `if (0)` so the mechanism and its self-loop
             * guard stay visible. */
            if (0 && !resolved && imux_code == 2 && dev->is_xl && dev->n_fbs == 2
                && g_consumer_fb) {
                /* Marked guess: fb1 has no code-2 source at this input, so fb2's
                 * code-2 entry for the same input is the only candidate. It is
                 * emitted with a trailing '?'. The self-loop case is refused
                 * inside xl9536_code2_guess(). */
                int g_fb = 0, g_mc = 0;
                if (xl9536_code2_guess(fb_dest, input_n,
                                       g_consumer_fb, g_consumer_mc,
                                       &g_fb, &g_mc)) {
                    char sigbuf[64];
                    signal_name(dev, g_fb, g_mc, sigbuf, sizeof sigbuf);
                    snprintf(buf, buflen, "%s%s?", pol, sigbuf);
                    resolved = 1;
                }
            }
            if (!resolved) {
                if (imux_code)
                    snprintf(buf, buflen, "%sINPUT%d(imux=%d)", pol, input_n, imux_code);
                else
                    snprintf(buf, buflen, "%sINPUT%d", pol, input_n);
            }
            return buf;
        }

        if (dev->n_fbs > 1 && dev->n_fbs <= 2 && fb_dest >= 1 && pair < 54) {
            /* XC9536XL IMUX (both FB1 and FB2 receivers).
             * The shared AND-array A-byte encoding is identical for all receivers
             * in the 2-FB device; the n_to_m / m_code_pin path that resolves
             * FB2-dest signals applies equally to FB1-dest. */
            const char *pol = is_comp ? "NOT_" : "";
            int input_n = pair + 1;
            int self_col  = (input_n <= 27) ? (49 + input_n) : (22 + input_n);
            int self_bside = (input_n > 27);
            int self_code = 0;
            (void)self_col; (void)self_bside;
            self_code = xl_imux_abyte(dev, fuses, fb_dest, input_n);
            if (self_code && dev->imux_table) {
                int self_idx = (fb_dest-1)*XL_INPUTS_PER_FB + (input_n-1);
                const imux_entry_t *self_ent = dev->imux_table[self_idx];
                int self_cnt = dev->imux_count ? dev->imux_count[self_idx] : 0;
                int self_resolved = 0;
                for (int ei = 0; ei < self_cnt && self_ent; ei++) {
                    if (self_ent[ei].mux_code == (unsigned char)self_code) {
                        if (self_ent[ei].pin > 0)
                            pin_lit_name(pol, self_ent[ei].pin, buf, buflen);
                        else if (self_ent[ei].mc > 0) {
                            char sigbuf[64];
                            signal_name(dev, self_ent[ei].fb, self_ent[ei].mc,
                                        sigbuf, sizeof sigbuf);
                            snprintf(buf, buflen, "%s%s", pol, sigbuf);
                        } else
                            snprintf(buf, buflen, "%sfb%dmc%d", pol,
                                     self_ent[ei].fb, self_ent[ei].mc);
                        self_resolved = 1;
                        break;
                    }
                }
                if (self_resolved) return buf;
                /* Code not found: try code=0 (contamination fallback) */
                if (self_code) {
                    for (int ei = 0; ei < self_cnt && self_ent; ei++) {
                        if (self_ent[ei].mux_code == 0) {
                            if (self_ent[ei].pin > 0)
                                pin_lit_name(pol, self_ent[ei].pin, buf, buflen);
                            else if (self_ent[ei].mc > 0) {
                                char sigbuf[64];
                                signal_name(dev, self_ent[ei].fb, self_ent[ei].mc,
                                            sigbuf, sizeof sigbuf);
                                snprintf(buf, buflen, "%s%s", pol, sigbuf);
                            } else
                                snprintf(buf, buflen, "%sfb%dmc%d", pol,
                                         self_ent[ei].fb, self_ent[ei].mc);
                            return buf;  /* contamination resolved via code=0 */
                        }
                    }
                }
            }
            /* DEBUG: trace pair=4 */
            int m = xl9536_fb2_n_to_m[pair];
            if (m > 0 && m <= 18) {
                /* Read ISE IMUX A-byte at col=49+M (band 50..67 for M=1..18) */
                int col_m = 49 + m;
                int ise_code = 0;
                /* IMUX A-byte is always in the fb=1 (first page) region of the band,
                 * regardless of which FB's signals we're decoding. */
                const unsigned char *ac = device_col(dev, fuses, 1, col_m);
                /* Bit offsets:  off=6 (bit0=1), off=14 (bit1=2), off=22 (bit2=4),
                 *                off=30 (bit3=8), off=38 (bit4=16)              */
                for (int bit = 0; bit < 5; bit++)
                    ise_code |= (ac[bit*8+6] << bit);
                /* Match code to pin */
                const char *pol = is_comp ? "NOT_" : "";
                int pin = 0;
                for (int k = 0; k < 4; k++) {
                    int ec = xl9536_fb2_m_code_pin[m-1][k].code;
                    int ep = xl9536_fb2_m_code_pin[m-1][k].pin;
                    /* code=0 in table = hardwired default (match when ise_code==0) */
                    if ((ec == 0 && ise_code == 0) || (ec != 0 && ec == ise_code)) {
                        pin = ep;
                        break;
                    }
                }
                if (pin > 0) {
                    pin_lit_name(pol, pin, buf, buflen);
                    return buf;  /* resolved via n_to_m */
                }
                /* pin==0: fall through to imux_table (VQ44 default mux=0).
                 * The n_to_m ISE-slot A-byte is PC44-specific; for VQ44 the
                 * actual mux code lives in the fallback path (fb_dest=2, always
                 * zero in XC9536XL, so code=0 = default routing). */
            }
        }

        /* Fallback: original logic for FB1 or non-XC9536XL devices */
        int col, is_b_side;
        if (input <= 27) { col = 49 + input; is_b_side = 0; }
        else             { col = 22 + input; is_b_side = 1; }

        (void)col; (void)is_b_side;
        int imux_code = xl_imux_abyte(dev, fuses, fb_dest, input);

        /* If IMUX lookup table available, resolve to pin/MC name */
        const char *pol = is_comp ? "NOT_" : "";
        int resolved = 0;
        if (dev->imux_table) {   /* mux_code=0 allowed: VQ44 self-routing */
            int idx = (fb_dest - 1) * XL_INPUTS_PER_FB + (ise_slot - 1);
            const imux_entry_t *entries = dev->imux_table[idx];
            int cnt = dev->imux_count ? dev->imux_count[idx] : 0;
            for (int ei = 0; ei < cnt && entries; ei++) {
                if (entries[ei].mux_code == (unsigned char)imux_code) {
                    if (entries[ei].pin > 0) {
                        pin_lit_name(pol, entries[ei].pin, buf, buflen);
                    } else if (entries[ei].mc > 0) {
                        /* pin=0 with mc>0: shared table entry -> look up the pin
                         * in the device's package-specific mc_pin table */
                        char sigbuf[64];
                        signal_name(dev, entries[ei].fb, entries[ei].mc,
                                    sigbuf, sizeof sigbuf);
                        snprintf(buf, buflen, "%s%s", pol, sigbuf);
                    } else {
                        /* truly buried MC with no package pin */
                        snprintf(buf, buflen, "%sfb%dmc%d",
                                 pol, entries[ei].fb, entries[ei].mc);
                    }
                    resolved = 1;
                    break;
                }
            }
        }
        if (!resolved) {
            /* Contamination fallback: dense designs can corrupt the IMUX A-byte
             * when an AND-array fuse at the same physical position as the A-byte
             * read offset gets set by an unrelated product term.
             * If the code is non-zero but unresolvable, try code=0 (the default
             * routing that applies when ISE does not write an explicit A-byte). */
            if (imux_code && dev->imux_table) {
                int idx2 = (fb_dest - 1) * XL_INPUTS_PER_FB + (ise_slot - 1);
                const imux_entry_t *e0 = dev->imux_table[idx2];
                int c0 = dev->imux_count ? dev->imux_count[idx2] : 0;
                for (int ei = 0; ei < c0 && e0; ei++) {
                    if (e0[ei].mux_code == 0) {
                        /* code=0 fallback -- contamination likely */
                        if (e0[ei].pin > 0)
                            pin_lit_name(pol, e0[ei].pin, buf, buflen);
                        else if (e0[ei].mc > 0) {
                            char sigbuf[64];
                            signal_name(dev, e0[ei].fb, e0[ei].mc,
                                        sigbuf, sizeof sigbuf);
                            snprintf(buf, buflen, "%s%s", pol, sigbuf);
                        } else
                            snprintf(buf, buflen, "%sfb%dmc%d",
                                     pol, e0[ei].fb, e0[ei].mc);
                        resolved = 1;
                        break;
                    }
                }
            }
            if (!resolved && dev->imux_table && fb_dest >= 1 && ise_slot >= 1
                && ise_slot <= XL_INPUTS_PER_FB) {
                /* Use the device's own per-package imux table before the flat
                 * derived table.
                 *
                 * The flat xl9536_derived table is indexed by (slot, code)
                 * only. Codes 1, 16 and 17 give the same source for
                 * destination FB1 and FB2, which is why a flat table works
                 * for them; code 2 can differ between destination FBs, which
                 * a flat table cannot express.
                 *
                 * dev->imux_table is indexed (fb_dest-1)*54 + (slot-1), so it
                 * carries the distinction. */
                int idx = (fb_dest - 1) * XL_INPUTS_PER_FB + (ise_slot - 1);
                const imux_entry_t *ent = dev->imux_table[idx];
                int cnt = dev->imux_count ? dev->imux_count[idx] : 0;
                for (int ei = 0; ei < cnt && ent; ei++) {
                    if (ent[ei].mux_code != (unsigned char)imux_code) continue;
                    if (ent[ei].fb < 1 || ent[ei].mc < 1) break;
                    char sigbuf[64];
                    signal_name(dev, ent[ei].fb, ent[ei].mc, sigbuf, sizeof sigbuf);
                    snprintf(buf, buflen, "%s%s", pol, sigbuf);
                    resolved = 1;
                    break;
                }
            }
            if (!resolved && dev->n_fbs == 2) {
                /* FB-indexed vendor input-choice table first. */
                int efb = 0, emc = 0;
                if (xl9536_fbimux_src(fb_dest, ise_slot, imux_code, &efb, &emc)) {
                    char sigbuf[64];
                    signal_name(dev, efb, emc, sigbuf, sizeof sigbuf);
                    snprintf(buf, buflen, "%s%s", pol, sigbuf);
                    resolved = 1;
                }
            }
            if (!resolved) {
                /* derived table, codes 1/16/17 only */
                int dfb = 0, dmc = 0;
                if (xl9536_derived_src(ise_slot, imux_code, &dfb, &dmc)) {
                    char sigbuf[64];
                    signal_name(dev, dfb, dmc, sigbuf, sizeof sigbuf);
                    snprintf(buf, buflen, "%s%s", pol, sigbuf);
                    resolved = 1;
                }
            }
            /* Marked guess disabled: fb2's code-2 entry would be borrowed for a
             * code the vendor does not define on fb1, and a guess can be
             * wrong. Kept behind `if (0)` so the mechanism and its self-loop
             * guard stay visible. */
            if (0 && !resolved && imux_code == 2 && dev->is_xl && dev->n_fbs == 2
                && g_consumer_fb) {
                /* Marked guess -- see xl9536_code2_guess(). This is the site that
                 * fires for XC9536XL; the other INPUT%d emission in this
                 * function is a different branch. */
                int g_fb = 0, g_mc = 0;
                if (xl9536_code2_guess(fb_dest, input,
                                       g_consumer_fb, g_consumer_mc,
                                       &g_fb, &g_mc)) {
                    char sigbuf[64];
                    signal_name(dev, g_fb, g_mc, sigbuf, sizeof sigbuf);
                    snprintf(buf, buflen, "%s%s?", pol, sigbuf);
                    resolved = 1;
                }
            }
            if (!resolved) {
                if (imux_code)
                    snprintf(buf, buflen, "%sINPUT%d(imux=%d)", pol, input, imux_code);
                else
                    snprintf(buf, buflen, "%sINPUT%d", pol, input);
            }
        }
    } else {
        snprintf(buf, buflen, "%sFB%d_pair%d", is_comp ? "NOT_" : "", fb_dest, pair);
    }
    return buf;
}

/* ---- PT scanning in a single AND-array column ---- */

static int __attribute__((unused)) scan_pt_slot(const device_t *dev, const unsigned char *fuses,
                         int fb, int n_cols, int offset, pt_t *pt) {
    pt->col_offset = offset;
    pt->is_tail    = (offset >= 72);
    pt->n_lits     = 0;

    for (int c = 0; c < n_cols; c++) {
        const unsigned char *col = device_col(dev, fuses, fb, c);
        if (col_is_idle(dev, col)) continue;  /* skip default columns */
        /* XC9500XL: cols 6 and 7 (bands 7-8) are USER BITS (entity name).
         * These appear as spurious signal connections at PT slot positions.
         * Skip them during PT scanning to avoid tautological ghost MCs. */
        if (dev->is_xl && (c == 6 || c == 7)) continue;
        if (fuse_connected(col, offset)) {    /* 1 = signal IS in this PT */
            if (pt->n_lits < MAX_LITS) {
                pt->lits[pt->n_lits].pair    = c / 2;
                pt->lits[pt->n_lits].is_comp = (c % 2 == 0);
                pt->n_lits++;
            }
        }
    }
    return (pt->n_lits > 0);
}

/* scan_pt_slot_pg: same as scan_pt_slot but takes a raw AND-array page (1 or 2)
 * instead of a function-block number. For XC9536XL (n_fbs=2) the AND array is
 * interleaved across both pages; use xl_and_loc() to determine page+offset.
 * allow_pair3 is kept for the callers' signature and is not used: columns 6
 * and 7 (pair 3) are scanned like any other pair. */
static int scan_pt_slot_pg(const device_t *dev, const unsigned char *fuses,
                            int page, int n_cols, int offset, pt_t *pt,
                            int allow_pair3) {
    pt->col_offset = offset;
    pt->is_tail    = (offset >= 72);
    pt->n_lits     = 0;
    for (int c = 0; c < n_cols; c++) {
        const unsigned char *col = device_col(dev, fuses, page, c);
        if (col_is_idle(dev, col)) continue;
        /* Columns 6 and 7 (pair 3) are scanned like any other pair on every
         * XL density; allow_pair3 is no longer used. */
        (void)allow_pair3;
        if (fuse_connected(col, offset)) {
            if (pt->n_lits < MAX_LITS) {
                pt->lits[pt->n_lits].pair    = c / 2;
                pt->lits[pt->n_lits].is_comp = (c % 2 == 0);
                pt->n_lits++;
            }
        }
    }
    return (pt->n_lits > 0);
}

/* Forward declaration: slot_is_exported_xl() is defined later, beside
 * apply_pt_export_fix_xl(). decode_mc() below needs it for the OE slot
 * (slot 1 = PT5). */
static int slot_is_exported_xl(const device_t *dev, const unsigned char *fuses,
                                int fb, int mc, int slot);

/* ---- Decode one macrocell ---- */

static void decode_mc(const device_t *dev, const unsigned char *fuses,
                       int fb, int mc, mc_t *m) {
    memset(m, 0, sizeof(*m));
    m->pre_export_n = -1;   /* C10: UNKNOWN until the export post-pass records it -- a default must not read as "empty" */
    m->fb  = fb;
    m->mc  = mc;
    m->pin = dev->mc_pin ? dev->mc_pin[(fb-1)*18 + mc-1] : 0;
    m->K   = (mc-1) / 3;
    m->r   = (mc-1) % 3;

    /* MC1 (r=0,K=0) and MC5 (r=1,K=1) in the second FB of an FB pair are
     * real, independent macrocells on every XL density, not buried donors:
     * each carries its own product terms at its own xl_and_loc() position.
     * The old suppression below is kept as a no-op. */
    int pair_base_fb = ((fb - 1) / 2) * 2;
    if (dev->is_xl && dev->n_fbs > 1 && fb > 1 && pair_base_fb == 0) {
        if (0)
            return;   /* never taken */
    }

    int ncol = dev->fc_pairs_per_fb * 2;

    /* ---- Attribute decode (family-aware) ----
     * For multi-FB XL devices, ALL attribute fuses (for both FB1 and FB2 MCs)
     * live in the first 108-byte half (fb=1) of each attribute band.
     * Read all attributes using attr_fb = 1 for XL multi-FB, otherwise use fb. */
    int attr_fb_pre = (dev->is_xl && dev->n_fbs > 1) ? 1 : fb;
    /* aoff: byte offset for attribute reads. Differs from mc_off(mc) for
     * pair_base=2 FBs (XC9572XL FB3/FB4) and for pair_base=0 r=1 MCs (see
     * xl_attr_mc_off()). Used in both XL decode blocks. */
    int aoff = dev->is_xl ? xl_attr_mc_off(dev->n_fbs, fb, mc) : mc_off(mc);
    /* Keep the un-normalised offset. The normalisation below subtracts 108
     * once, which is enough for the established reads (never past page 2)
     * but not for the `dev:` line: on XC9572XL some macrocells have
     * xl_attr_mc_off() in 216..323. */
    int aoff_raw = aoff;
    /* xl_attr_mc_off() can return a position >= 108 for pair_base=0 r=1
     * MCs; every other case (r=0, r=2, pair_base=2, non-XL) returns < 108,
     * so attr_page equals attr_fb_pre there. Only the columns indexed by
     * aoff need this; c16/c18/c24/c25 below are read via mc_off(mc) and stay
     * on attr_fb_pre / page 1. */
    int attr_page = attr_fb_pre;
    if (dev->is_xl && aoff >= 108) { attr_page = 2; aoff -= 108; }
    /* Slot 0's own selector, cols 12/13 (the vendor's CPT0A/CPT0B; this
     * header calls the slot PT3). Slot 0's dedicated control function is
     * the product-term clock. */
    const unsigned char *c12  = device_col(dev,fuses,attr_fb_pre,COL_PT3_MUX_B1);
    const unsigned char *c13  = device_col(dev,fuses,attr_fb_pre,COL_PT3_MUX_B0);
    const unsigned char *c16  = device_col(dev,fuses,attr_fb_pre,COL_PT4_MUX_B1);
    const unsigned char *c17  = device_col(dev,fuses,attr_fb_pre,COL_PT4_MUX_B0);
    const unsigned char *c18  = device_col(dev,fuses,attr_fb_pre,COL_PT1_MUX_B1);
    const unsigned char *c19  = device_col(dev,fuses,attr_fb_pre,COL_PT1_MUX_B0);
    const unsigned char *c14  = device_col(dev,fuses,attr_fb_pre,COL_PT5_MUX_B1);
    const unsigned char *c15  = device_col(dev,fuses,attr_fb_pre,COL_PT5_MUX_B0);
    const unsigned char *c20  = device_col(dev,fuses,attr_fb_pre,COL_PT2_MUX_B1); (void)c20;
    const unsigned char *c21  = device_col(dev,fuses,attr_fb_pre,COL_PT2_MUX_B0); (void)c21;
    /* The XL slot selectors are read at the per-macrocell attribute position
     * (attr_page/aoff), not at mc_off(mc). Same columns as non-XL; the raw
     * bits are complemented, but dev_attr already inverts for XL, so the same
     * rule applies:
     *   dev_attr(b1)=0,b0=0  slot unused
     *   dev_attr(b1)=1,b0=1  slot dedicated to its control function
     *   dev_attr(b1)=1,b0=0  slot merged into the DATA sum */
    const unsigned char *x16 = device_col(dev,fuses,attr_page,COL_PT4_MUX_B1);
    const unsigned char *x17 = device_col(dev,fuses,attr_page,COL_PT4_MUX_B0);
    const unsigned char *x18 = device_col(dev,fuses,attr_page,COL_PT1_MUX_B1);
    const unsigned char *x19 = device_col(dev,fuses,attr_page,COL_PT1_MUX_B0);
    const unsigned char *x14 = device_col(dev,fuses,attr_page,COL_PT5_MUX_B1);
    const unsigned char *x15 = device_col(dev,fuses,attr_page,COL_PT5_MUX_B0);
    const unsigned char *x12 = device_col(dev,fuses,attr_page,COL_PT3_MUX_B1);
    const unsigned char *x13 = device_col(dev,fuses,attr_page,COL_PT3_MUX_B0);
    int xl_rst_ded = dev->is_xl && dev_attr_off(dev,x16,aoff) && dev_attr_off(dev,x17,aoff);
    int xl_pst_ded = dev->is_xl && dev_attr_off(dev,x18,aoff) && dev_attr_off(dev,x19,aoff);
    int xl_oe_ded  = dev->is_xl && dev_attr_off(dev,x14,aoff) && dev_attr_off(dev,x15,aoff);
    /* Is PT slot 0 dedicated to its control function? Same two-bit rule as
     * the RST/PST/OE slots, read at the family's own attribute position
     * (XL: attr_page/aoff; non-XL: mc_off(mc)). dev_attr normalises the two
     * families' opposite fuse polarity, so one test serves both. */
    int clk_slot_dedicated = dev->is_xl
        ? (dev_attr_off(dev,x12,aoff) && dev_attr_off(dev,x13,aoff))
        : (dev_attr(dev,c12,mc)       && dev_attr(dev,c13,mc));
    /* Is PT slot 4 dedicated to PTINV? Same two-bit rule as the
     * RST/PST/OE/CLK slots, read at COL_PT2_MUX_B1/B0 (cols 20/21 -- this
     * header calls them the "5th data slot"). dev_attr_off() normalises the
     * families' opposite fuse polarity: raw 1,1 -> 0,0 on non-XL and raw
     * 0,0 -> 1,1 on XL are the same normalised change. */
    const unsigned char *x20 = device_col(dev,fuses,attr_page,COL_PT2_MUX_B1);
    const unsigned char *x21 = device_col(dev,fuses,attr_page,COL_PT2_MUX_B0);
    int ptinv_slot_dedicated = dev->is_xl
        ? (dev_attr_off(dev,x20,aoff) && dev_attr_off(dev,x21,aoff))
        : (dev_attr_off(dev,x20,mc_off(mc)) && dev_attr_off(dev,x21,mc_off(mc)));
    const unsigned char *c22  = device_col(dev,fuses,attr_fb_pre,COL_INVERT_OUT); (void)c22;
    const unsigned char *c24  = device_col(dev,fuses,attr_fb_pre,COL_FB_FROM_LOWER);
    const unsigned char *c25  = device_col(dev,fuses,attr_fb_pre,COL_FB_TO_UPPER);
    const unsigned char *c26  = device_col(dev,fuses,attr_page,COL_PWR_LOW);
    const unsigned char *c27  = device_col(dev,fuses,attr_page,COL_OE_GTS);
    const unsigned char *c28  = device_col(dev,fuses,attr_page,COL_OE_MUX_B0);
    const unsigned char *c29  = device_col(dev,fuses,attr_page,COL_OE_MUX_B1);

    /* For XC9500XL: has_reset and has_preset are NOT valid from the old
     * non-XL attribute columns (c16/c18 map to different functions on XL).
     * Leave them 0 here; the PT scan below derives them from the fuse map. */
    int oe_slot_dedicated = 1;   /* conservative default */
    if (!dev->is_xl) {
        /* Each PT slot is selected by a two-bit mux: COL_PT4_MUX_B1/B0 for
         * the RESET slot, COL_PT1_MUX_B1/B0 for PRESET. B1 alone cannot
         * tell a dedicated control from a slot merged into the DATA sum.
         *
         *   (B1,B0) = (1,1)  slot unused
         *             (0,0)  dedicated to its control function
         *             (0,1)  merged into DATA
         *
         * A macrocell with five DATA terms has all five selectors at (0,1). */
        m->has_reset  = dev_attr(dev, c16, mc) && dev_attr(dev, c17, mc);
        m->has_preset = dev_attr(dev, c18, mc) && dev_attr(dev, c19, mc);
        /* Same two-bit rule for the OE slot. */
        oe_slot_dedicated = dev_attr(dev, c14, mc) && dev_attr(dev, c15, mc);
    }

    /* Global set/reset enables, per family. The two families differ in
     * column and in addressing (both from xc9500.h):
     *   non-XL  COL_GSR_RESET 41 / COL_GSR_SET 42, read at mc_off(mc)
     *   XL      COL_XL_RESET_GSR 40 / COL_XL_SET_GSR 41, read at
     *           xl_attr_mc_off() with attr_page/aoff
     * Polarity is dev_attr()/dev_attr_off()'s family rule. vhdl.c declares
     * a gsr port only from these two fields. */
    {
        int gcol_r = dev->is_xl ? COL_XL_RESET_GSR : COL_GSR_RESET;
        int gcol_s = dev->is_xl ? COL_XL_SET_GSR   : COL_GSR_SET;
        const unsigned char *cgr = device_col(dev, fuses,
                                              dev->is_xl ? attr_page : attr_fb_pre, gcol_r);
        const unsigned char *cgs = device_col(dev, fuses,
                                              dev->is_xl ? attr_page : attr_fb_pre, gcol_s);
        m->gsr_reset = dev->is_xl ? dev_attr_off(dev, cgr, aoff) : dev_attr(dev, cgr, mc);
        m->gsr_set   = dev->is_xl ? dev_attr_off(dev, cgs, aoff) : dev_attr(dev, cgs, mc);
    }

    /* OE mode detection.
     * For XC9500 (non-XL): decode from col27/28/29 (OE_GTS, OE_MUX_B0/B1).
     * For XC9500XL: set to 0 here; determined AFTER the OE scan below (see bottom
     * of decode_mc), based on col30 (COL_OE_INVERT) and OE product-term presence. */
    if (!dev->is_xl) {
        {   /* Pass OE3/OE4 so GTS3 and GTS4 are named on the devices that
             * have them (XC95144/216/288). */
            const unsigned char *c30 = device_col(dev, fuses, fb, 30);
            const unsigned char *c31 = device_col(dev, fuses, fb, 31);
            m->oe_mode = decode_oe_mode4(c27, c28, c29, c30, c31, mc);
        }
    }
        {
        /* Read every column the family's .dev file names. xc9500.dev names
         * 37 columns and xc9500xl.dev 36, and the sets differ: XL has
         * COEINV(30), CCKINV(35), CKEN0/1(36/37), QSREN(44) and SAPWR0..4 at
         * 45-49, where non-XL has OE3/OE4 at 30/31, TRAN at 35 and CGND at 48.
         * The table is selected by dev->is_xl. */
        static const int named_nonxl[] = {12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,35,36,37,40,41,42,43,44,45,46,48,49,50,51,52,53,54};
        static const int named_xl[]    = {12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,32,33,34,35,36,37,39,40,41,42,43,44,45,46,47,48,49};
        const int *named = dev->is_xl ? named_xl : named_nonxl;
        unsigned nnamed = dev->is_xl
            ? sizeof(named_xl)/sizeof(named_xl[0])
            : sizeof(named_nonxl)/sizeof(named_nonxl[0]);
        unsigned q;
        int k;
        for (k = 0; k < 72; k++) m->devbit[k] = -1;
        for (q = 0; q < nnamed; q++) {
            /* Use aoff, not mc_off(mc): XL's attribute offset comes from
             * xl_attr_mc_off(). For non-XL, aoff == mc_off(mc). */
            (void)device_col(dev, fuses, attr_page, named[q]);
            /* This read's page comes from the un-normalised offset. For XL,
             * device_col()'s second argument is a 108-byte sub-block index
             * within the band (fuses + col*n_fbs*108 + (fb-1)*108), so page p
             * and offset o address band position (p-1)*108 + o; with
             * p = attr_fb_pre + aoff_raw/108 and o = aoff_raw%108 that is
             * aoff_raw, for any number of pages. */
            {
                int pg_ = attr_fb_pre, of_ = aoff_raw;
                const unsigned char *cx;
                if (dev->is_xl && of_ >= 0) { pg_ += of_ / 108; of_ %= 108; }
                cx = (of_ >= 0 && of_ < 108)
                   ? device_col(dev, fuses, pg_, named[q]) : NULL;
                if (cx) m->devbit[named[q]] = cx[of_];
                /* XC_ATTR=1 dumps this read's own page and offset:
                 *
                 *     XC_ATTR=1 ./xc9500dump decode -- <file.jed> 2>&1 >/dev/null
                 *
                 * A new attribute read should use these addresses rather
                 * than re-deriving them. */
                if (getenv("XC_ATTR"))
                    fprintf(stderr,
                        "ATTR fb=%d mc=%d col=%-2d page=%d off=%-4d "
                        "aoff=%d aoff_raw=%d val=%d\n",
                        fb, mc, named[q], pg_, of_, aoff, aoff_raw,
                        cx ? cx[of_] : -1);
            }
        }
        /* CPTINV (column 22) marks an inverted D/T input; the fitter's report
         * writes it `!sig.D = <terms>`. A combinational inverter driving an
         * output from a registered source is merged by the fitter into that
         * register with an inverted D. */
        /* The final scope is applied below, once m->is_toggle is known.
         * Record the raw bit here. */
        m->d_inverted = (m->devbit[22] == 0);
        /* Columns 31/44/45/46/48 mean OE4/FBOE0/FBOE1/FBINV/CGND on non-XL
         * only; on XL the same column numbers are QSREN and SAPWR bits (see
         * dc_xl[]). Read for non-XL only. */
        if (!dev->is_xl) {
            m->oe4   = m->devbit[31] < 0 ? 0 : m->devbit[31];
            m->fboe0 = m->devbit[44] < 0 ? 0 : m->devbit[44];
            m->fboe1 = m->devbit[45] < 0 ? 0 : m->devbit[45];
            m->fbinv = m->devbit[46] < 0 ? 0 : m->devbit[46];
            m->cgnd  = m->devbit[48] < 0 ? 0 : m->devbit[48];
            for (k = 0; k < 4; k++)
                m->sapwr[k] = m->devbit[51+k] < 0 ? 0 : m->devbit[51+k];
        }
        m->dev_is_xl = dev->is_xl;
        m->have_dev_bits = 1;
    }
    m->imports_from_lower = dev_attr(dev, c24, mc);
    m->exports_to_upper   = dev_attr(dev, c25, mc);

    if (dev->is_xl) {
        /* XC9500XL: bypass + 2-bit CLK mux + per-MC invert.
         * On multi-FB XL devices the attribute fuses live in the fb=1 half
         * of each attribute band (pair_base=0: FB1 and FB2 share page 1).
         * For pair_base=2 (FB3/FB4, XC9572XL) they are also on page 1 but at
         * xl_attr_mc_off(fb,mc), not mc_off(mc). All MCs in the same K-group
         * (K=0..2 or K=3..5) share one attribute byte. */
        /* attr_page already accounts for the pair_base=0 r=1 page-2
         * overflow. */
        const unsigned char *c32 = device_col(dev,fuses,attr_page,COL_XL_BYPASS);
        const unsigned char *c33 = device_col(dev,fuses,attr_page,COL_XL_CLK_B1);
        const unsigned char *c34 = device_col(dev,fuses,attr_page,COL_XL_CLK_B0);
        const unsigned char *c35 = device_col(dev,fuses,attr_page,COL_XL_CLK_INVERT);
        const unsigned char *c39 = device_col(dev,fuses,attr_page,COL_XL_IS_TOGGLE);
        const unsigned char *c42 = device_col(dev,fuses,attr_page,COL_XL_INIT_SET);
        const unsigned char *c44 = device_col(dev,fuses,attr_page,COL_XL_FAST_SLEW);

        int bypass    = dev_attr_off(dev, c32, aoff);
        /* CLK MUX: use aoff in place of mc_off(mc) */
        int clk_code  = (c33[aoff] << 1) | c34[aoff];  /* {CLK_B1, CLK_B0} */
        m->clk_falling = dev_attr_off(dev, c35, aoff);
        m->is_toggle   = dev_attr_off(dev, c39, aoff);
        m->init_set    = dev_attr_off(dev, c42, aoff);
        m->slew_slow   = !dev_attr_off(dev, c44, aoff); /* fast_slew=1 means NOT slow */
        int clk_mux = clk_code;

        /* Decode to internal clk_mode */
        if (bypass) {
            m->clk_mode = CLK_MODE_COMB;
            m->is_combi = 1;
        } else {
            /* Map the XL 2-bit code to the internal CLK_MODE constants.
             *
             * XC9536XL VQ44 (n_fbs<=2, is_xl):
             *   ISE writes no GCK3 bits for K=2/K=5 MCs and omits GCK1 bits
             *   for some K=0/K=3 positions. Code 0,0 (XL_CLK_GCK2) is the
             *   write-nothing default; the actual clock is fixed by K-group:
             *     K%3==0 -> GCK1,  K%3==1 -> GCK2,  K%3==2 -> GCK3
             *   Only code 0 is overridden; explicit GCK1 (code 1) and PT-CLK
             *   (code 3) are kept as written. */
            switch (clk_mux) {
                case XL_CLK_GCK1: m->clk_mode = CLK_MODE_GCK1; break;
                case XL_CLK_GCK2:
                    if (dev->is_xl && dev->n_fbs <= 2) {
                        /* Derive from K-group -- code 0 is not trusted on XC9536XL */
                        switch (m->K % 3) {
                            case 0: m->clk_mode = CLK_MODE_GCK1; break;
                            case 1: m->clk_mode = CLK_MODE_GCK2; break;
                            case 2: m->clk_mode = CLK_MODE_GCK3; break;
                        }
                    } else {
                        m->clk_mode = CLK_MODE_GCK2;
                    }
                    break;
                case XL_CLK_GCK3: m->clk_mode = CLK_MODE_GCK3; break;
                default:           m->clk_mode = CLK_MODE_PT;   break;
            }
            m->is_combi = 0;
        }
        /* XL pwr_low: col26 = "STD power" attribute.
         * For XL (F0 default, 1=active): 1=STD power (the fitter sets this).
         * Use aoff (xl_attr_mc_off) for pair_base=2 FBs. */
        m->pwr_low = !dev_attr_off(dev, c26, aoff); /* 1=LOW if STD-power bit not set */

    } else {
        /* XC9500 non-XL: 3-bit CLK mode + global GCK invert */
        const unsigned char *c35 = device_col(dev,fuses,fb,COL_CLK_B2);
        const unsigned char *c36 = device_col(dev,fuses,fb,COL_CLK_B1);
        const unsigned char *c37 = device_col(dev,fuses,fb,COL_CLK_B0);
        const unsigned char *c40 = device_col(dev,fuses,fb,COL_IS_TOGGLE);
        const unsigned char *c43 = device_col(dev,fuses,fb,COL_INIT_SET);
        const unsigned char *c49 = device_col(dev,fuses,fb,COL_SLEW_SLOW);
        const unsigned char *c50 = device_col(dev,fuses,fb,COL_PWR_LOW_ALT);
        /* GCK invert is a single device-wide property of the clock net, stored
         * only in FB1's band -- not the clocked macrocell's own FB, unlike
         * every other read in this block. Other FBs' col0 at the GCK1-invert
         * offset stays inactive whatever the setting. */
        const unsigned char *col0= device_col(dev,fuses,1,0);

        m->clk_mode = decode_clk_mode_non_xl(c35, c36, c37, mc);
        m->is_combi = (m->clk_mode == CLK_MODE_COMB);

        int gck_num = (m->clk_mode==CLK_MODE_GCK1)?1:
                      (m->clk_mode==CLK_MODE_GCK2)?2:
                      (m->clk_mode==CLK_MODE_GCK3)?3:0;
        m->clk_falling = gck_num ? gck_inverted(col0, gck_num) : 0;

        m->is_toggle  = dev_attr(dev, c40, mc);
        m->init_set   = dev_attr(dev, c43, mc);
        /* pwr_low and slew have inverted polarity vs other non-XL attributes.
         * Convention: 0=active for most attrs, but:
         *   pwr_low: 1=LOW power (default 0=STD power, programmed to 1=LOW)
         *   slew_slow: 1=SLOW (inactive MCs are marked SLOW by the fitter; 0=FAST)
         * Use !dev_attr (which returns 0=active) -> (raw==1) for non-XL. */
        m->slew_slow  = !dev_attr(dev, c49, mc);  /* 1=SLOW, 0=FAST */
        m->pwr_low    = !dev_attr(dev, c26, mc) || !dev_attr(dev, c50, mc);
    }

    m->is_real_output = (m->oe_mode != 0) || m->is_toggle;

    /* CPTINV (column 22) marks an inverted D/T input; the fitter's report
     * writes it `!sig.D = <terms>`. Losing it makes the macrocell implement
     * the opposite function.
     *
     * Polarity: on non-XL raw CPTINV=0 means inverted; on XL raw CPTINV=1
     * means inverted. It applies to combinational, D-type and T-type
     * macrocells on every density. */
    if (dev->is_xl) {
        m->d_inverted = (m->devbit[22] == 1);
    } else if (m->is_combi) {
        /* keep the raw value read above */
    } else if (dev->n_fbs == 6) {
        /* keep the raw value read above */
    } else if (dev->n_fbs == 4) {
        /* XC9572, registered D/T: raw CPTINV=0 means inverted. */
        m->d_inverted = (m->devbit[22] == 0);
    } else if (dev->n_fbs == 8 || dev->n_fbs == 12 || dev->n_fbs == 16) {
        /* XC95144, XC95216, XC95288, registered D/T: raw CPTINV=0 means
         * inverted. */
        m->d_inverted = (m->devbit[22] == 0);
    } else if (dev->n_fbs == 2) {
        /* XC9536, registered D and T, with or without an async-reset PT: raw
         * CPTINV=0 means inverted. */
        m->d_inverted = (m->devbit[22] == 0);
    } else {
        m->d_inverted = 0;
    }

    /* ---- PT scanning ---- */
    /*
     * XC9536XL multi-FB layout (fb > 1, i.e. FB2+):
     *   The 216-byte band interleaves both FBs in the same two pages.
     *   Page 1 (bytes 0-107): r=0 and r=1 PT rows for FB1 and FB2.
     *   Page 2 (bytes 108-215): r=2 PT rows for FB1 and FB2.
     *   Within each row, FB2 is at +8 bytes from FB1 (r=0/r=1) or +6 (r=2).
     *   Use xl_and_loc() to get the (page, offset) per PT.
     *
     * FB1 (fb==1, all devices): see the FB1 branch below.
     */
    int pg_data, pg_oe, pg_rst, pg_pst;
    int off_data, off_oe, off_rst, off_pst;

    if (dev->is_xl && dev->n_fbs > 1 && fb > 1) {
        /* FB2+ on multi-FB XL: xl_and_loc() positions. MC4 and MC18 are real
         * macrocells and read their own positions. */
        int scan_r = m->r, scan_K = m->K;
        /* scan_r/scan_K: on 4-FB parts MC18 (r=2,K=5) starts out mapped to
         * MC5's slot (r=1,K=1); DATA, OE, RST and PST below each switch it
         * back to MC18's own position. */
        if (dev->n_fbs <= 4) {
            if (m->r == 2 && m->K == 5 && dev->n_fbs == 4) { scan_r = 1; scan_K = 1; }
        }

        {
            /* DATA at the macrocell's own position. For FB2_MC18 (r=2,K=5) on
             * XC9572XL that is xl_and_loc(4,2,2,5,0) -> page 3, off 107
             * (band offset 323). */
            int d_r = scan_r, d_K = scan_K;
            if (m->r == 2 && m->K == 5 && dev->n_fbs == 4) { d_r = m->r; d_K = m->K; }
            xl_and_loc(dev->n_fbs, fb, d_r, d_K, 0, &pg_data, &off_data);
        }
        {
            /* OE at MC18's own position. */
            int e_r = scan_r, e_K = scan_K;
            if (m->r == 2 && m->K == 5 && dev->n_fbs == 4) { e_r = m->r; e_K = m->K; }
            xl_and_loc(dev->n_fbs, fb, e_r, e_K, 1, &pg_oe,   &off_oe);
        }
        {
            /* RST at MC18's own position. */
            int t_r = scan_r, t_K = scan_K;
            if (m->r == 2 && m->K == 5 && dev->n_fbs == 4) { t_r = m->r; t_K = m->K; }
            xl_and_loc(dev->n_fbs, fb, t_r, t_K, 2, &pg_rst,  &off_rst);
        }
        {
            /* PST at MC18's own position. */
            int s_r = scan_r, s_K = scan_K;
            if (m->r == 2 && m->K == 5 && dev->n_fbs == 4) { s_r = m->r; s_K = m->K; }
            xl_and_loc(dev->n_fbs, fb, s_r, s_K, 3, &pg_pst,  &off_pst);
        }
    } else {
        /* FB1 (or non-XL, or single-FB).
         *
         * XL FB1: slots come from xl_and_loc() at the macrocell's own (r, K);
         * on 2-FB and 4-FB parts r=0 DATA comes from pt_slot_offset_xl().
         *
         * Non-XL: pt_slot_offset (single-page layout). */
        if (dev->is_xl) {
            /* XL FB1 DATA. 8-FB parts interleave the function blocks in each band,
             * so DATA uses the interleaved xl_and_loc() address there. */
            if (dev->n_fbs >= 8) {
                /* 8-FB parts: FB1 DATA at the interleaved address (an FB-major
                 * address would land in the wrong half of the band). */
                xl_and_loc(dev->n_fbs, fb, m->r, m->K, 0, &pg_data, &off_data);
            } else {
                pg_data = fb;
                /* 2-FB and 4-FB parts: DATA follows the page layout for every r.
                 * Rows 0-4 hold MC1/4/7/10/13/16, rows 5-9 MC2/5/8/11/14/17,
                 * rows 10-14 MC3/6/9/12/15/18, each in the order PT3, PT5,
                 * PT4, PT1, PT2; rows 0-8 are eight fuses wide and rows 9-14
                 * six. DATA is PT3, slot 0 of the r group (row r*5):
                 *     n_fbs=2   r=0 K   r=1 80+K    r=2 156+K
                 *     n_fbs=4   r=0 K   r=1 160+K   r=2 312+K */
                xl_and_loc(dev->n_fbs, fb, m->r, m->K, 0, &pg_data, &off_data);
                if (0) {
                    /* fixed 2-FB r=1 offset, not used */
                    off_data = 72 + 8 + m->K;
                } else if (m->r >= 1) {
                    /* r>=1: xl_and_loc() front/tail formula (see above). r=2 DATA is
                     * its own slot, not shared with the r=1 sibling. */
                    xl_and_loc(dev->n_fbs, fb, m->r, m->K, 0, &pg_data, &off_data);
                } else {
                    off_data = pt_slot_offset_xl(m->r, m->K, 0); /* correct for DATA */
                }
            }
            if (m->r == 2) {
                if (dev->n_fbs >= 8) {
                    /* 8-FB parts: OE, RST and PST from r=2's own slots. */
                    xl_and_loc(dev->n_fbs, fb, m->r, m->K, 1, &pg_oe,  &off_oe);
                    xl_and_loc(dev->n_fbs, fb, m->r, m->K, 2, &pg_rst, &off_rst);
                    xl_and_loc(dev->n_fbs, fb, m->r, m->K, 3, &pg_pst, &off_pst);
                } else {
                    /* OE reads r=2's own pf=1 slot */
                    xl_and_loc(dev->n_fbs, fb, m->r, m->K, 1, &pg_oe,  &off_oe);
                    /* RST reads r=2's own slot. */
                    xl_and_loc(dev->n_fbs, fb, m->r, m->K, 2, &pg_rst, &off_rst);
                    /* PST: r=2 formula */
                    xl_and_loc(dev->n_fbs, fb, m->r, m->K, 3, &pg_pst, &off_pst);
                }
            } else {
                /* r=0 and r=1: xl_and_loc for FB1 */
                xl_and_loc(dev->n_fbs, fb, m->r, m->K, 1, &pg_oe,  &off_oe);
                xl_and_loc(dev->n_fbs, fb, m->r, m->K, 2, &pg_rst, &off_rst);
                xl_and_loc(dev->n_fbs, fb, m->r, m->K, 3, &pg_pst, &off_pst);
            }
        } else {
            pg_data = fb; pg_oe = fb; pg_rst = fb; pg_pst = fb;
            off_data = pt_slot_offset(m->r, m->K, 0);
            off_oe   = pt_slot_offset(m->r, m->K, 1);
            off_rst  = pt_slot_offset(m->r, m->K, 2);
            off_pst  = pt_slot_offset(m->r, m->K, 3);
        }
    }

    pt_t tmp;

    /* On FB1 of a 2-FB or 4-FB XL part, an r=2 MC (K=0..4) and its r=1
     * sibling share an OE slot location; r=2 reads it as OE only when the
     * slot is dedicated. K=5 (MC18, buried) is excluded. */
    int fb1_r2_shares_with_r1 = (dev->is_xl && fb == 1 && dev->n_fbs > 1 &&
                                 dev->n_fbs < 8 && m->r == 2 && m->K < 5);

    /* Primary data slot. Column 7 (pair 3 true) is a valid literal on FB1
     * and FB2 alike. */
    int allow_pair3_data = (dev->is_xl && dev->n_fbs <= 2 &&
                            ((fb-1)/2)*2 == 0);  /* pair_base==0, both FB1 and FB2 */
    m->clk_slot_empty = 0;
    if (!scan_pt_slot_pg(dev, fuses, pg_data, ncol, off_data, &tmp, allow_pair3_data)) {
        if (m->clk_mode == CLK_MODE_PT) m->clk_slot_empty = 1;
    } else {
        if (m->clk_mode == CLK_MODE_PT && tmp.n_lits >= 1)
            m->clk_pt_col = tmp.lits[0].pair*2 + (tmp.lits[0].is_comp ? 0 : 1);
        /* When this slot is the dedicated product-term clock it is not a data
         * term and does not join the D/T sum. */
        if (m->clk_mode == CLK_MODE_PT && clk_slot_dedicated) {
            m->clk_pt = tmp; m->has_clk_pt = 1;
        } else if (m->n_data_pts < MAX_PTS) {
            tmp.slot = 0;   /* primary slot = CPT0 */
            m->data_pts[m->n_data_pts++] = tmp;
        }
    }

    /* Additional data slots (PT2..PT5 for combinational MCs). */
    /* Multi-FB XL parts, FB1 included: the additional slots come from
     * xl_and_loc() (interleaved address). The per-r loops below compute an
     * FB-major offset and serve non-XL only. */
    if (dev->is_xl && dev->n_fbs > 1) {
        /* The macrocell's own (r, K). */
        int scan_r = m->r, scan_K = m->K;

        int pg_ce, off_ce;
        xl_and_loc(dev->n_fbs, fb, scan_r, scan_K, 4, &pg_ce, &off_ce);
        /* Scan pf=1..4 (OE, RST, PST, CE) as additional data if they're not control */
        int add_pgs[4], add_offs[4];
        xl_and_loc(dev->n_fbs, fb, scan_r, scan_K, 1, &add_pgs[0], &add_offs[0]);
        xl_and_loc(dev->n_fbs, fb, scan_r, scan_K, 2, &add_pgs[1], &add_offs[1]);
        xl_and_loc(dev->n_fbs, fb, scan_r, scan_K, 3, &add_pgs[2], &add_offs[2]);
        xl_and_loc(dev->n_fbs, fb, scan_r, scan_K, 4, &add_pgs[3], &add_offs[3]);
        /* Skip a control slot only when it is dedicated. A slot whose
         * selector reads "merged" carries a DATA product term. */
        for (int i = 0; i < 4; i++) {
            if (add_pgs[i]==pg_oe  && add_offs[i]==off_oe  && xl_oe_ded)  continue;
            if (add_pgs[i]==pg_rst && add_offs[i]==off_rst && xl_rst_ded) continue;
            if (add_pgs[i]==pg_pst && add_offs[i]==off_pst && xl_pst_ded) continue;
            /* Slot 4 dedicated to PTINV is an XOR input, not an OR term. Take
             * it out of the sum and keep it separately. */
            if (i == 3 && ptinv_slot_dedicated) {
                if (scan_pt_slot_pg(dev,fuses,add_pgs[i],ncol,add_offs[i],
                                    &m->ptinv_pt, 0))
                    m->has_ptinv = 1;
                continue;
            }
            if (scan_pt_slot_pg(dev,fuses,add_pgs[i],ncol,add_offs[i],&tmp, 0)
                && m->n_data_pts < MAX_PTS) {
                tmp.slot = i + 1;   /* CPT1..CPT4 */
                m->data_pts[m->n_data_pts++] = tmp;
            }
        }
    } else if (m->r == 0) {
        for (int g = 1; g <= 4; g++) {
            int doff = g*8 + m->K;
            /* Skip a control slot only when it is dedicated. A slot whose
             * two-bit selector reads (0,1) is merged into the DATA sum. */
            if ((doff==off_oe  && oe_slot_dedicated) ||
                (doff==off_rst && m->has_reset) ||
                (doff==off_pst && m->has_preset)) continue;
            if (g == 4 && ptinv_slot_dedicated) {   /* PTINV */
                if (scan_pt_slot_pg(dev,fuses,pg_data,ncol,doff,&m->ptinv_pt,0))
                    m->has_ptinv = 1;
                continue;
            }
            if (scan_pt_slot_pg(dev,fuses,pg_data,ncol,doff,&tmp, 0) && m->n_data_pts < MAX_PTS) {
                tmp.slot = g;   /* CPT1..CPT4 (see apply_pt_export_fix_nonxl()) */
                m->data_pts[m->n_data_pts++] = tmp;
            }
        }
    } else if (m->r == 1) {
        /* An r=1 macrocell's primary DATA is at g=5 ((pf+5)*8+K with pf=0),
         * so its additional slots are g=6..9. r=0's loop uses g=1..4. */
        for (int g = 6; g <= 9; g++) {
            int doff = g*8 + m->K;
            /* Skip a control slot only when it is dedicated. A slot whose
             * two-bit selector reads (0,1) is merged into the DATA sum. */
            if ((doff==off_oe  && oe_slot_dedicated) ||
                (doff==off_rst && m->has_reset) ||
                (doff==off_pst && m->has_preset)) continue;
            if (g == 9 && ptinv_slot_dedicated) {   /* PTINV */
                if (scan_pt_slot_pg(dev,fuses,pg_data,ncol,doff,&m->ptinv_pt,0))
                    m->has_ptinv = 1;
                continue;
            }
            if (scan_pt_slot_pg(dev,fuses,pg_data,ncol,doff,&tmp, 0) && m->n_data_pts < MAX_PTS) {
                tmp.slot = g - 5;   /* g=6..9 -> CPT1..CPT4 */
                m->data_pts[m->n_data_pts++] = tmp;
            }
        }
    } else { /* r==2 */
        /* Additional data slots for r=2 MCs. Non-XL only: every XL part has
         * n_fbs >= 2 and takes the canonical path above. */
        {
            /* r=2's five slots are 72+{6,12,18,24,30}+K. Slot 72+K is r=1's
             * fifth slot, not this macrocell's. The control slots 12/18/24 are
             * scanned when they are not dedicated, as in the r=0 and r=1
             * loops. */
            static const int r2_tb[] = {12, 18, 24, 30};
            for (int ti = 0; ti < 4; ti++) {
                int doff = 72 + r2_tb[ti] + m->K;
                if ((doff==off_oe  && oe_slot_dedicated) ||
                    (doff==off_rst && m->has_reset) ||
                    (doff==off_pst && m->has_preset)) continue;
                if (ti == 3 && ptinv_slot_dedicated) {   /* PTINV */
                    if (scan_pt_slot_pg(dev,fuses,pg_data,ncol,doff,&m->ptinv_pt,0))
                        m->has_ptinv = 1;
                    continue;
                }
                if (scan_pt_slot_pg(dev,fuses,pg_data,ncol,doff,&tmp, 0) && m->n_data_pts < MAX_PTS) {
                    tmp.slot = ti + 1;   /* ti 0..3 -> CPT1..CPT4, same OE/RST/PST/PTINV order as the checks above */
                    m->data_pts[m->n_data_pts++] = tmp;
                }
            }
        }
    }

    /* Control slots. */
    if (dev->is_xl) {
        /* XL: the slot selectors say whether a slot is a dedicated control
         * term; a non-empty scan alone cannot tell a reset from a slot used
         * for DATA. */
        /* r=2 reads its own RST slot, so there is no sharing with r=1.
         * xl_rst_ded decides reset vs data. */
        if (xl_rst_ded &&
            scan_pt_slot_pg(dev,fuses,pg_rst,ncol,off_rst,&m->rst_pt, 0))
            m->has_reset = 1;
        if (xl_pst_ded &&
            scan_pt_slot_pg(dev,fuses,pg_pst,ncol,off_pst,&m->pst_pt, 0))
            m->has_preset = 1;
    } else {
        if (m->has_reset)  scan_pt_slot_pg(dev,fuses,pg_rst,ncol,off_rst,&m->rst_pt, 0);
        if (m->has_preset) scan_pt_slot_pg(dev,fuses,pg_pst,ncol,off_pst,&m->pst_pt, 0);
    }

    /* CE (clock enable) for XC9500XL.
     * The CE product term uses the PST slot (pf=3) of the macrocell's own
     * r-group: xl_and_loc(fb, r, K, pf=3). Whether a macrocell has a clock
     * enable comes from its own CKEN0/CKEN1 attribute bits. */
    if (dev->is_xl) {
        /* The CE attribute is read from the macrocell's own function block. */
        int attr_fb_ce = fb;
        /* CKEN0 is column 36, CKEN1 column 37 (= COL_XL_CE_OR_R), read through
         * m->devbit[], which already carries this macrocell's attribute
         * page/offset. */
        const unsigned char *c36_ce = device_col(dev, fuses, attr_fb_ce, COL_XL_CE_OR_R);
        /* Attribute position of the CE bit by r-group (not used: m->devbit[]
         * is read instead):
         *   r=0 or r=1 CE: mc_off of the r=1 MC in the same K-group (mc_off(3*K+2))
         *   r=2 CE:        mc_off of the r=0 MC in the K+1 group   (mc_off(3*(K+1)+1)) */
        int r1_mc;
        if (m->r == 2)
            r1_mc = 3 * (m->K + 1) + 1;   /* r=0 MC in K+1 group */
        else
            r1_mc = 3 * m->K + 2;          /* r=1 MC in same K group */
        if (r1_mc > 18) r1_mc = 18;        /* clamp for last K-group edge case */
        int r1_moff = mc_off(r1_mc);
        (void)c36_ce; (void)r1_moff;
        /* The clock enable is marked by CKEN0 XOR CKEN1:
         *     CKEN0=0 CKEN1=1  ->  clock enable
         *     CKEN0=1 CKEN1=0  ->  clock enable (CE and RESET slots transposed)
         *     CKEN0=0 CKEN1=0  ->  no clock enable */
        int k_group_has_ce = (m->devbit[COL_XL_CE_OR_R] !=
                              m->devbit[COL_XL_CE_OR_R - 1]);
        /* CKEN (0,1) selects a clock enable; the term is the shared PST/CE
         * slot's only when that slot's selector CPT3 (cols 18/19, XL raw =
         * logical) is SPC (1,1). With CPT3 off (slot empty) or EXP (slot
         * lent to a neighbour) the enable is the constant Gnd (the fitter
         * writes `y.CE = Gnd;`): the register never loads. The transposed
         * group (CKEN 1,0) keeps its reading. */
        if (k_group_has_ce && m->have_dev_bits && m->devbit[COL_XL_CE_OR_R - 1] == 0 && m->devbit[COL_XL_CE_OR_R] == 1
            && !(m->devbit[18] == 1 && m->devbit[19] == 1)) {
            m->ce_const0 = 1;
            k_group_has_ce = 0;
        }
        if (k_group_has_ce) {
            /* The CE product term is in the macrocell's own r-group. */
            int ce_r = m->r;
            int pg_ce, off_ce;
            xl_and_loc(dev->n_fbs, fb, ce_r, m->K, 3, &pg_ce, &off_ce);
            if (scan_pt_slot_pg(dev, fuses, pg_ce, ncol, off_ce, &m->ce_pt, 0)) {
                m->has_ce = 1;
                /* Slot 3 carries the clock enable, and the preset scan reads
                 * the same address. The CKEN pair says whether that preset is
                 * real:
                 *     CKEN0=0 CKEN1=1  ->  no preset
                 *     CKEN0=1 CKEN1=0  ->  real preset (it shares the term) */
                if (m->devbit[COL_XL_CE_OR_R - 1] == 0 &&
                    m->devbit[COL_XL_CE_OR_R] == 1) {
                    m->has_preset = 0;
                    m->pst_pt.n_lits = 0;
                } else if (m->devbit[COL_XL_CE_OR_R - 1] == 1 &&
                           m->devbit[COL_XL_CE_OR_R] == 0) {
                    /* For the CKEN0=1,CKEN1=0 group the CE and RESET slots are
                     * transposed: the term read as ASYNC_RESET is the clock
                     * enable, and the term read as CLK_ENABLE is the preset. */
                    pt_t tmp = m->ce_pt;
                    m->ce_pt   = m->rst_pt;
                    m->rst_pt  = tmp;
                    m->has_reset = (m->rst_pt.n_lits > 0);
                }
            }
        }
    }

    /* OE product-term scan.
     * XL: scan the OE slot, then set oe_mode from OE0..OE2 (GTS), the slot
     * content and the COEINV polarity bit (see below).
     * Non-XL: scan only when oe_mode==3 (GOE2 / PT-OE). */
    if (dev->is_xl) {
        /* FB1 r=2's own slot is scanned as OE only when dedicated */
        if (!fb1_r2_shares_with_r1 || xl_oe_ded) scan_pt_slot_pg(dev, fuses, pg_oe, ncol, off_oe, &m->oe_pt, 0);
        m->has_oe_pt = (m->oe_pt.n_lits > 0) && xl_oe_ded;   /* OE only when PT5 is dedicated */
        /* The OE scan above reads a fixed physical PT slot (PT5, slot index 1
         * in slot_is_exported_xl()'s 0-4 numbering, matching xl_and_loc()'s
         * row order PT3, PT5, PT4, PT1, PT2). If this macrocell exported that
         * slot to a neighbour (see apply_pt_export_fix_xl()), the slot holds
         * the exported term, not an OE of this macrocell, and the macrocell
         * must not be reported as a PT-OE output. */
        if (m->has_oe_pt && slot_is_exported_xl(dev, fuses, fb, mc, 1))
            m->has_oe_pt = 0;
        const unsigned char *c30 = device_col(dev,fuses,attr_page,COL_OE_INVERT);
        /* Use aoff for all attribute reads: correct for pair_base=2 FBs */
        int oe_invert  = dev_attr_off(dev, c30,  aoff);
        /* For pair_base=2 FBs, K=0 lives at aoff=54/62 and its col30 is read
         * like any other position. */
        int oe_via_gts = dev_attr_off(dev, c27,  aoff);
        int oe_mux_b0  = dev_attr_off(dev, c28,  aoff);  /* GOE2 discriminator */
        /* OE modes for XL:
         *   OE_GTS=1, OE_MUX_B0=0 -> GOE1 (GTS1), oe_mode=2
         *   OE_GTS=1, OE_MUX_B0=1 -> GOE2 (GTS2), oe_mode=3
         *   OE_INVERT=1, no GTS   -> always-on, oe_mode=1
         *   otherwise             -> tristate or PT-OE */
        /* COL_OE_INVERT is an OE polarity bit, not an "always-on" bit, so this
         * decision consults the OE slot scanned above. With an occupied OE
         * slot, as the fitter's equations show:
         *
         *     OBUFT  T = AND(s0,s1,s2)      ->  `!y0.OE = s0_q & s1_q & s2_q;`
         *                                       COL_OE_INVERT = 1
         *     OBUFT  T = NOT AND(s0,s1,s2)  ->  ` y0.OE = s0_q & s1_q & s2_q;`
         *                                       COL_OE_INVERT = 0
         *
         * i.e. COL_OE_INVERT=1 means OE = NOT(PT) and 0 means OE = PT.
         * "Always-on" is the empty-slot case only.
         *
         * Non-XL never reaches this code: on non-XL column 30 is OE3, a
         * different vendor field, and non-XL oe_mode is decided earlier. */
        /* XL OE field list: OE0 27 | OE1 28 | OE2 29 | COEINV 30.
         *   OE0=1                 GTS
         *   (OE1,OE2) = (0,0)     GTS1
         *               (1,0)     GTS2
         *               (0,1)     GTS3
         *               (1,1)     GTS4
         *   COEINV=1              inverted (`!y.OE = g; // GTS`) */
        if      (oe_via_gts) { int oe_sel_hi = dev_attr_off(dev, c29, aoff);   /* OE2: GTS3/GTS4 */
                               m->oe_mode = oe_sel_hi ? (oe_mux_b0 ? 5 : 4) : (oe_mux_b0 ? 3 : 2);
                               m->has_oe_pt = 0;
                               m->oe_gts_invert = oe_invert; }
        else if (m->has_oe_pt) { m->oe_mode = 3;               /* PT-OE ... */
                                 m->oe_pt_invert = oe_invert;  /* ... polarity */ }
        else if (oe_invert)    m->oe_mode = 1;   /* always-on: OE slot EMPTY */
        else                   m->oe_mode = 0;   /* tristate / inactive */
    } else {
        if (m->oe_mode == 3) {
            /* oe_mode==3 is the shared non-XL code for "GTS2" and "product-term
             * OE"; the tiebreak is whether this macrocell's OE slot really
             * holds an enable.
             *
             * PT1 is steered by the vendor's CPT1A/CPT1B pair (non-XL device
             * bits 14/15):
             *
             *     CPT1A/B   meaning
             *       0 0     PT-OE
             *       0 1     GTS
             *       1 1     GTS
             *
             * so (0,0) -- and only (0,0) -- steers PT1 to the output enable.
             * A macrocell that only uses PT1 as part of its data sum is GTS2.
             *
             * Read through m->devbit[], which is what the `dev:` dump prints;
             * re-deriving the access with dev_attr_off()/device_col() gives a
             * different value.
             *
             * If either bit is absent (-1) the slot is scanned unconditionally. */
            int cpt1a = m->devbit[14], cpt1b = m->devbit[15];
            int steered_to_oe = (cpt1a < 0 || cpt1b < 0)
                              ? 1 : (cpt1a == 0 && cpt1b == 0);
            if (steered_to_oe) {
                if (scan_pt_slot_pg(dev,fuses,pg_oe,ncol,off_oe,&m->oe_pt, 0))
                    m->has_oe_pt = 1;
            }
            /* decode_oe_mode4() returns 3 for (OE2,OE3,OE4) = (1,0,1), the GTS2
             * code, and for (1,1,1). (1,1,1) is what the vendor's writer uses
             * for a product-term OE; with CPT1 not SPC there is no enable term
             * and no global: the fitter's equation is `y.OE = Gnd`, an output
             * that is never enabled. Such a cell is tristate (oe_mode 0).
             * Vendor legend (logical = NOT raw): raw (1,1,1) is OE[4:2] = 000 =
             * PTOE, and CPT1 raw (1,1)/(0,1)/(1,0) is off/OR/EXP; the PT1 term
             * reaches the enable only when CPT1 = SPC (raw 0,0). */
            else if (cpt1a >= 0 && cpt1b >= 0 && m->have_dev_bits
                     && m->devbit[29] == 1 && m->devbit[30] == 1 && m->devbit[31] == 1)
                m->oe_mode = 0;
        }
        /* A buried or pinned open-drain macrocell -- the fitter's
         * "sig = Gnd; sig.OE = <terms>" -- is not marked by OE0/OE1: those
         * read 1/1, as on an inactive cell (oe_mode 0), so the tiebreak
         * above never runs. Column 26 (COL_PWR_LOW) reads 0 on such a
         * macrocell and 1 on an ordinary one at the same location; OE0/OE1
         * (cols 27/28) do not differ.
         *
         * Gated on oe_mode==0, not merely !has_oe_pt, so a GOE2 case
         * (oe_mode==3, !has_oe_pt) is never touched. vhdl.c's tri-state
         * emission keys on has_oe_pt && oe_mode==3. */
        if (m->oe_mode == 0 && m->n_data_pts == 0 && dev_attr(dev, c26, mc)) {
            if (scan_pt_slot_pg(dev,fuses,pg_oe,ncol,off_oe,&m->oe_pt, 0)) {
                m->has_oe_pt = 1;
                m->oe_mode = 3;
            }
        }
    }
}

/* ---- Decode full chip ---- */

/* XL only: does macrocell (fb,mc)'s physical PT slot `slot` (0..4,
 * CPT0..CPT4) read normalised (False,True) -- "EXP", the export state?
 * Uses the same per-slot two-bit selector columns and attr_page/aoff
 * addressing decode_mc() uses for the dedication (SPC) checks. The four
 * states are off=(0,0), SPC=(1,1), merged-into-data=(1,0), EXP=(0,1). */
static int slot_is_exported_xl(const device_t *dev, const unsigned char *fuses,
                                int fb, int mc, int slot) {
    static const int col_b1[5] = { COL_PT3_MUX_B1, COL_PT5_MUX_B1,
                                    COL_PT4_MUX_B1, COL_PT1_MUX_B1,
                                    COL_PT2_MUX_B1 };
    static const int col_b0[5] = { COL_PT3_MUX_B0, COL_PT5_MUX_B0,
                                    COL_PT4_MUX_B0, COL_PT1_MUX_B0,
                                    COL_PT2_MUX_B0 };
    int aoff = xl_attr_mc_off(dev->n_fbs, fb, mc);
    int attr_page = (dev->n_fbs > 1) ? 1 : fb;
    if (aoff >= 108) { attr_page = 2; aoff -= 108; }
    const unsigned char *cA = device_col(dev, fuses, attr_page, col_b1[slot]);
    const unsigned char *cB = device_col(dev, fuses, attr_page, col_b0[slot]);
    return !dev_attr_off(dev, cA, aoff) && dev_attr_off(dev, cB, aoff);
}

/* Does macrocell (fb,mc)'s own CAS0 or CAS1 (per `upper`: false =
 * CAS0/COL_FB_FROM_UPPER, true = CAS1/COL_FB_FROM_LOWER; see the naming
 * note at apply_pt_export_fix_xl) read normalised True?
 *
 * Uses the same per-mc aoff/attr_page addressing as slot content
 * (xl_attr_mc_off), not attr_fb_pre/mc_off(mc). */
static int mc_cas_bit(const device_t *dev, const unsigned char *fuses,
                       int fb, int mc, int upper) {
    int col = upper ? COL_FB_FROM_LOWER : COL_FB_FROM_UPPER;
    int aoff = xl_attr_mc_off(dev->n_fbs, fb, mc);
    int attr_page = (dev->n_fbs > 1) ? 1 : fb;
    if (aoff >= 108) { attr_page = 2; aoff -= 108; }
    const unsigned char *c = device_col(dev, fuses, attr_page, col);
    return dev_attr_off(dev, c, aoff);
}

/* The XL PT-export post-pass. Runs after every macrocell in the device
 * is decoded, so a donor and every macrocell its export can reach are
 * available regardless of scan order. decode_mc() tags each data_pts[]
 * entry with its physical slot (0..4).
 *
 * The vendor's XL bitmap legend names the three cascade bits
 * "CAS[2:0](1xx=EXP DN,x1x=IMP UP,xx1=IMP DN)":
 *   CAS0 = COL_FB_FROM_UPPER (col 23)  IMP DN: import the bus of mc-1
 *   CAS1 = COL_FB_FROM_LOWER (col 24)  IMP UP: import the bus of mc+1
 *   CAS2 = COL_FB_TO_UPPER   (col 25)  EXP DN: this macrocell's export bus
 *                                      goes to mc-1 when set, to mc+1 when not
 * Every macrocell has one export bus, carrying its own EXP-state slots
 * (B1,B0 = 0,1) and travelling in its CAS2 direction, MC1 <-> MC18
 * wrapping inside the function block. The macrocell the bus reaches
 * captures it into its own OR when its import bit facing the source is set
 * (CAS1 for a bus arriving from mc+1, CAS0 for one from mc-1). A macrocell
 * that does not capture passes the bus on with its own export bus, in its
 * own CAS2 direction, until some macrocell captures it. The fitter's
 * netlist (.vm6) shows the same: a pass-through macrocell's MC_SI_EXPORT is
 * its own exported terms plus `IV_TRUE <neighbour>.EXP`.
 *
 * Two phases, so no term is judged against a slot it did not come from:
 * (1) lift every EXP-state term out of its own macrocell (descending slot
 * order); (2) for each donor in ascending order, walk its bus to the
 * capturing macrocell and append its terms there. Unresolved (marked on
 * the donor, never put back into it): a pass-through whose CAS2 points
 * back the way the bus came, 17 hops with no capture, or a receiver
 * without room for the terms (MAX_PTS).
 *
 * Column naming: xc9500.h's COL_FB_FROM_UPPER/COL_FB_FROM_LOWER macro
 * names are the opposite of the semantics -- COL_FB_FROM_UPPER (col 23)
 * is CAS0 (true => donor is one lower), COL_FB_FROM_LOWER (col 24) is CAS1
 * (true => donor is one higher). The column addresses are correct. */
static int mc_cas2_xl(const device_t *dev, const unsigned char *fuses,
                      int fb, int mc) {
    int aoff = xl_attr_mc_off(dev->n_fbs, fb, mc);
    int attr_page = (dev->n_fbs > 1) ? 1 : fb;
    if (aoff >= 108) { attr_page = 2; aoff -= 108; }
    const unsigned char *c = device_col(dev, fuses, attr_page, COL_FB_TO_UPPER);
    return dev_attr_off(dev, c, aoff);
}

static void apply_pt_export_fix_xl(const device_t *dev,
                                    const unsigned char *fuses,
                                    chip_t *chip) {
    if (!dev->is_xl) return;

    for (int fb = 1; fb <= dev->n_fbs; fb++) {
        static pt_t lifted[MACROCELLS_PER_FB][MAX_PTS];
        int n_lifted[MACROCELLS_PER_FB] = { 0 };

        /* phase 1: lift every EXP-state term out of its own macrocell.
         * back-to-front: removing an entry must never shift an index still
         * to be visited in this same loop. */
        for (int mc = 1; mc <= MACROCELLS_PER_FB; mc++) {
            mc_t *m = &chip->mc[fb-1][mc-1];
            m->pre_export_n = m->n_data_pts;   /* C10: before any term is lifted */
            for (int p = m->n_data_pts - 1; p >= 0; p--) {
                int slot = m->data_pts[p].slot;
                if (slot < 0 || slot > 4) continue;   /* not XL data-slot tagged */
                if (!slot_is_exported_xl(dev, fuses, fb, mc, slot)) continue;
                lifted[mc-1][n_lifted[mc-1]++] = m->data_pts[p];
                for (int q = p; q < m->n_data_pts - 1; q++)
                    m->data_pts[q] = m->data_pts[q + 1];
                m->n_data_pts--;
            }
        }

        /* phase 2: walk each donor's export bus to the macrocell that
         * captures it. */
        for (int mc = 1; mc <= MACROCELLS_PER_FB; mc++) {
            int n = n_lifted[mc-1];
            if (n == 0) continue;
            int cur = mc, dir = mc_cas2_xl(dev, fuses, fb, mc) ? -1 : +1;
            int recv = 0;
            for (int hop = 1; hop < MACROCELLS_PER_FB; hop++) {
                int nxt = cur + dir;
                if (nxt < 1) nxt = MACROCELLS_PER_FB;
                if (nxt > MACROCELLS_PER_FB) nxt = 1;
                /* facing import bit: CAS1 for a bus arriving from above
                 * (dir -1), CAS0 for one arriving from below (dir +1) */
                if (mc_cas_bit(dev, fuses, fb, nxt, dir < 0)) { recv = nxt; break; }
                /* nxt passes the bus on in its OWN export direction; one
                 * pointing back the way the bus came is not a chain */
                if ((mc_cas2_xl(dev, fuses, fb, nxt) ? -1 : +1) != dir) break;
                cur = nxt;
            }
            mc_t *d = &chip->mc[fb-1][mc-1];
            d->export_role |= 1;                /* C10: a donor */
            if (!recv) { d->has_unresolved_export = 1; continue; }
            mc_t *r = &chip->mc[fb-1][recv-1];
            r->export_role |= 2;                /* C10: a receiver */
            if (r->n_data_pts + n > MAX_PTS) { d->has_unresolved_export = 1; continue; }
            for (int i = 0; i < n; i++)
                r->data_pts[r->n_data_pts++] = lifted[mc-1][i];
        }
    }
}

/* The non-XL twin of apply_pt_export_fix_xl() above. Same per-slot MUX
 * B1/B0 columns and CAS0/CAS1/CAS2 column numbers on both families; the
 * difference is addressing: non-XL has no paged attribute space, so plain
 * dev_attr(dev,col,mc) (mc_off() internally) replaces XL's
 * xl_attr_mc_off() + dev_attr_off().
 *
 * decode_mc() tags the non-XL additional data slots with their CPT index:
 * g (r=0), g-5 (r=1, g runs 6..9) and ti+1 (r=2), following the vendor's
 * CPT1..4 = PTOE/PTR/PTS/PTINV order. */
static int slot_is_exported_nonxl(const device_t *dev, const unsigned char *fuses,
                                   int fb, int mc, int slot) {
    static const int col_b1[5] = { COL_PT3_MUX_B1, COL_PT5_MUX_B1,
                                    COL_PT4_MUX_B1, COL_PT1_MUX_B1,
                                    COL_PT2_MUX_B1 };
    static const int col_b0[5] = { COL_PT3_MUX_B0, COL_PT5_MUX_B0,
                                    COL_PT4_MUX_B0, COL_PT1_MUX_B0,
                                    COL_PT2_MUX_B0 };
    const unsigned char *cA = device_col(dev, fuses, fb, col_b1[slot]);
    const unsigned char *cB = device_col(dev, fuses, fb, col_b0[slot]);
    return !dev_attr(dev, cA, mc) && dev_attr(dev, cB, mc);
}

static int mc_cas_bit_nonxl(const device_t *dev, const unsigned char *fuses,
                             int fb, int mc, int upper) {
    int col = upper ? COL_FB_FROM_LOWER : COL_FB_FROM_UPPER;
    const unsigned char *c = device_col(dev, fuses, fb, col);
    return dev_attr(dev, c, mc);
}

/* The rule is apply_pt_export_fix_xl()'s on the non-XL addressing. The
 * vendor's non-XL bitmap legend carries the same cascade string
 * "CAS[2:0](1xx=EXP DN,x1x=IMP UP,xx1=IMP DN)": every macrocell's export bus
 * travels in its CAS2 direction (col 25; set -> mc-1, clear -> mc+1), is
 * captured by the first macrocell whose facing import bit is set (CAS1
 * col 24 for a bus from mc+1, CAS0 col 23 for one from mc-1), and is
 * otherwise passed on in that macrocell's own CAS2 direction, MC1 <-> MC18
 * wrapping. Two phases: lift every EXP-state term, then walk each donor's
 * bus. Unresolved (donor flagged, terms never put back): a back-turn, 17
 * hops without capture, or no room (MAX_PTS). Runs on every non-XL
 * density. */
static int mc_cas2_nonxl(const device_t *dev, const unsigned char *fuses,
                         int fb, int mc) {
    const unsigned char *c = device_col(dev, fuses, fb, COL_FB_TO_UPPER);
    return dev_attr(dev, c, mc);
}

static void apply_pt_export_fix_nonxl(const device_t *dev,
                                       const unsigned char *fuses,
                                       chip_t *chip) {
    if (dev->is_xl) return;

    for (int fb = 1; fb <= dev->n_fbs; fb++) {
        static pt_t lifted[MACROCELLS_PER_FB][MAX_PTS];
        int n_lifted[MACROCELLS_PER_FB] = { 0 };

        /* phase 1: lift every EXP-state term out of its own macrocell,
         * back-to-front so no index still to be visited shifts. */
        for (int mc = 1; mc <= MACROCELLS_PER_FB; mc++) {
            mc_t *m = &chip->mc[fb-1][mc-1];
            m->pre_export_n = m->n_data_pts;   /* C10: before any term is lifted */
            for (int p = m->n_data_pts - 1; p >= 0; p--) {
                int slot = m->data_pts[p].slot;
                if (slot < 0 || slot > 4) continue;
                if (!slot_is_exported_nonxl(dev, fuses, fb, mc, slot)) continue;
                lifted[mc-1][n_lifted[mc-1]++] = m->data_pts[p];
                for (int q = p; q < m->n_data_pts - 1; q++)
                    m->data_pts[q] = m->data_pts[q + 1];
                m->n_data_pts--;
            }
        }

        /* phase 2: walk each donor's export bus to the macrocell that
         * captures it. */
        for (int mc = 1; mc <= MACROCELLS_PER_FB; mc++) {
            int n = n_lifted[mc-1];
            if (n == 0) continue;
            int cur = mc, dir = mc_cas2_nonxl(dev, fuses, fb, mc) ? -1 : +1;
            int recv = 0;
            for (int hop = 1; hop < MACROCELLS_PER_FB; hop++) {
                int nxt = cur + dir;
                if (nxt < 1) nxt = MACROCELLS_PER_FB;
                if (nxt > MACROCELLS_PER_FB) nxt = 1;
                /* facing import bit: CAS1 for a bus arriving from above
                 * (dir -1), CAS0 for one arriving from below (dir +1) */
                if (mc_cas_bit_nonxl(dev, fuses, fb, nxt, dir < 0)) { recv = nxt; break; }
                /* nxt passes the bus on in its OWN export direction; one
                 * pointing back the way the bus came is not a chain */
                if ((mc_cas2_nonxl(dev, fuses, fb, nxt) ? -1 : +1) != dir) break;
                cur = nxt;
            }
            mc_t *d = &chip->mc[fb-1][mc-1];
            d->export_role |= 1;                /* C10: a donor */
            if (!recv) { d->has_unresolved_export = 1; continue; }
            mc_t *r = &chip->mc[fb-1][recv-1];
            r->export_role |= 2;                /* C10: a receiver */
            if (r->n_data_pts + n > MAX_PTS) { d->has_unresolved_export = 1; continue; }
            for (int i = 0; i < n; i++)
                r->data_pts[r->n_data_pts++] = lifted[mc-1][i];
        }
    }
}

/* -- The two chip-level fields a faithful rebuild carries --
 *
 * They live here, not in main.c, because `decode` and `vhdl` both need
 * them and this file owns the addressing formulas they use.
 */

/* chip_signature -- the user signature, eight bits per character.
 *
 * The vendor's "# Signature bits" line (the same in both .dev files) names
 * SI0..SI31 at columns 6 and 7 with a 0-based macrocell index. Composed with
 * device.h's mc_off() (non-XL) / xl_attr_mc_off() (XL) it gives 32 fuse
 * addresses. The table below is that map, written as the 1-based macrocell
 * index of each bit of each character:
 *
 *   character ci -> column 6 + ci/2, row set mcrow[ci & 1]
 *   mcrow[0] b0..b7 = mc  4,13, 3,12, 2,11, 1,10   (= SI24..SI31 / SI8..SI15)
 *   mcrow[1] b0..b7 = mc  8,17, 7,16, 6,15, 5,14   (= SI16..SI23 / SI0..SI7)
 *
 * Polarity is device.h's dev_attr_off(), handed the byte rather than the
 * buffer.
 */
int chip_signature(const JedFile *jf, const device_t *dev, unsigned char out[4]) {
    static const int mcrow[2][8] = { { 4, 13, 3, 12, 2, 11, 1, 10 },
                                     { 8, 17, 7, 16, 6, 15, 5, 14 } };
    if (!jf || !dev || !out) return 0;
    for (int ci = 0; ci < 4; ci++) {
        int col = 6 + ci / 2, b, v = 0;
        for (b = 0; b < 8; b++) {
            int mc  = mcrow[ci & 1][b];
            int off = dev->is_xl ? xl_attr_mc_off(dev->n_fbs, 1, mc)
                                 : mc_off(mc);
            const unsigned char *c =
                device_col(dev, jf->fuses, 1 + off / COLUMN_WIDTH, col);
            size_t a = (size_t)(c - jf->fuses) + (size_t)(off % COLUMN_WIDTH);
            if (a >= jf->count) return 0;
            unsigned char raw = jf->fuses[a];
            if (dev_attr_off(dev, &raw, 0)) v |= 1 << b;
        }
        out[ci] = (unsigned char)v;
    }
    return 1;
}

/* chip_signature_text -- what the signature line says, from
 * chip_signature()'s 32 bits. hprep6 writes the first four bytes of -n,
 * pads a shorter string with spaces and accepts 0x01..0xFF; -autosig writes
 * 0x00000000 (no signature). So: hex always; the quoted ASCII only when every
 * byte is printable (0x20..0x7E); zero says none; nothing is masked. */
void chip_signature_text(const JedFile *jf, const device_t *dev, char *buf, size_t n) {
    unsigned char sb[4];
    int i, printable = 1;
    if (!chip_signature(jf, dev, sb)) {
        snprintf(buf, n, "unreadable (the JED is shorter than the signature fuses)");
        return;
    }
    for (i = 0; i < 4; i++) if (sb[i] < 0x20 || sb[i] > 0x7E) printable = 0;
    if (!sb[0] && !sb[1] && !sb[2] && !sb[3])
        snprintf(buf, n, "0x00000000 (none -- all 32 bits zero, what hprep6 -autosig writes)");
    else if (printable)
        snprintf(buf, n, "0x%02X%02X%02X%02X \"%c%c%c%c\"", sb[0], sb[1], sb[2], sb[3], sb[0], sb[1], sb[2], sb[3]);
    else
        snprintf(buf, n, "0x%02X%02X%02X%02X (not printable ASCII)", sb[0], sb[1], sb[2], sb[3]);
}

/* chip_keeper -- the XL I/O bus-hold keeper (`cpldfit -terminate float|keeper`).
 *
 * One global fuse per device: `device_col(dev,1,2) + xl_attr_mc_off(n_fbs,1,9)`,
 * i.e. row index 8 of the vendor's global column 2 -- one row past the last
 * row the .dev names. Addresses 566 / 1126 / 2246 / 4486 on the four XL
 * densities.
 *
 * Its polarity is not dev_attr_off()'s: raw 0 is the default (keeper) and
 * raw 1 is `-terminate float`.
 *
 * Returns 1 = keeper, 0 = float, -1 = not a field of this family. */
int chip_keeper(const JedFile *jf, const device_t *dev) {
    if (!jf || !dev || !dev->is_xl) return -1;
    int off = xl_attr_mc_off(dev->n_fbs, 1, 9);
    const unsigned char *c = device_col(dev, jf->fuses, 1 + off / COLUMN_WIDTH, 2);
    size_t a = (size_t)(c - jf->fuses) + (size_t)(off % COLUMN_WIDTH);
    if (a >= jf->count) return -1;
    return jf->fuses[a] ? 0 : 1;
}

void decode_chip(const JedFile *jf, const device_t *dev, chip_t *chip) {
    memset(chip, 0, sizeof(*chip));
    chip->dev   = dev;
    chip->fuses = jf->fuses;
    if (!dev) return;
    /* chip_t.mc is dimensioned [16][18], so up to 16 FBs (XC95288XL) fit. */
    if (dev->n_fbs < 1 || dev->n_fbs > 16) {
        fprintf(stderr, "ERROR: %s reports %d function blocks; this build "
                "handles 1..16 (chip_t.mc is [16][18]).\n",
                dev->id, dev->n_fbs);
        return;
    }
    for (int fb = 1; fb <= dev->n_fbs; fb++)
        for (int mc = 1; mc <= MACROCELLS_PER_FB; mc++)
            decode_mc(dev, jf->fuses, fb, mc, &chip->mc[fb-1][mc-1]);
    apply_pt_export_fix_xl(dev, jf->fuses, chip);
    apply_pt_export_fix_nonxl(dev, jf->fuses, chip);
    compute_fb_unique_unresolved_pairs(dev, chip);
    g_named_chip = chip;   /* enables the identity-path naming below */
}

/* ---- Text decode output ---- */

static void print_pt(const chip_t *chip, int fb_dest, const pt_t *pt, int indent) {
    const device_t *dev = chip->dev;
    if (pt->n_lits == 0) { printf("%*s(empty)\n", indent, ""); return; }
    printf("%*s", indent, "");
    /* SIGNAME_MAX: nonxl_plane_wired_or_name() refuses to truncate, so a
     * smaller buffer would make a wired pair with many sources fall back to
     * FBn_PAIRm here while the VHDL resolves it. */
    char buf[SIGNAME_MAX];
    for (int i = 0; i < pt->n_lits; i++) {
        if (i > 0) printf(" AND ");
        printf("%s", pair_signal_name(dev, chip->fuses, fb_dest,
                                       pt->lits[i].pair, pt->lits[i].is_comp,
                                       buf, sizeof buf));
        if (getenv("XC_DUMP_PAIR")) printf("{p%d}", pt->lits[i].pair);
    }
    printf("\n");
}

void decode_print(const chip_t *chip) {
    const device_t *dev = chip->dev;
    /* chip_t.mc is [16][18]; decode_chip already rejects anything outside
     * 1..16, so this only needs to match. */
    if (!dev || dev->n_fbs < 1 || dev->n_fbs > 16) return;
    /* Whether any pair_base=2 FB (FB3/FB4) is active. */
    int has_pb2_active = 0;
    if (dev->n_fbs >= 4) {
        for (int fb2 = 3; fb2 <= dev->n_fbs && !has_pb2_active; fb2++)
            for (int mc2 = 1; mc2 <= MACROCELLS_PER_FB && !has_pb2_active; mc2++) {
                const mc_t *mx = &chip->mc[fb2-1][mc2-1];
                if (mx->is_real_output || mx->n_data_pts > 0) has_pb2_active = 1;
            }
    }
    for (int fb = 1; fb <= dev->n_fbs; fb++) {
        printf("\n--- FB%d ---\n", fb);
        /* Print the FastCONNECT select value of every pair in this function
         * block:
         *   address  col = 55 + pair/3, rows by role pair%3
         *            role0 [6,14,22,30] role1 [54,62,70,7]
         *            role2 [31,39,47,55]
         *   5th bit  rows 38 / 15 / 63, value = 29 - index
         *   value 31 = G0 = nothing selected (erased)
         *   value 30 = G1 = local feedback
         *   otherwise index = 29 - value into the pair's <out> source list
         * Printed on every non-XL device, including pairs whose literals
         * resolve by pin. */
        /* XL analogue of the non-XL `fc:` line: print the imux code selected for
         * every one of the 54 function-block inputs.
         *   col  = 49 + input (1..27) or 22 + input (28..54)
         *   side = 6 for inputs 1..27 (A-bits), 7 for 28..54 (B-bits)
         *   offset = bit*8*n_fbs + (fb-1)*8 + side, band = col*n_fbs*108
         *   the code is the vendor's G-label: 1,2,3,4 external and 16..19
         *   feedback, as in the .chp `label` attribute */
        if (dev->is_xl && dev->n_fbs) {
            int input, first = 1;
            for (input = 1; input <= 54; input++) {
                int col  = (input <= 27) ? 49 + input : 22 + input;
                int side = (input > 27) ? 7 : 6;
                long band = (long)col * dev->n_fbs * 108;
                int bit, v = 0;
                for (bit = 0; bit < 5; bit++) {
                    long q = band + (long)bit*8*dev->n_fbs + (long)(fb-1)*8 + side;
                    if (q < 0) continue;
                    if (chip->fuses[q]) v |= (1 << bit);
                }
                if (first) { printf("  imux:"); first = 0; }
                printf(" i%d=%d", input, v);
            }
            if (!first) printf("\n");
        }
        if (!dev->is_xl && dev->jed_fuses_per_fb) {
            static const int rrow[3][5] = {{6,14,22,30,38},{54,62,70,7,15},{31,39,47,55,63}};
            int pair, first = 1;
            for (pair = 0; pair < dev->fc_pairs_per_fb && pair < 36; pair++) {
                int col = 55 + pair/3, role = pair%3, b, v = 0, ok = 1;
                for (b = 0; b < 5; b++) {
                    long idx = (long)(fb-1)*dev->jed_fuses_per_fb
                             + (long)col*108 + rrow[role][b];
                    if (idx < 0) { ok = 0; break; }
                    if (chip->fuses[idx]) v |= (1 << b);
                }
                if (!ok) break;
                if (first) { printf("  fc:"); first = 0; }
                printf(" p%d=%d", pair, v);
            }
            if (!first) printf("\n");
        }
        /* Report what the identity region names for every pair. Columns
         * 72..fbcols-1 form a 36-plane map (plane = [0,8,15,22,29][pair%5] +
         * pair/5) whose one-hot bit names the local-feedback source. Every
         * identity column is read here, including where the pin path
         * answers first. */
        if (!dev->is_xl && dev->jed_fuses_per_fb) {
            static const int pbase[5] = {0,8,15,22,29};
            int fbcols = dev->jed_fuses_per_fb / 108;
            int pair, first = 1;
            for (pair = 0; pair < dev->fc_pairs_per_fb && pair < 36; pair++) {
                int plane = pbase[pair%5] + pair/5, c, jj, hits = 0, idx = -1;
                for (c = 72; c < fbcols; c++) {
                    for (jj = 0; jj < 3; jj++) {
                        long q = (long)(fb-1)*dev->jed_fuses_per_fb
                               + (long)c*108 + plane + 36*jj;
                        if (q < 0) continue;
                        if (chip->fuses[q]) { hits++; idx = 3*(c-72)+jj; }
                    }
                }
                if (first) { printf("  lf:"); first = 0; }
                if (hits == 1) printf(" p%d=FB%d_MC%d", pair, idx/18+1, idx%18+1);
                else           printf(" p%d=n%d", pair, hits);
                /* When the plane is not one-hot, print how many sources it
                 * marks. */
            }
            if (!first) printf("\n");
        }
        for (int mc = 1; mc <= MACROCELLS_PER_FB; mc++) {
            const mc_t *m = &chip->mc[fb-1][mc-1];
            char sigbuf[32];
            signal_name(dev, fb, mc, sigbuf, sizeof sigbuf);
            /* Skip inactive MCs */
            /* Liveness: a buried open-drain macrocell is `= Gnd` with a
             * product-term OE (zero data terms, its content in the OE
             * condition), and a register held only by GSR (`D = Gnd;
             * AP = gsr`) has neither; both are live and the emitter writes
             * them (vhdl.c mc_active()). */
            if (!m->is_real_output && m->n_data_pts == 0 && !m->has_clk_pt &&
                !m->has_reset && !m->has_preset && !m->has_ce && !m->ce_const0 &&
                !m->has_oe_pt && !m->gsr_reset && !m->gsr_set) continue;

            const char *clk_str =
                (m->clk_mode==CLK_MODE_GCK1) ? "GCK1"  :
                (m->clk_mode==CLK_MODE_GCK2) ? "GCK2"  :
                (m->clk_mode==CLK_MODE_GCK3) ? "GCK3"  :
                (m->clk_mode==CLK_MODE_COMB) ? "COMB"  :
                (m->clk_mode==CLK_MODE_PT)   ? "PT-CLK": "?";

            printf("[FB%d_MC%d pin%-3d %s] r=%d K=%d\n",
                   fb, mc, m->pin, sigbuf, m->r, m->K);
            if (m->gsr_reset) printf("  [reset by GSR]\n");
            if (m->gsr_set)   printf("  [set by GSR]\n");
            printf("  clk=%-6s %s | ff=%s init=%s | oe=%d%s | pwr=%s slew=%s\n",
                   clk_str,
                   m->clk_falling ? "FALL" : "RISE",
                   m->is_toggle ? "T" : m->is_combi ? "C" : "D",
                   m->init_set  ? "SET" : "RST",
                   m->oe_mode,
                   /* oe_mode=2: GOE1 (GTS1)  oe_mode=3: GOE2(GTS2) or PT-OE
                    * Distinguish: GOE2 sets has_oe_pt=0, PT-OE sets has_oe_pt=1 */
                   m->oe_mode==3 && !m->has_oe_pt ? "(GOE2)"      :
                   m->has_oe_pt  ? "(PT-OE)"    :
                   m->oe_mode==1 ? "(always-on)" :
                   m->oe_mode==2 ? "(GOE1)"      :
                   m->oe_mode==4 ? "(GOE3)"      :   /* GTS3 */
                   m->oe_mode==5 ? "(GOE4)"      :   /* GTS4 */
                   "(tristate)",
                   m->pwr_low   ? "LOW" : "STD",
                   m->slew_slow ? "SLOW" : "FAST");
        if (m->have_dev_bits) {
            /* Dump every vendor-named control bit verbatim, using the name
             * table for this family. No interpretation. */
            static const char *dn_nonxl[] = {"CPT0A","CPT0B","CPT1A","CPT1B","CPT2A","CPT2B","CPT3A","CPT3B","CPT4A","CPT4B","CPTINV","CAS0","CAS1","CAS2","ORPWR","OE0","OE1","OE2","OE3","OE4","TRAN","CCK0","CCK1","CTFF","CRST","CSET","PRESET","FBOE0","FBOE1","FBINV","CGND","QSREN","SAPWR0","SAPWR1","SAPWR2","SAPWR3","SAPWR4"};
            static const int   dc_nonxl[] = {12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,35,36,37,40,41,42,43,44,45,46,48,49,50,51,52,53,54};
            static const char *dn_xl[]    = {"CPT0A","CPT0B","CPT1A","CPT1B","CPT2A","CPT2B","CPT3A","CPT3B","CPT4A","CPT4B","CPTINV","CAS0","CAS1","CAS2","ORPWR","OE0","OE1","OE2","COEINV","TRAN","CCK0","CCK1","CCKINV","CKEN0","CKEN1","CTFF","CRST","CSET","PRESET","CGND","QSREN","SAPWR0","SAPWR1","SAPWR2","SAPWR3","SAPWR4"};
            static const int   dc_xl[]    = {12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,32,33,34,35,36,37,39,40,41,42,43,44,45,46,47,48,49};
            const char **dn = m->dev_is_xl ? dn_xl : dn_nonxl;
            const int   *dc = m->dev_is_xl ? dc_xl : dc_nonxl;
            unsigned nd = m->dev_is_xl
                ? sizeof(dc_xl)/sizeof(dc_xl[0])
                : sizeof(dc_nonxl)/sizeof(dc_nonxl[0]);
            unsigned q; int first = 1;
            for (q = 0; q < nd; q++) {
                if (m->devbit[dc[q]] < 0) continue;
                if (first) { printf("  dev:"); first = 0; }
                printf(" %s=%d", dn[q], m->devbit[dc[q]]);
            }
            if (!first) printf("\n");
        }

            g_consumer_fb = fb; g_consumer_mc = mc;
            if (m->has_reset) {
                printf("  ASYNC_RESET:\n");
                print_pt(chip, fb, &m->rst_pt, 4);
            }
            if (m->has_preset) {
                printf("  ASYNC_PRESET:\n");
                print_pt(chip, fb, &m->pst_pt, 4);
            }
            if (m->has_ce) {
                printf("  CLK_ENABLE:\n");
                print_pt(chip, fb, &m->ce_pt, 4);
            }
            if (m->ce_const0)
                printf("  CLK_ENABLE: constant Gnd (CE selected by CKEN, CE slot not dedicated -- the register never loads)\n");
            if (m->has_oe_pt) {
                printf("  OE condition:\n");
                print_pt(chip, fb, &m->oe_pt, 4);
            }
            /* Where slot 0's selector does not identify a dedicated clock, say so
             * rather than print the clock as a D/T input term. */
            if (m->clk_mode == CLK_MODE_PT) {
                if (m->has_clk_pt) {
                    printf("  PT-CLK clock term:\n");
                    print_pt(chip, fb, &m->clk_pt, 4);
                } else if (m->clk_slot_empty) {
                    /* not a product term -- the clock here is the constant Gnd */
                    printf("  PT-CLK selected, slot 0 empty: constant clock (no product term)\n");
                } else {
                    printf("  PT-CLK clock term: NOT IDENTIFIED"
                           " (slot 0 selector is not dedicated)\n");
                }
            }
            printf("  D/T input (%d PT%s):\n", m->n_data_pts,
                   m->n_data_pts==1?"":"s");
            for (int p = 0; p < m->n_data_pts; p++) {
                printf("    PT%d: ", p+1);
                print_pt(chip, fb, &m->data_pts[p], 0);
            }
            /* The PTINV term is XOR'd with the sum above, not OR'd into it,
             * so it is printed separately. */
            if (m->has_ptinv) {
                printf("  XOR term (PTINV):\n    ");
                print_pt(chip, fb, &m->ptinv_pt, 0);
            }
        }
    }
}

/* -- FBINV complement of a wired-AND member --
 *
 * FBINV (non-XL column 46, the vendor's FbkInv, "feedback invert";
 * programmed = raw 0) is set by hprep6 on a FastCONNECT wired-AND input
 * that the AND reads complemented. On such a member the AND-array rail the
 * decoder calls TRUE carries the complement, so the member enters the
 * wired-AND complemented ('!' in the composite, vhdl.c or_expand_expr) and
 * its own single-literal feedback reads are complemented here.
 *
 * Not complemented: a pad read (FBINV sits in the feedback path), and an
 * FbkInv the fitter set on a macrocell it reduced to a constant -- every
 * vendor artefact (.rpt, .vm6, tsim) reads that one uninverted.
 * The rule keys on what the fuses resolve, never on a pin list. */

/* Is macrocell (f,m) a survivor of any multi-hit identity-plane row -- an
 * input of a FastCONNECT wired-AND? The same row scan the composite uses.
 * Cached per fuse map: valid because every command owns one unmodified
 * fuse map per process. */
int mc_is_wired_and_member(const device_t *dev, const unsigned char *fuses, int f, int m) {
    static const unsigned char *cached = 0; static const device_t *cdev = 0;
    static unsigned char mem[16 * 18];
    if (cached != fuses || cdev != dev) {
        int fd, pr; char b[4096];
        memset(mem, 0, sizeof mem);
        for (fd = 1; fd <= dev->n_fbs; fd++)
            for (pr = 0; pr < 36; pr++) {
                const char *s = b; int a, bb, n;
                if (!nonxl_plane_wired_or_name(dev, fuses, fd, pr, "", b, sizeof b)) continue;
                while (*s) {
                    if (*s == '|' || *s == '!') { s++; continue; }
                    if (sscanf(s, "n_FB%dMC%d%n", &a, &bb, &n) == 2) {
                        if (a >= 1 && a <= 16 && bb >= 1 && bb <= 18) mem[(a - 1) * 18 + bb - 1] = 1;
                        s += n;
                    } else s++;
                }
            }
        cached = fuses; cdev = dev;
    }
    return mem[(f - 1) * 18 + m - 1];
}

/* A single macrocell feedback is named n_FB<f>MC<m> everywhere in this file
 * (a feedback source is the FB output, not the pad); a pad is n_p<pin> and
 * is never touched. Composite and unresolved names pass through. */
const char *pair_signal_name(const device_t *dev, const unsigned char *fuses,
                                     int fb_dest, int pair, int is_comp,
                                     char *buf, int buflen) {
    int f = 0, m = 0, n = -1;
    const char *nm = pair_signal_name_raw(dev, fuses, fb_dest, pair, is_comp, buf, buflen);
    const char *base = nm;
    if (dev->is_xl || !fuses) return nm;
    if (strncmp(base, "NOT_", 4) == 0) base += 4;
    if (sscanf(base, "n_FB%dMC%d%n", &f, &m, &n) != 2 || n < 0 || base[n] != '\0')
        return nm;
    if (f < 1 || f > dev->n_fbs || m < 1 || m > MACROCELLS_PER_FB) return nm;
    if (device_col(dev, fuses, f, 46)[mc_off(m)] != 0) return nm;   /* not programmed */
    if (!mc_is_wired_and_member(dev, fuses, f, m)) return nm;       /* path 1: as read */
    snprintf(buf, buflen, "%sn_FB%dMC%d", (base == nm) ? "NOT_" : "", f, m);
    return buf;
}
