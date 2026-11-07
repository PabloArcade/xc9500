#include "xc9500.h"
#include "device.h"
#include "jed.h"
#include "decode.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Annotated diff ---- */

/* The region label names the position class of the fuse, which matches the
 * read site that consumes it:
 *   "and-array"      -- a literal-scan position: read by the literal scan
 *                       (device.h fuse_connected) and by nothing else
 *   "mc-attribute"   -- one of the 18 per-macrocell positions of a column
 *                       page (non-XL mc_off(1..18); XL xl_attr_mc_off(n,fb,mc),
 *                       band-absolute). Read by the attribute / FC-select /
 *                       signature / global reads, or by nothing (not every
 *                       attribute position is read; `coverage` counts them)
 *   "identity-plane" -- non-XL columns 72 and up: read only by the identity-
 *                       plane decoder
 * Positions come from device.h's mc_off()/xl_attr_mc_off(), over the same
 * range decode.c uses (fb 1..n_fbs, mc 1..18). */
static int is_attr_offset_nonxl(int offset) {
    int mc;
    for (mc = 1; mc <= MACROCELLS_PER_FB; mc++)
        if (mc_off(mc) == offset) return 1;
    return 0;
}

static void annotate_fuse(const device_t *dev, const unsigned char *xl_attr,
                           size_t k, int *fb_out, int *col_out, int *offset_out,
                           const char **region_out) {
    *fb_out = 1; *col_out = -1; *offset_out = 0;
    *region_out = "unknown";

    if (!dev) return;

    int fb, col, offset;
    if (dev->is_xl) {
        /* Band-major: band (col) varies slowest, then FB, then offset */
        int band_stride = dev->n_fbs * COLUMN_WIDTH;
        col    = (int)(k / band_stride);
        int rem = (int)(k % band_stride);
        fb     = rem / COLUMN_WIDTH + 1;
        offset = rem % COLUMN_WIDTH;
        *region_out = xl_attr[rem] ? "mc-attribute" : "and-array";
    } else {
        /* FB-major */
        fb     = (int)(k / dev->jed_fuses_per_fb) + 1;
        int fib = (int)(k % dev->jed_fuses_per_fb);
        int and_bits = dev->fc_pairs_per_fb * 2 * COLUMN_WIDTH;
        col    = fib / COLUMN_WIDTH;
        offset = fib % COLUMN_WIDTH;
        if (fib < and_bits)
            *region_out = is_attr_offset_nonxl(offset) ? "mc-attribute" : "and-array";
        else
            *region_out = "identity-plane";
    }
    *fb_out = fb; *col_out = col; *offset_out = offset;
}

void xc9500_diff(const JedFile *a, const JedFile *b) {
    if (a->count != b->count) {
        fprintf(stderr, "diff: fuse counts differ (%zu vs %zu)\n",
                a->count, b->count);
        return;
    }
    /* Prefer the JED's declared device name (exact package match), then
     * fuse-count detection -- same order as run_decode()/run_vhdl().
     * device_detect() alone cannot reach devices identified only by name
     * (e.g. XC95108), and annotate_fuse() does nothing on a NULL dev. */
    const device_t *dev = a->device[0] ? device_find(a->device) : NULL;
    if (!dev) dev = device_detect((int)a->count);

    size_t total_diff = 0;
    for (size_t i=0; i<a->count; i++) total_diff += (a->fuses[i] != b->fuses[i]);

    /* XL: mark every band position that is some macrocell's attribute
     * position (xl_attr_mc_off, fb 1..n_fbs, mc 1..18) once, up front. */
    unsigned char *xl_attr = NULL;
    if (dev && dev->is_xl) {
        size_t band = (size_t)dev->n_fbs * COLUMN_WIDTH;
        xl_attr = calloc(band, 1);
        if (!xl_attr) { fprintf(stderr, "diff: out of memory\n"); return; }
        for (int f = 1; f <= dev->n_fbs; f++)
            for (int m = 1; m <= MACROCELLS_PER_FB; m++) {
                int p = xl_attr_mc_off(dev->n_fbs, f, m);
                if (p >= 0 && (size_t)p < band) xl_attr[p] = 1;
            }
    }

    int cluster = 0;
    size_t i = 0;
    while (i < a->count) {
        if (a->fuses[i] == b->fuses[i]) { i++; continue; }
        size_t cstart = i;
        while (i < a->count && a->fuses[i] != b->fuses[i]) i++;
        cluster++;
        printf("cluster %d: bits [%zu..%zu], %zu differing\n",
               cluster, cstart, i-1, i-cstart);
        for (size_t k=cstart; k<i; k++) {
            int fb, col, offset; const char *region;
            annotate_fuse(dev, xl_attr, k, &fb, &col, &offset, &region);
            if (col >= 0)
                printf("  bit %6zu : FB%d %s  col=%-2d offset=%-3d : %d -> %d\n",
                       k, fb, region, col, offset, a->fuses[k], b->fuses[k]);
            else
                printf("  bit %6zu : FB%d %s  cfg-offset=%-4d : %d -> %d\n",
                       k, fb, region, offset, a->fuses[k], b->fuses[k]);
        }
    }
    if (!cluster) printf("0 fuses differ\n");
    else          printf("\n%zu of %zu fuses differ\n", total_diff, a->count);
    free(xl_attr);
}

/* ---- Structural dump ---- */

/* Every per-pair source name and macrocell attribute the dump prints comes
 * from decode.c: decode_chip() once, then pair_signal_name() (the function
 * decode prints its literals with) and the decoded mc_t fields. Nothing is
 * re-derived here, so the dump and decode always agree. */
static void dump_fb(const device_t *dev, const unsigned char *fuses, int fb,
                    const chip_t *chip) {
    int n_cols = dev->fc_pairs_per_fb * 2;
    int active = 0, idle = 0;
    for (int c = 0; c < n_cols; c++) {
        const unsigned char *col = device_col(dev, fuses, fb, c);
        if (col_is_idle(dev, col)) idle++;
        else active++;
    }
    printf("\n=== FB%d ===\n", fb);
    printf("  AND-array: %d columns, %d active, %d idle\n", n_cols, active, idle);
    printf("  Layout: %s\n", dev->is_xl ? "XC9500XL (band-major, 54 IMUX inputs)" :
                                           "XC9500 (FB-major, full crossbar)");

    /* Print active columns */
    for (int c = 0; c < n_cols; c++) {
        const unsigned char *col = device_col(dev, fuses, fb, c);
        if (col_is_idle(dev, col)) continue;
        int pair = c / 2;
        const char *side = (c%2==0) ? "COMP" : "TRUE";
        if (!dev->is_xl && pair < dev->fc_pairs_per_fb) {
            char nm[SIGNAME_MAX];
            pair_signal_name(dev, fuses, fb, pair, c % 2 == 0, nm, sizeof nm);
            printf("  col %3d (pair%2d %s = %s): active\n", c, pair, side, nm);
        } else {
            printf("  col %3d (INPUT%2d %s): active\n", c, pair+1, side);
        }
    }

    /* MC attribute snapshot: decode's own fields, for the macrocells decode
     * itself prints. The liveness test must stay identical to decode_print's
     * (including gsr_reset / gsr_set: a GSR-only register is listed). */
    printf("\n  MC attributes:\n");
    printf("  %-5s %-6s %-8s %-5s %-3s %-5s %-5s %-10s %-4s\n",
           "MC","pin","clk","fall","ff","rst","pst","oe","init");
    for (int mc = 1; mc <= MACROCELLS_PER_FB; mc++) {
        const mc_t *m = &chip->mc[fb-1][mc-1];
        if (!m->is_real_output && m->n_data_pts == 0 && !m->has_clk_pt &&
            !m->has_reset && !m->has_preset && !m->has_ce &&
            !m->has_oe_pt && !m->gsr_reset && !m->gsr_set) continue;
        const char *clk_str =
            (m->clk_mode==CLK_MODE_GCK1) ? "GCK1"  :
            (m->clk_mode==CLK_MODE_GCK2) ? "GCK2"  :
            (m->clk_mode==CLK_MODE_GCK3) ? "GCK3"  :
            (m->clk_mode==CLK_MODE_COMB) ? "COMB"  :
            (m->clk_mode==CLK_MODE_PT)   ? "PT-CLK": "?";
        const char *oe_str =
            m->oe_mode==3 && !m->has_oe_pt ? "GOE2"      :
            m->has_oe_pt  ? "PT-OE"     :
            m->oe_mode==1 ? "always-on" :
            m->oe_mode==2 ? "GOE1"      :
            m->oe_mode==4 ? "GOE3"      :
            m->oe_mode==5 ? "GOE4"      : "tristate";
        printf("  MC%-2d pin%-3d %-8s %-5s %-3s %-5s %-5s %-10s %-4s\n",
               mc, m->pin, clk_str,
               m->clk_falling ? "FALL" : "RISE",
               m->is_toggle ? "T" : m->is_combi ? "C" : "D",
               m->has_reset ? "yes" : "no", m->has_preset ? "yes" : "no",
               oe_str, m->init_set ? "SET" : "RST");
    }
}

void xc9500_dump(const JedFile *jf, const device_t *dev) {
    printf("device: %s\n", jf->device[0] ? jf->device : "(from fuse count)");
    if (dev) {
        printf("matched: %s (%s, %d FBs, QF=%d)\n",
               dev->id, dev->family, dev->n_fbs,
               dev->n_fbs * dev->jed_fuses_per_fb);
        if (dev->is_xl && !dev->imux_table)
            printf("NOTE: XC9500XL IMUX table not yet verified -- "
                   "signal names will appear as INPUT-N\n");
    } else {
        printf("unknown device\n"); return;
    }
    /* chip_t is several MB: heap, not stack (the Windows build's stack is
     * sized for run_decode's own chip_t, not for a second one). */
    chip_t *chip = malloc(sizeof *chip);
    if (!chip) { fprintf(stderr, "dump: out of memory\n"); return; }
    decode_chip(jf, dev, chip);
    for (int fb = 1; fb <= dev->n_fbs; fb++)
        dump_fb(dev, jf->fuses, fb, chip);
    free(chip);
}
