#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "jed.h"
#include "device.h"
#include "xc9500.h"
#include "decode.h"
#include "ucf.h"
#include "target.h"
#include "vendor_cells.h"

/* Banner on every run, on stderr so it never mixes into a redirected
 * .vhd/.ucf or a piped decode. */
static void xc9500_banner(void) {
    fprintf(stderr,
        "xc9500 decoder tool by Darksoft %s. Use at your own risk.\n",
        XC9500TOOL_VERSION);
}

static void usage(const char *prog) {
    fprintf(stderr,
        "xc9500dump " XC9500TOOL_VERSION
        " -- XC9500 family .jed decompiler by Darksoft\n"
        "\n"
        "Commands:\n"
        "  %s dump   <file.jed>                   structural report\n"
        "  %s diff   <a.jed> <b.jed>              annotated bit-level diff\n"
        "  %s decode --viewlist                   list devices + packages\n"
        "  %s decode --device D --package P <f.jed>   PT + attribute decode\n"
        "  %s vhdl   --device D --package P <f.jed> [out.vhd] [out.ucf] [out.opt]\n"
        "  %s verify <file.jed> <file.ucf>        cross-reference UCF vs JED\n"
        "  %s coverage <file.jed>                 per-geometry fuse coverage table\n"
        "  %s devices                             list known device tables\n"
        "\n"
        "Device selection: the JED's own \"N DEVICE\" name is used first, the fuse count\n"
        "only when the JED names no device. Every command that reads a .jed REFUSES a\n"
        "JED that names a device this tool does not support.\nSource tables per device, measured against the vendor's .chp: see `devices`.\n"
        "XL imux tables present for XC9536XL, XC9572XL, XC95144XL\n"
        "and XC95288XL.\n",
        prog, prog, prog, prog, prog, prog, prog, prog);
}

/* forward declarations */
int run_vhdl(const char *jed, const char *vhd, const char *ucf_path,
             const char *opt_path, const char *want_dev, const char *want_pkg);

static int run_decode(const char *jed_path, const char *want_dev, const char *want_pkg) {
    JedFile jf;
    if (jed_load(jed_path, &jf) != 0) return 2;
    /* An explicit --device/--package wins; target_resolve() prints what the
     * JED itself names beside what was asked for. */
    const device_t *dev = NULL;
    if (want_dev || want_pkg) {
        int conflict = 0;
        dev = target_resolve(&jf, want_dev, want_pkg, &conflict);
        if (!dev) { jed_free(&jf); return 2; }
    }
    if (!dev && jf.device[0]) dev = device_find(jf.device);
    if (!dev && !want_dev && device_named_but_unknown(jf.device)) {
        /* Named but unsupported: refuse rather than let fuse-count matching
         * pick a different device of the same size. */
        fprintf(stderr, "unsupported device \"%s\" (named in the JED header); "
                        "refusing to guess from fuse count\n", jf.device);
        return 2;
    }
    if (!dev)         dev = device_detect((int)jf.count);
    if (dev && !device_fits_jed(dev, jf.count)) {
        fprintf(stderr,
            "run_decode: %s expects %lu fuses, this JED has %lu.\n"
            "  Refusing -- decoding one device's fuse map with another's\n"
            "  geometry reads past the end of the data.\n",
            dev->id,
            (unsigned long)dev->n_fbs * (unsigned long)dev->jed_fuses_per_fb,
            (unsigned long)jf.count);
        jed_free(&jf); return 2;
    }
    if (!dev) { jed_free(&jf); return 2; }
    /* The "device:" line is printed even when the command refuses below:
     * other tools read the device family from it, and it carries no pin
     * numbers, only the package-independent family. */
    printf("device: %s  (%s)\n", jf.device[0] ? jf.device : "(from fuse count)", dev->id);
    /* Refuse when the JED never names its package and none was given: the
     * generic row's pin table is a default that may belong to a different
     * package, so every pin number would be a guess. The refusal lists every
     * candidate package (device_generic_candidates(), shared with vhdl). */
    if (device_is_generic(dev) && !want_pkg) {
        char cand[256];
        int n = device_generic_candidates(dev, cand, sizeof cand);
        fprintf(stderr,
            "decode: REFUSING -- this JED never names its package.\n"
            "        Device resolved to %s, whose pin table is a DEFAULT and\n"
            "        may belong to a different package -- every pin NUMBER\n"
            "        below would be a guess (macrocell names, FBn_MCm, are\n"
            "        unaffected by package, but this command no longer\n"
            "        prints a guessed pin table to get them).\n"
            "        Name the package on the command line:\n"
            "            xc9500dump decode --device %.*s --package <PKG> %s\n"
            "        Candidate package%s for this device: %s\n"
            "        If you do not know it, READ THE CHIP -- the package\n"
            "        code is in the part number printed on it (e.g.\n"
            "        XC95108 PQ100AEM -> PQ100).\n",
            dev->id,
            (int)(strchr(dev->id, '-') - dev->id), dev->id,
            jed_path,
            n == 1 ? "" : "s", n ? cand : "(none known for this family)");
        jed_free(&jf);
        return 2;
    }
    /* XL global bits. Eight bits named by the vendor's device file; their
     * address is the standard XL interleave:
     *
     *     addr = col * n_fbs * 108 + row * (8 * n_fbs) + 6
     *
     *   CKON0..CKON2 -- a signal is mapped onto global clock net GCK1..3
     *   OEON0..3     -- the same for GTS1..4
     *   FRSI (GSR)   -- printed only; nothing is inferred from it (a design
     *                   can map GSR with the bit clear). */
    if (dev->is_xl && dev->n_fbs > 0) {
        static const char *gn[8] = { "FRSI", "CKON0", "CKON1", "CKON2",
                                     "OEON0", "OEON1", "OEON2", "OEON3" };
        int r; int first = 1;
        for (r = 0; r < 8; r++) {
            size_t a2 = (size_t)2 * dev->n_fbs * 108
                      + (size_t)r * (8 * dev->n_fbs) + 6;
            if (a2 >= (size_t)jf.count) continue;
            if (first) { printf("globals:"); first = 0; }
            printf(" %s=%d", gn[r], jf.fuses[a2] ? 1 : 0);
        }
        if (!first) printf("\n");
    }

    /* The user signature. `hprep6 -n <sig>` writes 32 bits (four
     * characters) into columns 6 and 7; the vendor names them SI0..SI31.
     *
     *   char 0,1 -> column 6      char 2,3 -> column 7
     *   char 0,2 -> row base  6   char 1,3 -> row base 38
     *   ASCII bit -> row offset: b6=+0 b5=+9 b4=+8 b3=+17 b2=+16 b1=+25 b0=+24
     *   fuses are inverted: a 0 fuse means a 1 bit.
     *
     * The columns are the same on both families; the layout and polarity are
     * not. Non-XL holds the bits at mc_off(mc) on page 1 (active at raw 0);
     * XL holds them at xl_attr_mc_off(n_fbs,1,mc), spread over several pages
     * as n_fbs grows (active at raw 1). decode.c's chip_signature() reads
     * them through each family's own addressing functions.
     *
     * Reported, never used for naming. */
    {
        /* Always printed, from decode.c's chip_signature_text(): the 32 bits in
         * hex, with the ASCII form beside them only when every byte is
         * 0x20..0x7E. The same text goes into the VHDL and UCF headers. */
        char sigtxt[96];
        chip_signature_text(&jf, dev, sigtxt, sizeof sigtxt);
        printf("signature: %s\n", sigtxt);
    }

    /* Warn when a programmed fuse has no known meaning: non-XL FBINV,
     * column 46, one fuse per macrocell. It is named by the vendor and read
     * by this tool, but what it does is not known. The warning names the
     * exact macrocell so the user can judge the output. */
    {
        /* Only FBINV warns. XL columns 31/38 do not warn, and the user signature
         * (columns 6/7) is decoded and reported on its own line. */
        static const int un_nonxl[] = { 46 };
        const int *uncols = un_nonxl;
        int n_uncols = dev->is_xl ? 0 : 1;
        int idle = 1;                    /* non-XL unprogrammed level */
        int fb, mc, q, hits = 0;
        for (q = 0; q < n_uncols; q++) {
            for (fb = 1; fb <= dev->n_fbs; fb++) {
                for (mc = 1; mc <= 18; mc++) {
                    const unsigned char *c =
                        device_col(dev, jf.fuses, fb, uncols[q]);
                    int v;
                    if (!c) continue;
                    v = c[mc_off(mc)];
                    if (v == idle) continue;
                    if (!hits) {
                        printf("WARNING: this JED programs fuses whose meaning "
                               "is NOT known.\n");
                        printf("         The decode below is still valid for "
                               "everything else. These fuses\n");
                        printf("         are read by this tool; "
                               "what they do is unproven.\n");
                    }
                    hits++;
                    printf("  unknown fuse ACTIVE: FB%d_MC%-2d col %-3d %s\n",
                           fb, mc, uncols[q],
                           "FBINV (named, meaning unproven)");
                }
            }
        }
        if (hits)
            printf("         %d unknown fuse%s active in total.\n\n",
                   hits, hits == 1 ? "" : "s");
    }
    chip_t chip;
    decode_chip(&jf, dev, &chip);
    decode_print(&chip);
    /* Vendor control cells (vendor_cells.c), listed after the decode. */
    vendor_cells_report(dev, jf.fuses, (unsigned long)jf.count, stdout, 0);
    jed_free(&jf);
    return 0;
}


/* run_coverage -- reader coverage per device geometry.
 *
 * A fuse counts as read when some read site in the decoder consumes its
 * value in a way that can change output. This is coverage of the reader
 * (what this tool consumes), not of the device (what each fuse does).
 *
 * reader_cov[] is generated data, not a formula:
 *   4th column   = read by some read site other than the XL col_is_idle()
 *                  shortcut
 *   never_read   = read by no read site
 *   neutral_only = read only by XL col_is_idle(), whose value cannot reach
 *                  output
 * Do not edit it by hand.
 */
static const struct reader_cov {
    int  is_xl, n_fbs;
    long qf, consumed, never_read, neutral_only;
    long qf_lo, mod_lo;          /* non-XL: cols 0-71 (the rest: cols 72+) */
} reader_cov[] = {
    /* is_xl n_fbs      qf      read never_read neutral_only   qf_lo  rd_lo */
    { 1,  2,   23328,   23328,     0,     0,       0,       0 },  /* XL2 */
    { 1,  4,   46656,   46656,     0,     0,       0,       0 },  /* XL4 */
    { 1,  8,   93312,   93312,     0,     0,       0,       0 },  /* XL8 */
    { 1, 16,  186624,  186624,     0,     0,       0,       0 },  /* XL16 */
    { 0,  2,   18144,   18144,     0,     0,   15552,   15552 },  /* nonXL2 */
    { 0,  4,   41472,   41472,     0,     0,   31104,   31104 },  /* nonXL4 */
    { 0,  6,   69984,   69984,     0,     0,   46656,   46656 },  /* nonXL6 */
    { 0,  8,  103680,  103680,     0,     0,   62208,   62208 },  /* nonXL8 */
    { 0, 12,  186624,  186624,     0,     0,   93312,   93312 },  /* nonXL12 */
    { 0, 16,  290304,  290304,     0,     0,  124416,  124416 },  /* nonXL16 */
};

/* Returns a status: 0 done, 2 refused or failed. */
static int run_coverage(const char *jed_path) {
    JedFile jf;
    if (jed_load(jed_path, &jf) != 0) return 2;
    const device_t *dev = NULL;
    if (jf.device[0]) dev = device_find(jf.device);
    if (!dev && device_named_but_unknown(jf.device)) {
        /* Named but unsupported: refuse rather than let fuse-count matching
         * pick a different device of the same size. */
        fprintf(stderr, "unsupported device \"%s\" (named in the JED header); "
                        "refusing to guess from fuse count\n", jf.device);
        return 2;
    }
    if (!dev)         dev = device_detect((int)jf.count);
    if (!dev) { fprintf(stderr, "coverage: unknown device\n"); jed_free(&jf); return 2; }

    printf("device            : %s  (%s)\n",
           jf.device[0] ? jf.device : "(from fuse count)", dev->id);
    printf("total fuses       : %d\n", (int)jf.count);

    const struct reader_cov *rc = NULL;
    size_t k;
    for (k = 0; k < sizeof reader_cov / sizeof reader_cov[0]; k++)
        if (reader_cov[k].is_xl == dev->is_xl && reader_cov[k].n_fbs == dev->n_fbs)
            rc = &reader_cov[k];
    long qf = (long)dev->n_fbs * dev->jed_fuses_per_fb;
    if (!rc || rc->qf != qf) {
        printf("\nNo measured reader coverage exists for this device geometry;\n");
        printf("nothing is printed rather than an unmeasured figure.\n");
        jed_free(&jf);
        return 0;
    }
    if ((long)jf.count != qf) {
        /* Same refusal as decode: this JED is not a complete fuse map of the
         * device it names. */
        fprintf(stderr, "coverage: %s expects %ld fuses, this JED has %lu -- refusing.\n",
                dev->id, qf, (unsigned long)jf.count);
        jed_free(&jf);
        return 2;
    }
    int fb_cols = dev->jed_fuses_per_fb / COLUMN_WIDTH;
    printf("function blocks   : %d      fuses per FB : %d  (%d cols x %d rows)\n",
           dev->n_fbs, dev->jed_fuses_per_fb, fb_cols, COLUMN_WIDTH);
    printf("\n");
    printf("READER coverage, MEASURED -- NOT device coverage.\n");
    printf("A fuse is READ when a read site in this tool's own source consumes\n");
    printf("its value in a way that can change the decoder's output (fuse-read log\n");
    printf("of this version's source, cross-checked by a flip test).\n");
    printf("READ says what THIS TOOL reads. It does not say what the fuse does\n");
    printf("in the device, or that its meaning is known.\n");
    printf("Figures are per device geometry, all packages pooled (%s%d).\n",
           dev->is_xl ? "XL" : "nonXL", dev->n_fbs);
    printf("\n");
    printf("                         fuses       READ              NOT READ\n");
    if (!dev->is_xl) {
        printf("  cols 0-71       %10ld %10ld %20ld\n",
               rc->qf_lo, rc->mod_lo, rc->qf_lo - rc->mod_lo);
        printf("  cols 72-%-3d     %10ld %10ld %20ld\n", fb_cols - 1,
               rc->qf - rc->qf_lo, rc->consumed - rc->mod_lo,
               (rc->qf - rc->qf_lo) - (rc->consumed - rc->mod_lo));
    }
    printf("  all             %10ld %10ld (%.1f%%) %11ld (%.1f%%)\n",
           rc->qf, rc->consumed, 100.0 * rc->consumed / (double)rc->qf,
           rc->qf - rc->consumed, 100.0 * (rc->qf - rc->consumed) / (double)rc->qf);
    printf("\n");
    printf("  NOT READ:     %6ld never read by any read site\n", rc->never_read);
    printf("                %6ld read only by the idle-column shortcut (value cannot reach output)\n",
           rc->neutral_only);
    jed_free(&jf);
    return 0;
}

/* Returns a status: 0 done, 2 refused or failed. */
static int run_verify(const char *jed_path, const char *ucf_path) {
    JedFile jf;
    if (jed_load(jed_path, &jf) != 0) return 2;
    /* The JED's declared device name first, then fuse-count detection. */
    const device_t *dev = jf.device[0] ? device_find(jf.device) : NULL;
    if (!dev && device_named_but_unknown(jf.device)) {
        fprintf(stderr, "unsupported device \"%s\" (named in the JED header); "
                        "refusing to guess from fuse count\n", jf.device);
        return 2;
    }
    if (!dev) dev = device_detect((int)jf.count);
    UcfFile uf;
    if (ucf_load(ucf_path, &uf) != 0) { jed_free(&jf); return 2; }

    printf("device: %s\n", dev ? dev->id : "unknown");
    printf("UCF signals: %zu\n\n", uf.count);

    chip_t chip;
    if (dev) decode_chip(&jf, dev, &chip);

    printf("%-20s %-5s %-4s %-4s %-8s %-8s %-8s %-6s\n",
           "signal", "pin", "FB", "MC", "reset?", "preset?", "oe-mode", "clk");
    for (size_t i = 0; i < uf.count; i++) {
        int pin = uf.entries[i].pin;
        int fb_out = 0, mc_out = 0;
        /* Find which MC this pin maps to */
        if (dev && dev->mc_pin) {
            for (int fb = 1; fb <= dev->n_fbs && !fb_out; fb++)
                for (int mc = 1; mc <= MACROCELLS_PER_FB && !fb_out; mc++)
                    if (dev->mc_pin[(fb-1)*18+mc-1] == pin) { fb_out=fb; mc_out=mc; }
        }
        if (!fb_out) {
            printf("%-20s %-5d %-4s %-4s %-8s %-8s %-8s %-6s\n",
                   uf.entries[i].name, pin, "-","-","-","-","-","-");
            continue;
        }
        const mc_t *m = &chip.mc[fb_out-1][mc_out-1];
        static const char *oe_strs[] = {"tri","on","GOE1","GOE2/PT"};
        static const char *clk_strs[] = {"-","GCK1","GCK2","GCK3","PT-CLK","COMB","-","-"};
        printf("%-20s %-5d %-4d %-4d %-8s %-8s %-8s %-6s\n",
               uf.entries[i].name, pin, fb_out, mc_out,
               m->has_reset  ? "yes" : "no",
               m->has_preset ? "yes" : "no",
               oe_strs[m->oe_mode & 3],
               clk_strs[m->clk_mode & 7]);
    }
    ucf_free(&uf);
    jed_free(&jf);
    return 0;
}

int main(int argc, char **argv) {
    /* Banner on stderr so it never mixes into a redirected .vhd/.ucf or a
     * piped decode. --nobanner suppresses it. */
    {   int bi, quiet = 0;
        for (bi = 1; bi < argc; bi++)
            if (strcmp(argv[bi], "--nobanner") == 0) { quiet = 1; break; }
        if (!quiet) xc9500_banner();
    }
    if (argc < 2) { usage(argv[0]); return 1; }

    if (strcmp(argv[1], "devices") == 0) {
        device_list();
        return 0;
    }
    /* --device / --package are pulled out of argv before the per-command
     * parsing so every command sees the same flags, and --viewlist works
     * without a file argument. */
    const char *opt_dev = NULL, *opt_pkg = NULL;
    int opt_viewlist = 0;
    {   int r = 1;
        for (int i = 1; i < argc; i++) {
            /* Option names and values are case-insensitive. */
            if (strcasecmp(argv[i], "--viewlist") == 0) { opt_viewlist = 1; continue; }
            if (strcasecmp(argv[i], "--nobanner") == 0) continue;   /* handled in main */
            /* -noportfix: keep an unresolved FastCONNECT literal as an
             * undriven stub signal instead of giving it a real input pin.
             * The pin costs a package pin but stops the fitter reducing the
             * unknown to a constant and rebuilding the macrocell around it. */
            if (strcasecmp(argv[i], "-noportfix") == 0 ||
                strcasecmp(argv[i], "--noportfix") == 0) { vhdl_no_port_fix = 1; continue; }
            if (strcasecmp(argv[i], "--device") == 0 && i + 1 < argc) { opt_dev = argv[++i]; continue; }
            if (strcasecmp(argv[i], "--package") == 0 && i + 1 < argc) { opt_pkg = argv[++i]; continue; }
            if (strncasecmp(argv[i], "--device=", 9) == 0) { opt_dev = argv[i] + 9; continue; }
            if (strncasecmp(argv[i], "--package=", 10) == 0) { opt_pkg = argv[i] + 10; continue; }
            /* An unrecognised "--" argument is an error, never a filename: a
             * misspelt option would otherwise shift every positional
             * argument by one. */
            if (strncmp(argv[i], "--", 2) == 0) {
                fprintf(stderr,
                    "xc9500dump: unrecognised option \"%s\"\n"
                    "  Options and their values are case-insensitive:\n"
                    "      --device <DEV>    or  --device=<DEV>\n"
                    "      --package <PKG>   or  --package=<PKG>\n"
                    "      --viewlist        --noportfix     --nobanner\n"
                    "  Run with no arguments for the full usage text.\n",
                    argv[i]);
                return 2;
            }
            argv[r++] = argv[i];
        }
        argc = r;
    }
    if (opt_viewlist) { target_viewlist(); return 0; }
    /* Checked again here: the match above runs before --nobanner is
     * stripped. */
    if (argc >= 2 && strcmp(argv[1], "devices") == 0) { device_list(); return 0; }

    if (argc < 3) { usage(argv[0]); return 1; }

    if (strcmp(argv[1], "dump") == 0) {
        JedFile jf;
        if (jed_load(argv[2], &jf) != 0) return 1;
        /* The JED's "N DEVICE" string wins, fuse count is the fallback -- the
         * same order run_decode()/run_vhdl() use. */
        const device_t *dev = jf.device[0] ? device_find(jf.device) : NULL;
        /* Refuse a JED naming a device this tool does not support. */
        if (!dev && device_named_but_unknown(jf.device)) {
            fprintf(stderr, "unsupported device \"%s\" (named in the JED header); "
                            "refusing to guess from fuse count\n", jf.device);
            jed_free(&jf);
            return 2;
        }
        if (!dev) dev = device_detect((int)jf.count);
        xc9500_dump(&jf, dev);
        jed_free(&jf);
        return 0;
    }
    if (strcmp(argv[1], "diff") == 0) {
        if (argc < 4) { usage(argv[0]); return 1; }
        JedFile a, b;
        if (jed_load(argv[2], &a) != 0) return 1;
        if (jed_load(argv[3], &b) != 0) { jed_free(&a); return 1; }
        /* diff labels every address with a device's geometry: refuse if
         * either JED names a device this tool does not support. */
        {   const JedFile *u = device_named_but_unknown(a.device) ? &a :
                               device_named_but_unknown(b.device) ? &b : NULL;
            if (u) {
                fprintf(stderr, "unsupported device \"%s\" (named in the JED header); "
                                "refusing to guess from fuse count\n", u->device);
                jed_free(&a); jed_free(&b);
                return 2;
            }
        }
        xc9500_diff(&a, &b);
        jed_free(&a); jed_free(&b);
        return 0;
    }
    if (strcmp(argv[1], "decode") == 0) {
        return run_decode(argv[2], opt_dev, opt_pkg);
    }
    if (strcmp(argv[1], "vhdl") == 0) {
        const char *vhd = (argc>=4) ? argv[3] : NULL;
        const char *ucf = (argc>=5) ? argv[4] : NULL;
        /* Optional build-options file: the fields no UCF line can express --
         * the user signature, the programmable ground and the XL keeper --
         * for the cpldfit and hprep6 command lines. */
        const char *opt = (argc>=6) ? argv[5] : NULL;
        /* Propagate the status: a refusal must not look like success to a
         * build script. */
        return run_vhdl(argv[2], vhd, ucf, opt, opt_dev, opt_pkg);
    }
    if (strcmp(argv[1], "coverage") == 0) {
        if (argc < 3) { fprintf(stderr, "usage: %s coverage <file.jed>\n", argv[0]); return 1; }
        return run_coverage(argv[2]);
    }
    if (strcmp(argv[1], "verify") == 0) {
        if (argc < 4) { usage(argv[0]); return 1; }
        return run_verify(argv[2], argv[3]);
    }

    usage(argv[0]); return 1;
}
