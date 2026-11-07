/* vendor_cells.c -- vendor control cells.
 *
 * These are the fuses decode.c does not decode into logic: OR-array cells
 * written by the vendor's bitmap writer. A per-macrocell OR bitmap starts
 * all-zero and only named cells are ever set.
 *
 * Each cell belongs to one group (vendor_cells_table.h):
 *   DEFAULT-ONLY -- the writer never sets it; its vendor value is XL 0 /
 *     non-XL 1 (the family's JED polarity). A different value cannot come
 *     from the vendor fitter: the file was edited, mis-read, or written by
 *     something else. The security cells are in this group:
 *       XL: the 30 read-protect cells are CSEC, the 30 write-protect cells
 *         DPROT (set by the programmer's read/write protect options, not by
 *         the fitter);
 *       XC95144: JEDEC 20334 (the FB2 CSEC position of the legend) is its own
 *         group, VC_NOWRITE -- no vendor flow writes it; read protect on this
 *         part is JEDEC 21052, an identity-plane cell outside this table;
 *       MSEC: no vendor flow writes it.
 *   ALWAYS-SET -- RPUX (XL 1 / non-XL 0).
 *   VARYING -- FBPWR (set = the FB has a used macrocell), CON (set = the FB
 *     exports a product term), the non-XL FB1 device globals (MCLK/MOE/
 *     OE<k>I/FRS0I), XC95288 CIN: printed as values.
 * A WARNING never changes the decode or the emission. */
#include <string.h>
#include "vendor_cells.h"

typedef struct { unsigned addr; unsigned char grp, fb; } vc_entry_t;
typedef struct { unsigned addr; const char *name; } vc_name_t;
typedef struct {
    const char *den; int is_xl, n_fbs;
    const vc_entry_t *e; int n;
    const vc_name_t *names; int nn;
} vc_density_t;

#include "vendor_cells_table.h"

static const char *const vc_label[VC_NGROUPS] = {
    "unnamed OR cells", "SI positions on FB2+", "global-block legend positions outside FB1's set", "CIN cells (not XC95288)",
    "CSEC (iMPACT -r writes these)", "DPROT (iMPACT -w writes these)", "MSEC (no vendor flow writes these)",
    "XC95144 FB2 CSEC legend position (no vendor flow writes it; iMPACT -r writes JEDEC 21052)", "RPUX", "FBPWR", "CON",
    "FB1 device globals", "CIN (XC95288)"
};

static int is_default_only(int g) { return g <= VC_NOWRITE; }

int vendor_cells_report(const device_t *dev, const unsigned char *fuses, unsigned long count, FILE *out, int as_vhdl_comment) {
    const vc_density_t *D = NULL;
    size_t i;
    for (i = 0; i < sizeof vc_densities / sizeof vc_densities[0]; i++)
        if (vc_densities[i].is_xl == dev->is_xl && vc_densities[i].n_fbs == dev->n_fbs) { D = &vc_densities[i]; break; }
    if (!D) {
        if (!as_vhdl_comment) fprintf(out, "vendor control cells: no table for %s\n", dev->id);
        return 0;
    }
    const int xl = D->is_xl;
    const int dflt = xl ? 0 : 1;           /* DEFAULT-ONLY vendor value */
    const int setv = xl ? 1 : 0;           /* the JED value of a writer-SET cell (RPUX, FBPWR, CON, FB1 globals, CIN) */
    int bad[VC_NGROUPS], first_fb[VC_NGROUPS], ndef = 0, ndef_ok = 0;
    unsigned first_addr[VC_NGROUPS];
    int pwr[MAX_FBS], con[MAX_FBS], cin[MAX_FBS];
    memset(bad, 0, sizeof bad); memset(pwr, 0, sizeof pwr); memset(con, 0, sizeof con); memset(cin, 0, sizeof cin);
    for (int g = 0; g < VC_NGROUPS; g++) { first_fb[g] = -1; first_addr[g] = 0; }
    for (int k = 0; k < D->n; k++) {
        const vc_entry_t *e = &D->e[k];
        if (e->addr >= count) continue;
        const int v = fuses[e->addr] ? 1 : 0;
        int nonvendor = 0;
        if (is_default_only(e->grp)) { ndef++; if (v == dflt) ndef_ok++; else nonvendor = 1; }
        else if (e->grp == VC_RPUX) { if (v != setv) nonvendor = 1; }
        else if (e->grp == VC_FBPWR && e->fb < MAX_FBS) pwr[e->fb] = (v == setv);
        else if (e->grp == VC_CON && e->fb < MAX_FBS) con[e->fb] = (v == setv);
        else if (e->grp == VC_CIN_VARY && e->fb < MAX_FBS) cin[e->fb] += (v == setv);
        if (nonvendor) {
            if (!bad[e->grp]) { first_fb[e->grp] = e->fb; first_addr[e->grp] = e->addr; }
            bad[e->grp]++;
        }
    }
    int total_bad = 0;
    for (int g = 0; g < VC_NGROUPS; g++) total_bad += bad[g];
    if (!as_vhdl_comment) {
        fprintf(out, "vendor control cells (%s table, %d cells; set/clear as the vendor writer sets them):\n",
                D->den, D->n);
        fprintf(out, "  FBPWR (set = FB has a used macrocell):");
        for (int f = 0; f < D->n_fbs; f++) fprintf(out, " FB%d=%s", f + 1, pwr[f] ? "set" : "clear");
        fprintf(out, "\n  CON   (set = FB exports a product term):");
        for (int f = 0; f < D->n_fbs; f++) fprintf(out, " FB%d=%s", f + 1, con[f] ? "set" : "clear");
        fprintf(out, "\n");
        if (D->nn > 0) {
            fprintf(out, "  FB1 device globals (1 = set):");
            for (int k = 0; k < D->nn; k++)
                if (D->names[k].addr < count)
                    fprintf(out, " %s=%d", D->names[k].name, (fuses[D->names[k].addr] ? 1 : 0) == setv);
            fprintf(out, "\n");
        }
        int has_cin = 0;
        for (int k = 0; k < D->n; k++) if (D->e[k].grp == VC_CIN_VARY) { has_cin = 1; break; }
        if (has_cin) {
            fprintf(out, "  CIN (XC95288 input select), cells set per FB:");
            for (int f = 0; f < D->n_fbs; f++) fprintf(out, " FB%d=%d", f + 1, cin[f]);
            fprintf(out, "\n");
        }
        fprintf(out, "  default-only cells at the vendor default (%d): %d of %d\n", dflt, ndef_ok, ndef);
    }
    for (int g = 0; g < VC_NGROUPS; g++) {
        if (!bad[g]) continue;
        fprintf(out, "%sWARNING: %d %s hold a value cpldfit/hprep6 do not write (first: FB%d, JEDEC %u) -- a non-vendor file; "
                     "logic decode unchanged\n", as_vhdl_comment ? "-- " : "", bad[g], vc_label[g], first_fb[g] + 1, first_addr[g]);
    }
    return total_bad;
}

/* The legend cell for "<pfx><idx>": an exact label first ("MOE2"), else a
 * two-name label containing it ("MOE2/MOE4"). On the 2-GTS parts the table
 * has no plain MOE2 and index 2 is the "MOE2/MOE4" cell; on the 4-GTS parts
 * index 2 is the plain "MOE2" cell (oeb3 on XC95144/216/288) and index 4 the
 * "MOE2/MOE4" one (oeb2). Returns -1 when absent. */
static long vc_named_addr(const vc_density_t *D, const char *pfx, int idx) {
    char want[16]; snprintf(want, sizeof want, "%s%d", pfx, idx);
    for (int k = 0; k < D->nn; k++) if (strcmp(D->names[k].name, want) == 0) return (long)D->names[k].addr;
    size_t wl = strlen(want);
    for (int k = 0; k < D->nn; k++) {
        const char *n = D->names[k].name, *sl = strchr(n, '/');
        if (!sl) continue;
        if (((size_t)(sl - n) == wl && strncmp(n, want, wl) == 0) || strcmp(sl + 1, want) == 0) return (long)D->names[k].addr;
    }
    return -1;
}

int vendor_cells_global_src(const device_t *dev, const unsigned char *fuses, unsigned long count, int is_oe, int net, int n_gts,
                            int *inverted) {
    if (inverted) *inverted = 0;
    if (dev->is_xl) return net;
    const vc_density_t *D = NULL;
    for (size_t i = 0; i < sizeof vc_densities / sizeof vc_densities[0]; i++)
        if (vc_densities[i].is_xl == dev->is_xl && vc_densities[i].n_fbs == dev->n_fbs) { D = &vc_densities[i]; break; }
    const int N = is_oe ? n_gts : 3;
    if (!D || N < 1 || net < 1 || net > N) return net;
    const char *pfx = is_oe ? "MOE" : "MCLK";
    long dflt = vc_named_addr(D, pfx, net - 1), alt = vc_named_addr(D, pfx, N + net - 1);
    /* non-XL: a writer-set cell reads 0 */
    int dset = dflt >= 0 && (unsigned long)dflt < count && fuses[dflt] == 0;
    int aset = alt >= 0 && (unsigned long)alt < count && fuses[alt] == 0;
    if (inverted && is_oe) {
        char want[8]; snprintf(want, sizeof want, "OE%dI", net - 1);
        long ia = -1;
        for (int k = 0; k < D->nn; k++) if (strcmp(D->names[k].name, want) == 0) ia = (long)D->names[k].addr;
        *inverted = ia >= 0 && (unsigned long)ia < count && fuses[ia] == 0;
    }
    if (aset && !dset) return (net % N) + 1;
    return net;
}
