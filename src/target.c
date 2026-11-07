/* target.c -- explicit device/package selection.
 *
 * Every LOC in a UCF names a physical pin, so the package must be right. A
 * JED often does not name its package; the device and package are given on
 * the command line and the tool prints what the file says beside what was
 * asked for, so any disagreement is visible before anything is written.
 *
 *     xc9500dump decode --viewlist
 *     xc9500dump decode --device XC95144 --package TQ100 board.jed
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "device.h"
#include "jed.h"
#include "target.h"
#include <strings.h>   /* strcasecmp */

/* Split "XC95144-TQ100" into its two halves. */
static void split_id(const device_t *d, char *dev, size_t dn,
                     char *pkg, size_t pn) {
    const char *dash = strchr(d->id, '-');
    size_t n = dash ? (size_t)(dash - d->id) : strlen(d->id);
    if (n >= dn) n = dn - 1;
    memcpy(dev, d->id, n);
    dev[n] = 0;
    if (dash && dash[1]) snprintf(pkg, pn, "%s", dash + 1);
    else                 pkg[0] = 0;
}

void target_viewlist(void) {
    char dev[64], pkg[64];
    char seen[64][64];
    int n_seen = 0;
    printf("Supported devices and packages\n");
    printf("------------------------------\n");
    /* Group by device. The device table is ordered for lookup, not for
     * display, so a device's rows are not contiguous; collect the distinct
     * devices first. */
    for (int i = 0; ; i++) {
        const device_t *d = device_nth(i);
        if (!d) break;
        split_id(d, dev, sizeof dev, pkg, sizeof pkg);
        if (!pkg[0]) continue;            /* generic fallback row: not a target */
        int have = 0;
        for (int k = 0; k < n_seen; k++)
            if (strcmp(seen[k], dev) == 0) { have = 1; break; }
        if (!have && n_seen < 64) snprintf(seen[n_seen++], 64, "%s", dev);
    }
    for (int k = 0; k < n_seen; k++) {
        printf("  %-10s ", seen[k]);
        for (int i = 0; ; i++) {
            const device_t *d = device_nth(i);
            if (!d) break;
            split_id(d, dev, sizeof dev, pkg, sizeof pkg);
            if (pkg[0] && strcmp(dev, seen[k]) == 0) printf("%s ", pkg);
        }
        printf("\n");
    }
    printf("\n");
    printf("Give both explicitly, e.g.:\n");
    printf("  xc9500dump decode --device XC95144 --package TQ100 board.jed\n");
    printf("  xc9500dump vhdl   --device XC95108 --package PC84  board.jed o.vhd o.ucf\n\n");
    printf("The tool prints what it finds in the JED beside what you asked for,\n");
    printf("so a disagreement is visible before anything is written.\n");
}

/* What does the JED itself claim? Returns a device string (possibly empty). */
static void jed_claims(const JedFile *jf, char *dev, size_t dn,
                       char *pkg, size_t pn) {
    dev[0] = pkg[0] = 0;
    if (!jf->device[0]) return;
    const char *dash = strchr(jf->device, '-');
    size_t n = dash ? (size_t)(dash - jf->device) : strlen(jf->device);
    if (n >= dn) n = dn - 1;
    memcpy(dev, jf->device, n);
    dev[n] = 0;
    /* An ISE device string is DEVICE-SPEED-PACKAGE, e.g. "XC95108-20-PC84":
     * the package is the last dash-separated field. */
    const char *last = strrchr(jf->device, '-');
    if (last && last[1]) {
        int real = 0;
        for (const char *p = last + 1; *p; p++)
            if (toupper((unsigned char)*p) != 'X') { real = 1; break; }
        if (real) snprintf(pkg, pn, "%s", last + 1);
    }
}

const device_t *target_resolve(const JedFile *jf,
                               const char *want_dev, const char *want_pkg,
                               int *out_conflict) {
    char jdev[64], jpkg[64];
    jed_claims(jf, jdev, sizeof jdev, jpkg, sizeof jpkg);
    if (out_conflict) *out_conflict = 0;

    printf("device requested: %-12s found in jed: %s\n",
           want_dev ? want_dev : "(none)",
           jdev[0] ? jdev : "not found");
    if (jpkg[0])
        printf("package requested: %-10s found in jed: %s\n",
               want_pkg ? want_pkg : "(none)", jpkg);
    else if (jf->pin_count)
        printf("package requested: %-10s found in jed: not found "
               "(QP%d: %d pins)\n",
               want_pkg ? want_pkg : "(none)", jf->pin_count, jf->pin_count);
    else
        printf("package requested: %-10s found in jed: not found\n",
               want_pkg ? want_pkg : "(none)");

    if (jdev[0] && want_dev && strcasecmp(jdev, want_dev) != 0) {
        printf("  *** MISMATCH: the JED names %s, you asked for %s\n",
               jdev, want_dev);
        if (out_conflict) *out_conflict = 1;
    }
    if (jpkg[0] && want_pkg && strcasecmp(jpkg, want_pkg) != 0) {
        printf("  *** MISMATCH: the JED names package %s, you asked for %s\n",
               jpkg, want_pkg);
        if (out_conflict) *out_conflict = 1;
    }

    /* A field not given on the command line is taken from the JED. The
     * device comes from "N DEVICE", the same string device_find() uses
     * without flags; the package is often absent from the file (QP0, or
     * "XXXXX" in the header) and must then be given. */
    if (!want_dev && jdev[0]) want_dev = jdev;
    if (!want_pkg && jpkg[0]) want_pkg = jpkg;

    if (!want_dev || !want_pkg) return NULL;

    char id[128];
    snprintf(id, sizeof id, "%s-%s", want_dev, want_pkg);
    const device_t *d = device_find(id);
    if (d) {
        size_t n = strlen(d->id);
        if (n && d->id[n-1] == '-') d = NULL;   /* generic row is not a target */
    }
    if (!d) {
        fprintf(stderr,
            "error: %s-%s is not a supported target.\n"
            "       Run `xc9500dump decode --viewlist` for the full list.\n",
            want_dev, want_pkg);
        return NULL;
    }

    /* The fuse count is an independent check on the device. */
    {   int want_fuses = d->jed_fuses_per_fb * d->n_fbs;
        if (jf->count && want_fuses && (int)jf->count != want_fuses) {
            printf("  *** MISMATCH: %s has %d fuses, this JED has %d\n",
                   d->id, want_fuses, (int)jf->count);
            if (out_conflict) *out_conflict = 1;
        }
    }
    /* So is the pin count, when the file carries one. */
    if (jf->pin_count && want_pkg) {
        int want_pins = 0;
        for (const char *p = want_pkg; *p; p++)
            if (isdigit((unsigned char)*p)) want_pins = want_pins * 10 + (*p - '0');
        if (want_pins && want_pins != jf->pin_count) {
            printf("  *** MISMATCH: package %s has %d pins, the JED says QP%d\n",
                   want_pkg, want_pins, jf->pin_count);
            if (out_conflict) *out_conflict = 1;
        }
    }
    printf("target: %s\n", d->id);
    return d;
}
