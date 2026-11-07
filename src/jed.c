#define _GNU_SOURCE
#include "jed.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Standard JEDEC ASCII format: fields are delimited by '*'. Whitespace and
 * newlines inside a field (including inside an L record's bit string) are
 * purely cosmetic once the field is parsed. Some files are wrapped in
 * STX (0x02) / ETX (0x03) + checksum bytes; we don't need to care, since we
 * only look for specific field prefixes and ignore everything else.
 *
 * MULTI-FORMAT SUPPORT:
 *   ISE standard  : STX + ETX, QF<n>, N DEVICE <name>
 *   Xeltec SuperPro: STX + ETX, QF<n> (leading-zero OK), no N DEVICE
 *   Wellon        : STX, no ETX, TYPE:XC9536*44PL -> QF0000 inside a multi-
 *                   line token; the real count comes from the L records
 */

/* scan_field: find the first occurrence of <prefix> anywhere inside a
 * token string (not just at start), then parse what follows with strtol.
 * Returns pointer to the digit string after the prefix, or NULL if not found. */
static const char *scan_field(const char *tok, size_t tl,
                               const char *prefix, size_t plen) {
    for (size_t i = 0; i + plen < tl; i++) {
        if (strncasecmp(tok + i, prefix, plen) == 0 &&
            (i == 0 || !isalnum((unsigned char)tok[i-1])))
            return tok + i + plen;
    }
    return NULL;
}

static char *read_whole_file(const char *path, size_t *len_out) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long sz = ftell(f);
    if (sz < 0) { fclose(f); return NULL; }
    rewind(f);
    char *buf = malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[got] = '\0';
    if (len_out) *len_out = got;
    return buf;
}

static void trim(char **tok, size_t *len) {
    char *t = *tok;
    size_t l = *len;
    while (l > 0 && (unsigned char)*t <= ' ') { t++; l--; }
    while (l > 0 && (unsigned char)t[l - 1] <= ' ') l--;
    *tok = t;
    *len = l;
}

int jed_load(const char *path, JedFile *out) {
    memset(out, 0, sizeof(*out));

    size_t len;
    char *buf = read_whole_file(path, &len);
    if (!buf) {
        fprintf(stderr, "jed_load: cannot open %s\n", path);
        return -1;
    }
    char *end = buf + len;

    /* Many .jed files carry a text header ("Programmer Jedec Bit Map",
     * date, etc.) before the STX (0x02) delimited JEDEC content. It is not
     * part of the field structure; if an STX byte is present, start
     * parsing from there. */
    char *start = buf;
    char *stx = memchr(buf, 0x02, len);
    if (stx) start = stx;

    long qf = -1;
    int default_fuse = 0;

    /* First pass: find QF (total fuse count), F0/F1 (default value), and
     * device name.  Search each field ANYWHERE in the token body, not just
     * at the token start, to handle:
     *   - Wellon: "44PL\r\nDM: XILINX\r\nQF0000" -- QF inside the token
     *   - Wellon: "\x02...\r\nTYPE:XC9536" -- TYPE: device id inside the token */
    for (char *p = start; p < end;) {
        char *star = memchr(p, '*', (size_t)(end - p));
        size_t toklen = star ? (size_t)(star - p) : (size_t)(end - p);
        char *tok = p;
        size_t tl = toklen;
        trim(&tok, &tl);

        /* QF<n>: fuse count -- search anywhere in token */
        if (qf <= 0) {
            /* QP<n> is the package pin count. It does not name the
             * package but constrains it -- QP84 cannot be TQ100. */
            {   const char *pp = scan_field(tok, tl, "QP", 2);
                if (pp && isdigit((unsigned char)*pp))
                    out->pin_count = (int)strtol(pp, NULL, 10);
            }
            const char *qp = scan_field(tok, tl, "QF", 2);
            if (qp && isdigit((unsigned char)*qp)) {
                long v = strtol(qp, NULL, 10);
                if (v > 0) qf = v;
                else        qf = 0;   /* QF0000 -- Wellon placeholder; see below */
            }
        }

        /* F0 / F1: default fuse value (must be exactly the 2-char token "F0"/"F1") */
        if (tl == 2 && tok[0] == 'F' && (tok[1] == '0' || tok[1] == '1')) {
            default_fuse = tok[1] - '0';
        }

        /* Device name: "N DEVICE <name>" (ISE) or "TYPE:<name>" (Wellon) */
        if (!out->device[0]) {
            const char *dp = NULL;
            if ((dp = scan_field(tok, tl, "N DEVICE", 8)) != NULL) {
                while (*dp == ' ') dp++;
            } else if ((dp = scan_field(tok, tl, "TYPE:", 5)) != NULL) {
                while (*dp == ' ') dp++;
            }
            if (dp && *dp && *dp != '\r' && *dp != '\n') {
                size_t n = 0;
                /* copy until whitespace or end of token */
                while (dp + n < tok + tl && dp[n] != '\r' && dp[n] != '\n'
                       && dp[n] != ' ' && dp[n] != '\t')
                    n++;
                if (n >= sizeof(out->device)) n = sizeof(out->device) - 1;
                memcpy(out->device, dp, n);
                out->device[n] = '\0';
            }
        }

        p = star ? star + 1 : end;
    }

    /* Wellon QF0000: placeholder -- determine actual count by scanning L records
     * to find max(addr + bit_count).  Works for any device without needing
     * a device-to-fuse-count table. */
    if (qf == 0) {
        long max_end = 0;
        for (char *p = start; p < end;) {
            char *star = memchr(p, '*', (size_t)(end - p));
            size_t toklen = star ? (size_t)(star - p) : (size_t)(end - p);
            char *tok = p; size_t tl = toklen;
            trim(&tok, &tl);
            if (tl > 1 && (tok[0] == 'L' || tok[0] == 'l') &&
                isdigit((unsigned char)tok[1])) {
                char *numend;
                long addr = strtol(tok + 1, &numend, 10);
                long bits = 0;
                for (char *c = numend; c < tok + tl; c++)
                    if (*c == '0' || *c == '1') bits++;
                if (addr + bits > max_end) max_end = addr + bits;
            }
            p = star ? star + 1 : end;
        }
        if (max_end > 0) {
            qf = max_end;
        }
    }

    if (qf <= 0) {
        fprintf(stderr, "jed_load: no QF record (and no L records) found in %s\n", path);
        free(buf);
        return -1;
    }

    out->count = (size_t)qf;
    out->fuses = malloc(out->count);
    if (!out->fuses) { free(buf); return -1; }
    memset(out->fuses, default_fuse, out->count);

    /* Second pass: apply every L<addr> <bits>* record. */
    for (char *p = start; p < end;) {
        char *star = memchr(p, '*', (size_t)(end - p));
        size_t toklen = star ? (size_t)(star - p) : (size_t)(end - p);
        char *tok = p;
        size_t tl = toklen;
        trim(&tok, &tl);

        if (tl > 1 && tok[0] == 'L' && isdigit((unsigned char)tok[1])) {
            char *numend;
            long addr = strtol(tok + 1, &numend, 10);
            long idx = addr;
            for (char *c = numend; c < tok + tl; c++) {
                if (*c == '0' || *c == '1') {
                    if (idx >= 0 && (size_t)idx < out->count)
                        out->fuses[idx] = (unsigned char)(*c - '0');
                    idx++;
                }
                /* anything else (space, newline) is cosmetic formatting */
            }
        }
        p = star ? star + 1 : end;
    }

    free(buf);
    return 0;
}

void jed_free(JedFile *jf) {
    free(jf->fuses);
    jf->fuses = NULL;
    jf->count = 0;
}
