#include "ucf.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* UCF files write LOC="5" or LOC="P5". Strip an optional leading P/p
 * before parsing the numeric pin. */
static int parse_pin(const char *s) {
    while (*s == ' ') s++;
    if (*s == 'P' || *s == 'p') s++;
    return (int)strtol(s, NULL, 10);
}

int ucf_load(const char *path, UcfFile *out) {
    memset(out, 0, sizeof(*out));
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "ucf_load: cannot open %s\n", path); return -1; }

    size_t cap = 32;
    out->entries = malloc(cap * sizeof(UcfEntry));
    out->count = 0;

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        /* strip comment */
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';

        char *net = strstr(line, "NET");
        if (!net) continue;
        char *q1 = strchr(net, '"');
        if (!q1) continue;
        char *q2 = strchr(q1 + 1, '"');
        if (!q2) continue;

        char name[64];
        size_t namelen = (size_t)(q2 - q1 - 1);
        if (namelen >= sizeof(name)) namelen = sizeof(name) - 1;
        memcpy(name, q1 + 1, namelen);
        name[namelen] = '\0';

        char *loc = strstr(q2, "LOC");
        if (!loc) continue;
        char *q3 = strchr(loc, '"');
        if (!q3) continue;
        char *q4 = strchr(q3 + 1, '"');
        if (!q4) continue;

        char pinbuf[32];
        size_t pinlen = (size_t)(q4 - q3 - 1);
        if (pinlen >= sizeof(pinbuf)) pinlen = sizeof(pinbuf) - 1;
        memcpy(pinbuf, q3 + 1, pinlen);
        pinbuf[pinlen] = '\0';

        if (out->count == cap) {
            cap *= 2;
            out->entries = realloc(out->entries, cap * sizeof(UcfEntry));
        }
        snprintf(out->entries[out->count].name, sizeof(out->entries[0].name), "%s", name);
        out->entries[out->count].name[sizeof(out->entries[0].name) - 1] = '\0';
        out->entries[out->count].pin = parse_pin(pinbuf);
        out->count++;
    }

    fclose(f);
    return 0;
}

void ucf_free(UcfFile *uf) {
    free(uf->entries);
    uf->entries = NULL;
    uf->count = 0;
}
