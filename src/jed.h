#ifndef JED_H
#define JED_H

#include <stddef.h>

/* A parsed JEDEC (.jed) fuse map. fuses[i] is 0 or 1. */
typedef struct {
    unsigned char *fuses;
    size_t count;
    char device[128]; /* from an "N DEVICE ..." note field, if present */
    int  pin_count;   /* from the QP field, if present; 0 = absent.
                       * A package hint; target.c uses it. */
} JedFile;

/* Loads and parses a .jed file. Returns 0 on success, -1 on failure
 * (missing file, no QF record, etc). */
int jed_load(const char *path, JedFile *out);

void jed_free(JedFile *jf);

#endif
