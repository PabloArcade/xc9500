#ifndef UCF_H
#define UCF_H
#include <stddef.h>

typedef struct {
    char name[64];
    int  pin;        /* numeric pin; 0 if BGA ball not parsed as number */
    char ball[8];    /* raw ball string (e.g. "B7" for CS48) */
} UcfEntry;

typedef struct {
    UcfEntry *entries;
    size_t    count;
} UcfFile;

int  ucf_load(const char *path, UcfFile *out);
void ucf_free(UcfFile *uf);

#endif /* UCF_H */
