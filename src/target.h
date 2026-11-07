/* target.h -- explicit device/package selection */
#ifndef TARGET_H
#define TARGET_H
#include "device.h"
#include "jed.h"

void target_viewlist(void);

/* Prints the requested device and package beside what the JED names, flags
 * any disagreement, and returns the resolved device row (NULL if the target
 * is unknown or either field was omitted).
 * *out_conflict is set when the JED's own fields disagree with the request. */
const device_t *target_resolve(const JedFile *jf,
                               const char *want_dev, const char *want_pkg,
                               int *out_conflict);
#endif
