#ifndef VENDOR_CELLS_H
#define VENDOR_CELLS_H
/* Vendor control cells: the fuses decode.c does not decode into logic,
 * reported by vendor-writer group (vendor_cells_table.h, generated).
 * Never changes logic: decode and vhdl output only gain this report. */
#include <stdio.h>
#include "device.h"

/* decode: print the vendor control cell block, and a WARNING line per group holding a non-vendor value.
 * vhdl (as_vhdl_comment = 1): print only the WARNING lines, each prefixed "-- ", and nothing when there is none.
 * Returns the number of cells holding a non-vendor value. */
int vendor_cells_report(const device_t *dev, const unsigned char *fuses, unsigned long count, FILE *out, int as_vhdl_comment);

/* A non-XL FB1 MCLK / MOE cell selects the pin that drives a global net:
 *   MCLK j (j=0..2): net GCK<j+1> <- pin GCK<j+1>;   MCLK 3+j: net GCK<j+1> <- pin GCK<((j+1) mod 3)+1>
 *   MOE  j (j<N):    net GTS<j+1> <- pin GTS<j+1>;   MOE  N+j: net GTS<j+1> <- pin GTS<((j+1) mod N)+1>   (N = the part's GTS pins)
 * and FB1 OE<k>I set = global OE net GTS<k+1> carries the complement of its pin.  is_oe 0 = clock, 1 = output enable; net is 1-based.
 * Returns the 1-based pin index driving the net (the net itself on XL parts, with no table, or with neither cell set); *inverted (may
 * be NULL) gets 1 when the OE net carries the complement. Cell addresses come from the generated vendor table, by legend name. */
int vendor_cells_global_src(const device_t *dev, const unsigned char *fuses, unsigned long count, int is_oe, int net, int n_gts,
                            int *inverted);

#endif
