# xc9500dump v6.136 -- source

XC9500 / XC9500XL (.jed) fuse map decompiler by Darksoft.
Reads the JEDEC file of a Xilinx XC9500 or XC9500XL CPLD and writes VHDL + UCF that ISE 14.7 compiles back into a part
that behaves the same. Built for arcade PCB preservation. Use at your own risk.

## Commands

```
xc9500dump v6.136 -- XC9500 family .jed decompiler by Darksoft

Commands:
  xc9500dump dump   <file.jed>                   structural report
  xc9500dump diff   <a.jed> <b.jed>              annotated bit-level diff
  xc9500dump decode --viewlist                   list devices + packages
  xc9500dump decode --device D --package P <f.jed>   PT + attribute decode
  xc9500dump vhdl   --device D --package P <f.jed> [out.vhd] [out.ucf] [out.opt]
  xc9500dump verify <file.jed> <file.ucf>        cross-reference UCF vs JED
  xc9500dump coverage <file.jed>                 per-geometry fuse coverage table
  xc9500dump devices                             list known device tables

Device selection: the JED's own "N DEVICE" name is used first, the fuse count
only when the JED names no device. Every command that reads a .jed REFUSES a
JED that names a device this tool does not support.
Source tables per device, measured against the vendor's .chp: see `devices`.
XL imux tables present for XC9536XL, XC9572XL, XC95144XL
and XC95288XL.
```

Examples:

```
xc9500dump decode --device XC95108 --package PQ100 chip.jed              equations per macrocell, to stdout
xc9500dump vhdl --device XC95144 --package TQ100 chip.jed chip.vhd chip.ucf chip.opt
xc9500dump dump chip.jed                                                 structural report
xc9500dump diff old.jed new.jed                                          what changed, bit by bit
xc9500dump verify chip.jed chip.ucf                                      UCF pin locations against the fuse map
xc9500dump decode --viewlist                                             supported devices and packages
```

- **decode** prints the device and the user signature, then per function block its FastCONNECT sources and, per
  macrocell, its product terms, clock, set / reset, output enable, register type, power and slew settings and the raw
  attribute bits, followed by the vendor control cells.
- **vhdl** writes `<out>.vhd` (for XST), `<out>.ucf` (one LOC per pin, plus the global clock / set-reset / output
  enable pins) and `<out>.opt` (build options: the user signature and the setting for unused pins). Every port is
  placed at its original pin; buried nodes and register chains carry keep attributes so XST does not merge them.
- **dump** prints the structural report: per function block, every AND-array column, the source it carries and
  whether it is active.
- **diff** lists every differing fuse of two JEDs, grouped in clusters, with its function block, region, column and
  offset.
- **verify** lists every signal of a UCF with the pin, function block and macrocell it is placed on and the reset,
  preset, output enable and clock settings the JED gives that macrocell.
- **coverage** prints, per device geometry, how many fuses the tool reads.
- **devices** lists every device table and how its source-name table compares with the vendor's own pin data.

The JED's own device and package names are used when present. When the JED does not name its package, give
`--device` and `--package`; the tool refuses rather than guess a pin table. The tool always prints what the JED says
beside what was asked for. Option names and values are not case-sensitive; `--device=D` also works.

Other options: `--noportfix` keeps an unresolved FastCONNECT source as an undriven signal instead of an input port;
`--nobanner` suppresses the banner.

The banner `xc9500 decoder tool by Darksoft v6.136. Use at your own risk.` goes to standard error, first, on every run.
Exit codes: 0 done, 1 usage, 2 refused or error (the reason is printed).

## Supported parts and packages (10 devices, 34 packages)

    XC9536     PC44 VQ44 CS48
    XC9572     PC44 PC84 PQ100 TQ100
    XC95108    PC84 PQ100 TQ100 PQ160
    XC95144    TQ100 PQ100 PQ160
    XC95216    HQ208 PQ160 BG352
    XC95288    HQ208 BG352
    XC9536XL   PC44 VQ44 CS48 VQ64
    XC9572XL   PC44 VQ44 CS48 VQ64 TQ100
    XC95144XL  TQ100 TQ144 CS144
    XC95288XL  TQ144 PQ208 BG256

A JED naming any other part is refused by name.

## Known limits

- Input is JEDEC only. A programmer's own read-back format (Intel hex, binary dump) is not read; save the read-back
  as .jed first.
- A JED that does not name its package needs `--device` and `--package`. The package code is in the part number
  printed on the chip (XC95108 PQ100AEM -> PQ100).
- Timing is not reproduced. A design that depends on gate delays (a ring oscillator, for example) will not behave the
  same after it is compiled again.
- The VHDL reproduces behaviour and pins, not the original source. Signal names are generated from pins and
  macrocells (`n_p<pin>`, `n_FB<fb>MC<mc>`).
- XL parts: the `decode` listing leaves out an enabled output whose macrocell has no data term. The VHDL for such an
  output is correct.
- `decode` can print 0 product terms for a macrocell where the fitter report says 1. This is an empty sum whose only
  imported term passes straight through to a neighbour; the decode agrees with the fitter's equation for it.
- Read the warnings. A JED that programs a fuse whose function is not known, or a FastCONNECT source that cannot be
  resolved, is reported on the output; the rest of the decode is still valid.

## Rebuild

```
cd src && make             Linux (any c99 compiler, C library only): xc9500dump
cd src && make windows     Windows x86-64 with x86_64-w64-mingw32-gcc: xc9500dump.exe
```

The Windows binary is linked with a 32 MB stack (`-Wl,--stack,33554432`); the decoder keeps large work arrays on the
stack and the default Windows stack is not enough. Every device table is compiled in (`src/device.c`,
`src/vendor_cells_table.h`): decoding needs no other program or data file.
