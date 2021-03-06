# Woodstock recreation for small FPGAs

This is a implementation of a Woodstock machine (like the HP-67) for FPGAs.
This current implemenation requires a Lattice MachXO2 with at least 2000 LEs and
11 free block RAMs. The RAMs could be compacted a bit to fit them in 7 Block RAMs
at the minimum. Other modifications like disabling hardware trace could be also 
implemented to reduce LE usage and fit it in a LCMXO2-1200. The target board
used is shown below. The populated FPGA is a LCMXO2-7000ZE.

<p align=left">
  <img src="/01_PS11_7000_RevA/PS11_67.jpg">
  Working prototype
</p>

Compilation report


Design Summary<br>
   Number of registers:    290 out of  7209 (4%)<br>
      PFU registers:          289 out of  6864 (4%)<br>
      PIO registers:            1 out of   345 (0%)<br>
   Number of SLICEs:       644 out of  3432 (19%)<br>
      SLICEs as Logic/ROM:    644 out of  3432 (19%)<br>
      SLICEs as RAM:            0 out of  2574 (0%)<br>
      SLICEs as Carry:         54 out of  3432 (2%)<br>
   Number of LUT4s:        1275 out of  6864 (19%)<br>
      Number used as logic LUTs:        1167<br>
      Number used as distributed RAM:     0<br>
      Number used as ripple logic:      108<br>
      Number used as shift registers:     0<br>
   Number of PIO sites used: 30 + 4(JTAG) out of 115 (30%)<br>
   Number of block RAMs:  11 out of 26 (42%)<br>
   Number of GSRs:  1 out of 1 (100%)<br>
   EFB used :       No<br>
   JTAG used :      No<br>
   Readback used :  No<br>
   Oscillator used :  Yes<br>
   Startup used :   No<br>
   POR :            On<br>
   Bandgap :        On<br>
   Number of Power Controller:  0 out of 1 (0%)<br>
   Number of Dynamic Bank Controller (BCINRD):  0 out of 6 (0%)<br>
   Number of Dynamic Bank Controller (BCLVDSO):  0 out of 1 (0%)<br>
   Number of DCCA:  0 out of 8 (0%)<br>
   Number of DCMA:  0 out of 2 (0%)<br>
   Number of PLLs:  0 out of 2 (0%)<br>
   Number of DQSDLLs:  0 out of 2 (0%)<br>
   Number of CLKDIVC:  0 out of 4 (0%)<br>
   Number of ECLKSYNCA:  0 out of 4 (0%)<br>
   Number of ECLKBRIDGECS:  0 out of 2 (0%)<br>

# Hardware

In the 01_PS11_7000_RevA folder, you can find the design files for the current prototype board.
This board has a landscape format (not like the real HP-67, more like the HP-11C). The board/schematic
where drawn in Eagle v4. Gerber files are also attached. This files do not contain any mistakes,
they work as they are. Except for the fact that the power consumed by the FPGA is a tad bit higher
that I'd happily have at ~7 mA @3.3V.
The internal oscillator of the FPGA may be a big contributor to this value, a fact that has not been
yet explored.

The HP-67 clock is ~180 kHz. Each opcode needs 56 clocks, one per bit, meaning around ~3200 opcodes
per second. I used the internal oscillator at 2.08 MHz and divided it by four to have a 520 kHz. This 
value was chosen more or less arbitrarly. The hardware trace function (activated using h and then f)
slows down the processor to about 800 opcodes per second, but the calculator remains usable, albeit slow.

<p align=left">
  <img src="/01_PS11_7000_RevA/PS11_Top.jpg">
  Top copper side
</p>
<p align=left">
  <img src="/01_PS11_7000_RevA/PS11_Bottom.jpg">
  Bottom copper side
</p>
<p align=left">
  <img src="/01_PS11_7000_RevA/IMG_20170427_134208272.jpg">
  It's alive !
</p>





