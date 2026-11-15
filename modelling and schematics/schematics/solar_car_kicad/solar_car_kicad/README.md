# Solar car - KiCad schematic

Open `solar_car.kicad_pro` in KiCad 9 (files are saved in KiCad 7 format; KiCad upgrades them on first save).
The project-local symbol library `solar_car` is registered in `sym-lib-table`.

Sheets: Power subsystem · Controller and driver · Solar tracker. Nets cross sheets via global labels;
GND/+3V3/+5V/+VMOT/VBAT/VSYS are power symbols.

To change controller wiring: edit `PINMAP` at the top of `gen.py`, run `python gen.py`, reopen in KiCad.
(Regenerating overwrites manual edits to the .kicad_sch files.)
