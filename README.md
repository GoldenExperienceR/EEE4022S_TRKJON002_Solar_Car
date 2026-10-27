# EEE4022S — Small-Scale Solar Car

Power efficiency, recovery, and generation strategies for a small-scale solar car
built for educational demonstration and outreach.

**Author:** Jonathan (Yonatan) Tirkaso — TRKJON002
**Course:** EEE4022S, Mechatronics, University of Cape Town
**Year:** 2026

---

## Repository layout

| Folder | Contents |
|---|---|
| `firmware/` | Final car firmware (`car_main_v3`, ESP32: dashboard, telemetry, co-pilot, drive) and the tracker firmware (`tracker_nano`); earlier versions in `firmware/archive/`. |
| `testing code/` | Proof-of-concept spikes and unit-test sketches, named by test ID (e.g. `U-06_ina219_calibration`). Never promoted into `firmware/`. |
| `simulation/` | Simulink model (`solar_car.slx`) and MATLAB scripts for the solar charging chain. |
| `modelling and schematics/` | `schematics/` (KiCad project), `step files/` (CAD) and `online components/` (downloaded STL parts). |
| `media/` | Test evidence, one folder per test ID (`media/<test-id>/<test-id>_<what>.mp4`). |
| `results/` | Test data and logs, one folder per test ID. |

### Schematics (KiCad)

Open `modelling and schematics/schematics/solar_car_kicad/solar_car.kicad_pro` in KiCad 9 (files are saved in KiCad 7 format; KiCad upgrades them on first save). The project-local symbol library `solar_car` is registered in `sym-lib-table`.

Sheets: Power subsystem, Controller and driver, Solar tracker. Nets cross sheets via global labels; GND, +3V3, +5V, +VMOT, VBAT and VSYS are power symbols.

To change controller wiring, edit `PINMAP` at the top of `gen.py`, run `python gen.py` and reopen in KiCad. Regenerating overwrites manual edits to the `.kicad_sch` files.

## Constraints

- Total bill of materials within **R2300** (the typical R2000 course budget plus 15%, approved by the supervisor).
- Powered by onboard solar PV with USB-C plug-in charging; energy-efficient design maximising run time per charge.
- Interactive and demonstrable — spanning mechanical, electrical, power, and embedded domains.

## Toolchain

- **Simulation:** MATLAB / Simulink
- **Firmware:** Arduino IDE (ESP32 and Arduino Nano)
- **Analysis:** Python via Anaconda, Jupyter
- **CAD:** Fusion 360 / FreeCAD
- **Schematics:** KiCad
- **References:** Zotero

## Conventions

- Binary model and CAD files are tracked but never diffable — commit them with a
  descriptive message stating what changed, since the diff will not tell you.
- Raw test captures stay out of git (see `.gitignore`); commit the reduced data
  and the script that reduced it.
