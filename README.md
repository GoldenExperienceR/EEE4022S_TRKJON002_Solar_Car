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
| `firmware/` | Final car firmware `car_main_v4` (ESP32: dashboard, telemetry, co-pilot, drive; adds panel-side INA219 at 0x44) and the tracker firmware `tracker_nano` (Arduino Nano). `car_main_v3` is the previous release; earlier versions are in `firmware/archive/`. |
| `testing code/` | Proof-of-concept spikes and unit/integration test sketches, named by test ID (e.g. `U-06_ina219_calibration`). Never promoted into `firmware/`. |
| `simulation/` | Simulink models and MATLAB scripts: `solar_car.slx` (full vehicle, `build_sev.m` / `run_sev.m`) and `solar_chain.slx` (charging chain validated against SIM-01, `build_solar.m` / `run_solar.m`; see `README_solar_chain.md`). |
| `modelling and schematics/` | `step files/` (vehicle chassis and solar tracker assemblies, zipped STEP), `online components/` (downloaded STL parts) and `schematics/`. |
| `figures/` | Report figures, named by chapter (`ch2_`, `ch3_`, `ch4_`), appendix (`app_`, `appx_`) or test ID. Includes the power, control and tracker schematics (`ch3_*_schematic.pdf/.png`). |
| `media/` | Test and outreach evidence, one folder per test ID (`media/<test-id>/<test-id>_<what>.mp4`). `out-01/` holds the outreach plan, ethics approval, consent form, questionnaire, presentation and responses. |
| `results/` | Test data and logs, one folder per test ID. |

## Constraints

- Total bill of materials within **R2300** (the typical R2000 course budget plus 15%, approved by the supervisor).
- Powered by onboard solar PV with USB-C plug-in charging; energy-efficient design maximising run time per charge.
- Interactive and demonstrable — spanning mechanical, electrical, power, and embedded domains.

## Toolchain

- **Simulation:** MATLAB / Simulink
- **Firmware:** Arduino IDE (ESP32 and Arduino Nano)
- **Analysis:** Python via Anaconda, Jupyter
- **CAD:** Onshape (3D-printed parts), laser cutting for acrylic decks

## Conventions

- Binary model and CAD files are tracked but never diffable — commit them with a
  descriptive message stating what changed, since the diff will not tell you.
- Raw test captures stay out of git (see `.gitignore`); commit the reduced data
  and the script that reduced it.
- One current version of each results or media file — replace a corrected file
  rather than keeping `v1`/`v2` copies.
