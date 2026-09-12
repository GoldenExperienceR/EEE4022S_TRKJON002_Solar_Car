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
| `firmware/` | Embedded source for the on-board controller — MPPT, motor drive, telemetry, power management state machine. |
| `simulation/` | Simulink models and MATLAB scripts. PV, battery, motor and drivetrain submodels. |
| `analysis/` | Python/Jupyter post-processing — test data reduction, efficiency calculations, plots for the report. |
| `hardware/` | Electrical schematics, PCB files, mechanical drawings, and the bill of materials. |
| `docs/` | Design notes, test procedures, meeting notes, and outreach material. |

## Constraints

- Total bill of materials below **R2000**.
- Powered solely by solar PV; energy-efficient design maximising run time per charge.
- Interactive and demonstrable — spanning mechanical, electrical, power, and embedded domains.

## Toolchain

- **Simulation:** MATLAB / Simulink
- **Firmware:** STM32CubeIDE (STM32CubeMX for configuration) / Arduino IDE
- **Analysis:** Python via Anaconda, Jupyter
- **CAD:** Fusion 360 / FreeCAD
- **References:** Zotero

## Conventions

- Binary model and CAD files are tracked but never diffable — commit them with a
  descriptive message stating what changed, since the diff will not tell you.
- Raw test captures stay out of git (see `.gitignore`); commit the reduced data
  and the script that reduced it.
