# firmware/

Embedded source for the on-board controller.

Expected subsystems:
- `mppt/` — maximum power point tracking (perturb-and-observe or incremental conductance)
- `motor/` — PWM motor drive and current limiting
- `power/` — battery management, charge/discharge supervision, low-voltage cutout
- `telemetry/` — logging and any display/serial output for the demonstration

Target: STM32 (CubeIDE project) or Arduino-compatible board — decide before
writing shared code, and keep hardware abstraction thin.
