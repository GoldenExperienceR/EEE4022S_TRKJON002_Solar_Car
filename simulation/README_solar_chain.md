# Solar charging chain model – template (solar_chain.slx)

A Simulink template for validating a model of the car's charging chain against the SIM-01 outdoor test.
It is **not complete**: the MPPT controller and the converter are empty subsystems (yellow) with defined
inputs and outputs, and some PV and battery parameters are placeholders (`[TBD]` in `params_solar.m`).

Irradiance + temperature -> PV panels (2 in series) -> converter <- duty D <- MPPT controller
-> 3S battery in parallel with the idle load. Sensors sit where the car's are: panel, MPPT output, battery.

| File | Purpose |
|---|---|
| params_solar.m | All parameters, tagged DETERMINED / MODEL / TBD, with how to find each TBD value |
| build_solar.m | Generates solar_chain.slx (rebuilt from scratch each run) |
| run_solar.m | Validation harness: runs the SIM-01 session and compares with the measured log |
| data/sun_session.csv | SIM-01 irradiance and panel temperature, 1-minute steps (t_min, G_Wm2, T_C) |
| data/log.csv | SIM-01 dashboard log (pack V/I, MPPT output power and current) |

## To complete the model
1. Characterise the panels (I-V sweep at known G and T) -> Vm, Im, cell count, temperature coefficients.
2. Identify the MPPT board (steps in load and light) -> algorithm, current limit, loop speed.
   Write the `MPPT_controller` subsystem (inputs v_pan, i_out, v_bat; output D).
3. Write the `Buck_converter` subsystem (inputs D, v_in, v_bat; outputs i_out, i_in), keeping
   the power balance v_in*i_in*eta = (v_bat + Vf)*i_out with eta from U-07.
4. Run `build_solar`, then `out = run_solar;` and check: MPPT output power (RMSE), pack voltage,
   and net energy into the pack within ±10 % of the measured value.
