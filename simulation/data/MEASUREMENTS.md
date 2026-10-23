# Measurement slots

Drop the files here (copy a `_TEMPLATE.csv`, drop the suffix). Scalars go straight into `params_sev.m` at the `[MEAS]` lines.

| # | Measure | How | File / param | Feeds |
|---|---|---|---|---|
| 1 | PV I-V curve, per panel and string | Sweep a load (rheostat/e-load) from short to open; note irradiance + panel temp | `pv_iv_G<W>_T<C>.csv` (`V_string_V,I_A`) | PV benchmark; refit `p.pv.*` |
| 2 | Charger operating points (5–10, varied sun/SOC) | INA219s / meters on charger in and out at the same time | `conv_eff.csv` (`Vin_V,Iin_A,Vout_V,Iout_A`) | **auto-fits** `p.conv.Vf`, `p.conv.Req` |
| 3 | Charger PV regulation voltage | V_pv while charging in steady sun | `p.mppt.Vref` | CV (real-module) mode |
| 4 | Charger current limit + CV voltage | Max I_out at low SOC; V_bat at end of charge | `p.chg.Imax`, `p.chg.Vcv` | charger limits |
| 5 | Input capacitance | Read cap markings on board | `p.conv.Cin` | dynamics |
| 6 | Pack internal resistance | Rest, apply a load step, R = ΔV/ΔI (at ~50 % SOC) | `batt_step.csv` → `p.batt.R` | battery |
| 7 | Pack capacity | Full → BMS cut-off at a constant load | `p.batt.Q`, `p.batt.Vmin` | battery |
| 8 | Parasitic current | ESP32 + sensors + Nano idle, motors off | `p.load.Ipar` | load |
| 9 | Full-system runs (validation) | Log everything during a charge test and an elevated motor run | `run_log_<name>.csv` | sim-vs-real comparison (stage 4) |

Stage 3 (load) will add: XL4015 efficiency vs current at 7 V, motor current vs PWM (elevated), stall current, vehicle mass, wheel radius.
