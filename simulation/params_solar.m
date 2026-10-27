function p = params_solar()
%PARAMS_SOLAR  Parameters for the solar_chain.slx template.
%   Each value is tagged:
%     [DETERMINED <test/DS>]  measured in that test or taken from a datasheet; use as is
%     [MODEL]                 numerical value needed by the model, not a property of the car
%     [TBD]                   placeholder; not yet determined - see the note on how to find it
%   The MPPT controller and converter blocks are empty until their behaviour is identified.

%% Simulation
p.sim.Ts   = 5e-4;     % [MODEL] fixed step [s]; fine enough for an averaged converter
p.sim.Tlog = 0.2;      % [MODEL] logging interval [s] (5 Hz, same as the dashboard log)

%% PV string: 2 x 4.2 W panels in series (PV Array block, single-diode model)
p.pv.Nser  = 2;        % [DETERMINED build]
p.pv.Voc   = 11.255;   % per module [V]  [DETERMINED U-16] mean of 11.24 / 11.27 V
p.pv.Isc   = 0.894;    % per module [A]  [DETERMINED U-16] scaled to 1000 W/m2
p.pv.Vm    = 9.0;      % per module [V]  [TBD] supplier lists 9 V (assumed Vmp, not confirmed). Find: I-V sweep with an electronic/variable load
p.pv.Im    = 0.80;     % per module [A]  [TBD] placeholder 0.9 x Isc; supplier rating 4.2 W / 9 V gives 0.47 A. Find: same I-V sweep
p.pv.Ncell = 18;       % cells per module [TBD] inferred from Voc (about 0.63 V per cell). Find: count cells / supplier data
p.pv.beta_Voc_pc  = -0.373;   % [%/degC] [TBD] typical polycrystalline Si (Cotfas 2018); confirm with hot/cold Voc
p.pv.alpha_Isc_pc =  0.087;   % [%/degC] [TBD] typical polycrystalline Si (Cotfas 2018)

%% MPPT module (5 A MPPT board) - controller and converter behaviour still to be identified
p.mppt.Vset = 14.8;    % panel operating voltage while charging [V] [DETERMINED SIM-01] multimeter
p.mppt.eta  = 0.831;   % conversion efficiency [DETERMINED U-07] (lower than the datasheet figure)
p.mppt.Vcv  = 12.6;    % charge cut-off voltage [V] [DETERMINED] set for 3S (4.2 V per cell)
p.mppt.Imax = NaN;     % charge-current limit [A] [TBD] board preset unclear (datasheet lists up to 5 A). Find: charge into a low pack in full sun
p.mppt.Cin  = 4.7e-3;  % input capacitance [F] [MODEL] gives the panel node a voltage state; value sets response speed only

%% Diode-OR into the system node (D1, 1N5822)
p.d1.Vf = 0.35;        % [V] [DETERMINED datasheet]

%% Battery: 3S Samsung INR18650-35E (Battery block, Li-ion)
p.batt.Vcell0 = 12.18/3;       % start cell voltage [V] [DETERMINED SIM-01] rested start pack voltage / 3
p.batt.Q     = 3.5;            % [Ah] [DETERMINED datasheet]
p.batt.Vnom  = 3.6*3;  p.batt.Vfull = 4.2*3;  p.batt.Vmin = 2.65*3;   % [V] [DETERMINED datasheet]
p.batt.R     = 0.15;           % pack internal resistance [ohm] [TBD] estimate. Find: voltage step under a known load step
p.batt.Inom  = 0.2*p.batt.Q;   % [MODEL] Battery-block curve-fit values below
p.batt.Qnomv = 0.9043*p.batt.Q;  p.batt.Vexp = 1.0804*p.batt.Vnom;  p.batt.Qexp = 0.04913*p.batt.Q;
p.batt.Tr    = 30;
p.batt.SOC0  = soc_from_cell(p.batt.Vcell0);   % same OCV table as the firmware

%% Idle system load (constant current sink across the pack)
p.load.I = 0.21;       % [A] [DETERMINED SIM-01] pv_A - charge current, parked, tracker fixed (SYS-11: 0.23 A with tracker)

%% Sign conventions for the controlled current sources and the panel current sensor
p.sgn.ccs  = 1;        % [MODEL] flip to -1 if the battery discharges when it should charge
p.sgn.ipan = 1;        % [MODEL] flip to -1 if panel current logs negative
end

function s = soc_from_cell(v)
% Rested OCV-SOC per cell, as in car_main firmware (Yadav et al. 2026, Table 4)
c = [2.80 3.4212 3.5024 3.5517 3.5877 3.6272 3.6867 3.7732 3.8702 3.9732 4.1307];
s = interp1(c, 0:10:100, min(max(v, c(1)), c(end)));
end
