function p = params_sev()
%PARAMS_SEV  Every number the SEV solar-subsystem model uses, in one place.
%   p = params_sev();
%   Lines tagged [MEAS] are placeholders (datasheet/estimate) waiting for your
%   bench measurements. Curves go in data/*.csv (see data/MEASUREMENTS.md) and
%   are loaded automatically at the bottom of this file when present.
%   NOTE: p is passed into MATLAB Function blocks, so keep it numeric-only.

%% Simulation
p.sim.Ts   = 1e-4;   % electrical step [s] (run_sev switches to 1e-3 for long runs)
p.sim.Tlog = 0.1;    % logging interval [s]

%% PV module (one panel) - values copied from your solar_car.slx PV Array block
p.pv.Nstr  = 2;           % modules in series (two PV Array blocks)
p.pv.Pm    = 4.14;        % [W]   [MEAS] confirm with I-V sweep
p.pv.Ncell = 18;
p.pv.Voc   = 10.9;        % [V]   [MEAS]
p.pv.Isc   = 0.495;       % [A]   [MEAS]
p.pv.Vm    = 9;           % [V]   [MEAS]
p.pv.Im    = 0.46;        % [A]   [MEAS]
p.pv.beta_Voc_pc  = -0.37;     % [%/degC]
p.pv.alpha_Isc_pc = 0.102;     % [%/degC]
% Single-diode parameters (computed by the PV Array mask for the values above;
% refit if you change Voc/Isc/Vm/Im)
p.pv.IL  = 0.49567;  p.pv.I0 = 2.498e-11;  p.pv.nI = 0.99514;
p.pv.Rsh = 848.6941; p.pv.Rs = 1.1458;
p.pv.alpha_Isc = p.pv.alpha_Isc_pc/100*p.pv.Isc;   % [A/degC]
p.pv.beta_Voc  = p.pv.beta_Voc_pc/100*p.pv.Voc;    % [V/degC]

%% MPPT charger power stage (averaged buck)
p.conv.Cin  = 470e-6;  % input capacitance [F]          [MEAS] read off the board
p.conv.Req  = 0.30;    % lumped conduction loss [ohm]    [MEAS] fit from data/conv_eff.csv
p.conv.Vf   = 0.35;    % freewheel diode drop [V]        [MEAS] fit from data/conv_eff.csv
p.conv.tau  = 1e-3;    % current-loop lag (stands in for L) [s]
p.conv.Imax = 3;       % hard current clamp [A]

%% MPPT algorithm   mode: 0=OFF(direct, D=1) 1=CV(fixed Vref, models the real module)
%                         2=P&O  3=IncCond  4=FOCV
p.mppt.mode  = 1;
p.mppt.Ts    = 0.05;   % algorithm update period [s]
p.mppt.D0    = 0.9;
p.mppt.Dmin  = 0.05;
p.mppt.Dmax  = 0.98;
p.mppt.Vref  = 18.0;   % CV set-point, string volts [V]  [MEAS] real module's regulation voltage
p.mppt.Kcv   = 0.2;    % CV integral gain [1/(V*s)]
p.mppt.dD    = 0.01;   % P&O / IncCond duty step
p.mppt.eps   = 0.002;  % IncCond dead-band
p.mppt.Toc   = 30;     % FOCV: open-circuit sample period [s]
p.mppt.Tocm  = 0.15;   % FOCV: open-circuit window [s]
p.mppt.kfocv = 0.80;   % FOCV: Vmp/Voc ratio

%% Charger limits (CC/CV) - applied on top of modes 1-4
p.chg.enable = 1;
p.chg.Imax   = 1.0;    % charge-current limit [A]        [MEAS] module setting
p.chg.Vcv    = 12.6;   % CV voltage, 3 x 4.2 V [V]       [MEAS] module setting
p.chg.Kv     = 2.0;    % V->A weighting of CV error
p.chg.K      = 1.0;    % limiter integral gain

%% Battery: 3S Li-ion, 3500 mAh cells (SPS Battery block, custom parameters)
p.batt.Ncell = 3;
p.batt.Q     = 3.5;                    % rated capacity [Ah]       [MEAS] capacity test
p.batt.Vnom  = 3.6*p.batt.Ncell;       % [V]
p.batt.Vfull = 4.2*p.batt.Ncell;       % [V]
p.batt.Vmin  = 0.75*p.batt.Vnom;       % cut-off [V]               [MEAS] BMS cut-off
p.batt.R     = 0.15;                   % pack internal R [ohm]     [MEAS] dV/dI step test
p.batt.Inom  = p.batt.Q/2.3;           % nominal discharge current [A]
p.batt.Qnomv = 0.9043*p.batt.Q;        % capacity at nominal voltage [Ah]
p.batt.Vexp  = 1.0804*p.batt.Vnom;     % exponential-zone voltage [V]
p.batt.Qexp  = 0.04913*p.batt.Q;       % exponential-zone capacity [Ah]
p.batt.Tr    = 30;                     % response time [s]
p.batt.SOC0  = 50;                     % initial SOC [%] (scenarios override)

%% Load (stage 1: current profile from workspace; stage 3 replaces with vehicle model)
p.load.Ipar = 0.15;    % parasitic draw: ESP32 + sensors + Nano [A]  [MEAS]

%% Sign conventions (check_stage1 tells you if either needs flipping)
p.sgn.ccs = 1;         % controlled current sources
p.sgn.ipv = 1;         % PV current measurement

%% Measured data overrides
dataDir = fullfile(fileparts(mfilename('fullpath')), 'data');
f = fullfile(dataDir, 'conv_eff.csv');
if isfile(f), p = fit_conv(p, f); end
end

function p = fit_conv(p, f)
% Fit Vf and Req from measured operating points: loss = Vf*(1-D)*Io + Req*Io^2
M = readmatrix(f);  M = M(all(isfinite(M),2), :);
if size(M,1) < 3, warning('params_sev:conv','conv_eff.csv needs >= 3 rows; skipped.'); return; end
[Vi, Ii, Vo, Io] = deal(M(:,1), M(:,2), M(:,3), M(:,4));
D = Vo./Vi;  loss = Vi.*Ii - Vo.*Io;
x = [(1-D).*Io, Io.^2] \ loss;
p.conv.Vf  = max(x(1), 0);
p.conv.Req = max(x(2), 0.05);
fprintf('params_sev: converter fit from %d points -> Vf = %.3f V, Req = %.3f ohm (eta %.1f-%.1f %%)\n', ...
    size(M,1), p.conv.Vf, p.conv.Req, 100*min(Vo.*Io./(Vi.*Ii)), 100*max(Vo.*Io./(Vi.*Ii)));
end
