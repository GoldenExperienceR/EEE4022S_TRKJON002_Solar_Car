function R = run_sev(scenario, modes, p)
%RUN_SEV  Run a scenario for one or more MPPT modes and compare them.
%   R = run_sev('steps', [0 1 2 3 4])
%   Scenarios: 'check' 3 s, 'steps' 60 s irradiance steps, 'cloud' 300 s
%   passing clouds, 'charge' 30 min full sun, 'drive' 10 min sun + load.
%   Modes: 0=OFF 1=CV 2=P&O 3=IncCond 4=FOCV.  Optional p overrides params_sev.
if nargin < 1, scenario = 'steps'; end
if nargin < 2, modes = [0 1 2 3 4]; end
if nargin < 3, p = params_sev(); end
mdl = 'sev_benchmark';
if ~isfile(fullfile(fileparts(mfilename('fullpath')), [mdl '.slx'])), build_sev(); end
load_system(mdl);
[t, G, T, Il, p] = profile(scenario, p);

names = {'OFF','CV','P&O','IncCond','FOCV'};
R = struct([]);
for k = 1:numel(modes)
    p.mppt.mode = modes(k);
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('p', p);
    in = in.setVariable('G_in', [t G]);
    in = in.setVariable('T_in', [t T]);
    in = in.setVariable('Iload_in', [t Il]);
    in = in.setModelParameter('StopTime', num2str(t(end)));
    tic; out = sim(in); el = toc;
    tt = out.simlog.time;  Y = out.simlog.signals.values;
    Pav = pv_mpp(Y(:,1), Y(:,2), p);
    R(k).mode = modes(k);  R(k).name = names{modes(k)+1};
    R(k).t = tt;  R(k).Y = Y;  R(k).Pav = Pav;  R(k).batt = out.battlog;
    R(k).E_av  = trapz(tt, Pav)/3600;               % Wh available
    R(k).E_pv  = trapz(tt, Y(:,5))/3600;            % Wh harvested from PV
    R(k).E_out = trapz(tt, Y(:,9).*Y(:,7))/3600;    % Wh delivered by charger
    R(k).eta_track = R(k).E_pv/max(R(k).E_av, eps);
    R(k).eta_conv  = R(k).E_out/max(R(k).E_pv, eps);
    R(k).SOC_end = Y(end,11);
    fprintf('%-8s track %5.1f %%  conv %5.1f %%  E_out %6.3f Wh  SOC %5.1f -> %5.1f %%  (%.0f s wall)\n', ...
        R(k).name, 100*R(k).eta_track, 100*R(k).eta_conv, R(k).E_out, Y(1,11), R(k).SOC_end, el);
end
plot_runs(R, scenario);
end

function [t, G, T, Il, p] = profile(sc, p)
switch lower(sc)
    case 'check'
        t = (0:0.01:3)';   G = 1000 + 0*t;  p.batt.SOC0 = 50;
    case 'steps'
        t = (0:0.01:60)';  G = 1000*(t<15) + 600*(t>=15 & t<30) + 200*(t>=30 & t<45) + 900*(t>=45);
        p.batt.SOC0 = 50;
    case 'cloud'
        rng(1);  t = (0:0.1:300)';  c = zeros(size(t));
        for j = 1:6, c = c + 0.6*exp(-((t - 300*rand)/(5 + 15*rand)).^2); end
        G = 950*(1 - min(c, 0.85));  p.batt.SOC0 = 50;
    case 'charge'
        t = (0:1:1800)';   G = 1000 + 0*t;  p.batt.SOC0 = 20;
    case 'drive'
        t = (0:0.5:600)';  G = 800 + 0*t;   p.batt.SOC0 = 80;
    otherwise
        error('run_sev:scenario', 'Unknown scenario "%s".', sc);
end
T  = 25 + 0*t;                                 % cell temperature [degC]
Il = p.load.Ipar + 0*t;                        % stage 1: parasitic draw only
if strcmpi(sc, 'drive'), Il = Il + 0.8*(mod(t, 60) < 40); end   % placeholder drive load [A]
if t(end) > 120                                % long run: coarser step
    p.sim.Ts = 1e-3;  p.conv.tau = 1e-2;
end
end

function plot_runs(R, sc)
figure('Name', ['SEV - ' sc]);  tl = tiledlayout(4, 1, 'TileSpacing','compact');
ax = nexttile; hold on; plot(R(1).t, R(1).Pav, 'k--', 'DisplayName','P_{avail}');
for k = 1:numel(R), plot(R(k).t, R(k).Y(:,5), 'DisplayName', R(k).name); end
ylabel('P_{pv} [W]'); legend('Location','best'); grid on
nexttile; hold on; for k = 1:numel(R), plot(R(k).t, R(k).Y(:,3)); end; ylabel('V_{pv} [V]'); grid on
nexttile; hold on; for k = 1:numel(R), plot(R(k).t, R(k).Y(:,7)); end; ylabel('I_{chg} [A]'); grid on
nexttile; hold on; for k = 1:numel(R), plot(R(k).t, R(k).Y(:,11)); end; ylabel('SOC [%]'); xlabel('t [s]'); grid on
title(tl, sc); linkaxes(findall(gcf,'Type','axes'), 'x'); %#ok<NASGU>
end
