function out = run_solar(envfile, logfile)
%RUN_SOLAR  Validation harness: simulate the SIM-01 sun session and compare with the measured log.
%   Ready to use once the MPPT controller and converter blocks in the template are completed
%   (until then the model outputs zero charge power).
%   out = run_solar('data/sun_session.csv', 'data/log.csv')
%   sun_session.csv : t_min, G_Wm2, T_C   (1-minute readings; held between readings)
%   log.csv         : the dashboard's log download (t_s, pack_V, pack_A, pv_W, pv_A, panel_V, panel_A, ...)
%   Start state and V_set come from params_solar.m.

here = fileparts(mfilename('fullpath'));
if nargin < 1, envfile = fullfile(here, 'data', 'sun_session.csv'); end
if nargin < 2, logfile = fullfile(here, 'data', 'log.csv'); end
E = readtable(envfile);  t = 60*E.t_min;
p = params_solar();
haveLog = isfile(logfile);
if haveLog
    M = readtable(logfile);  M.t_s = M.t_s - M.t_s(1);
    % start state comes from the rested pack voltage in params_solar (the logged pack_V is under charge)
end
fprintf('run_solar: Vset = %.2f V, start cell voltage %.3f V (SOC0 %.0f %%)\n', p.mppt.Vset, p.batt.Vcell0, p.batt.SOC0);

assignin('base', 'p', p);
assignin('base', 'G_in', [t E.G_Wm2]);
assignin('base', 'T_in', [t E.T_C]);
mdl = 'solar_chain';
if ~isfile(fullfile(here, [mdl '.slx'])), build_solar(); end
load_system(mdl);
so = sim(mdl, 'StopTime', num2str(t(end)), 'ReturnWorkspaceOutputs','on');
y = so.simlog.signals.values;  ts = so.simlog.time;
out = struct('t',ts, 'G',y(:,1), 'T',y(:,2), 'v_pan',y(:,3), 'i_pan',y(:,4), 'p_pan',y(:,5), 'D',y(:,6), ...
    'i_out',y(:,7), 'p_out',y(:,8), 'v_bat',y(:,9), 'i_bat',y(:,10), 'p_bat',y(:,11));

figure('Name','Charge profile: simulated vs measured');
tiledlayout(3,1);
nexttile; plot(ts/60, out.p_out); hold on; grid on; ylabel('MPPT output power (W)');
nexttile; plot(ts/60, out.v_bat); hold on; grid on; ylabel('Pack voltage (V)');
nexttile; plot(ts/60, out.v_pan); hold on; grid on; ylabel('Panel voltage (V)'); xlabel('Time (min)');
if haveLog
    tm = M.t_s/60;
    nexttile(1); plot(tm, M.pv_W, '.'); legend('simulated','measured');
    nexttile(2); plot(tm, M.pack_V, '.'); legend('simulated','measured');
    if ismember('panel_V', M.Properties.VariableNames), nexttile(3); plot(tm, M.panel_V, '.'); legend('simulated','measured'); end
    k = M.t_s <= ts(end);
    out.err_p = report('MPPT output power', M.pv_W(k),   interp1(ts, out.p_out, M.t_s(k)), 'W');
    out.err_v = report('Pack voltage',      M.pack_V(k), interp1(ts, out.v_bat, M.t_s(k)), 'V');
    Em = trapz(M.t_s(k), M.pv_W(k))/3600;  Es = trapz(ts, out.p_out)/3600;
    fprintf('  Solar energy into the system: measured %.2f Wh, simulated %.2f Wh (%+.1f %%)\n', Em, Es, 100*(Es-Em)/Em);
end
end

function e = report(name, meas, model, unit)
k = isfinite(meas) & isfinite(model);
e.rmse = sqrt(mean((model(k) - meas(k)).^2));
e.mean_pct = 100*mean(model(k) - meas(k))/mean(meas(k));
fprintf('  %-18s RMSE %.3f %s, mean difference %+.1f %% (%d points)\n', name, e.rmse, unit, e.mean_pct, nnz(k));
end
