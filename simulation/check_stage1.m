%CHECK_STAGE1  Build the model, run 3 s at 1000 W/m^2 in CV mode, and check
%   signs and sanity. Paste the Command Window output back to Claude.
clear; close all;
build_sev();
p = params_sev();
R = run_sev('check', 1, p);
Y = R.Y;  last = Y(:,1) > 0 & R.t > R.t(end) - 0.5;
Voc = p.pv.Nstr*p.pv.Voc;
vpv = mean(Y(last,3));  ipv = mean(Y(last,4));  ic = mean(Y(last,7));
vb0 = Y(2,9);  vb1 = Y(end,9);  soc0 = Y(2,11);  soc1 = Y(end,11);
ok = true;
fprintf('\n--- stage 1 check ---\n');
fprintf('V_pv %.2f V (Voc_string %.1f) | I_pv %.3f A | P_pv %.2f W | I_chg %.3f A | V_bat %.3f -> %.3f V\n', ...
    vpv, Voc, ipv, mean(Y(last,5)), ic, vb0, vb1);
if vb1 < 0
    ok = false; fprintf('FAIL  battery voltage negative -> battery terminals reversed in build_sev (swap nodeB/nodeG battery ports).\n');
end
if vpv > 1.02*Voc || (ic > 0.05 && vb1 < vb0)
    ok = false; fprintf('FAIL  sources inject the wrong way -> set p.sgn.ccs = -1 in params_sev.m\n');
end
if ipv < 0
    ok = false; fprintf('FAIL  PV current negative -> set p.sgn.ipv = -1 in params_sev.m\n');
end
if ok && abs(vpv - p.mppt.Vref) > 1.0
    ok = false; fprintf('WARN  CV loop not settled at Vref = %.1f V (V_pv = %.2f V).\n', p.mppt.Vref, vpv);
end
% Compare Battery block SOC with the coulomb counter
try
    b = R.batt;  if isstruct(b), fn = fieldnames(b); b = b.(fn{1}); end
    s = b.Data(:,1);
    fprintf('SOC: battery block %.4f -> %.4f %% | coulomb count %.4f -> %.4f %%\n', s(2), s(end), soc0, soc1);
catch e
    fprintf('Note: could not read Battery m output (%s)\n', e.message);
end
if ok, fprintf('PASS  stage 1 electrical model behaves.\n'); end
