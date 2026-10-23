function build_sev()
%BUILD_SEV  Generate sev_benchmark.slx (stage 1: PV string -> averaged MPPT
%   buck -> 3S battery, with a load-current input). Specialized Power Systems.
%   Edit params_sev.m, not the generated model; re-run this after structural
%   changes. Parameter-only changes do not need a rebuild.

mdl  = 'sev_benchmark';
here = fileparts(mfilename('fullpath'));
p = params_sev();
assignin('base', 'p', p);
assignin('base', 'G_in',     [0 1000; 1 1000]);   % defaults so the model
assignin('base', 'T_in',     [0 25;   1 25]);     % compiles when opened alone
assignin('base', 'Iload_in', [0 0;    1 0]);

if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']);
if isfile(f), delete(f); end
new_system(mdl);
try load_system('sps_lib'); catch, end
try load_system('powerlib'); catch, end
set_param(mdl, 'SolverType','Fixed-step', 'Solver','FixedStepAuto', ...
    'FixedStep','p.sim.Ts', 'StopTime','60');

%% Library candidates (first that works is used)
L.pg   = {'sps_lib/powergui','spspowerguiLib/powergui','powerlib/powergui'};
L.pv   = {'sps_lib/Sources/PV Array','spsPVArrayLib/PV Array','powerlib/Renewables/PV Array'};
L.bat  = {'sps_lib/Sources/Battery','spsBatteryLib/Battery'};
L.ccs  = {'sps_lib/Sources/Controlled Current Source','spsControlledCurrentSourceLib/Controlled Current Source', ...
          'powerlib/Electrical Sources/Controlled Current Source'};
L.vm   = {'sps_lib/Sensors and Measurements/Voltage Measurement','spsVoltageMeasurementLib/Voltage Measurement', ...
          'powerlib/Measurements/Voltage Measurement'};
L.cm   = {'sps_lib/Sensors and Measurements/Current Measurement','spsCurrentMeasurementLib/Current Measurement', ...
          'powerlib/Measurements/Current Measurement'};
L.rlc  = {'sps_lib/Passives/Series RLC Branch','spsSeriesRLCBranchLib/Series RLC Branch', ...
          'powerlib/Elements/Series RLC Branch'};
L.gnd  = {'sps_lib/Utilities/Ground','sps_lib/Passives/Ground','spsGroundLib/Ground','powerlib/Elements/Ground'};
L.mfn  = {'simulink/User-Defined Functions/MATLAB Function'};

%% Power circuit
pg = addb(mdl, L.pg, 'powergui', [20 20 110 60]);
set_param(pg, 'SimulationMode','Discrete', 'SampleTime','p.sim.Ts');

addb(mdl, 'simulink/Sources/From Workspace', 'G', [20 120 110 150]);
set_param([mdl '/G'], 'VariableName','G_in', 'Interpolate','on', 'OutputAfterFinalValue','Holding final value');
addb(mdl, 'simulink/Sources/From Workspace', 'T', [20 190 110 220]);
set_param([mdl '/T'], 'VariableName','T_in', 'Interpolate','on', 'OutputAfterFinalValue','Holding final value');

pv1 = addb(mdl, L.pv, 'PV1', [180 100 260 180]);
pv2 = addb(mdl, L.pv, 'PV2', [180 240 260 320]);
for b = {pv1, pv2}, setPV(b{1}, p); end

cmpv = addb(mdl, L.cm, 'I_pv', [300 340 340 380]);
cin  = addb(mdl, L.rlc, 'C_in', [340 130 370 190]);
set_param(cin, 'BranchType','C', 'Capacitance','p.conv.Cin');
vmpv = addb(mdl, L.vm, 'V_pv', [400 130 430 190]);
ccsi = addb(mdl, L.ccs, 'Buck_in', [470 130 510 190]);
ccso = addb(mdl, L.ccs, 'Buck_out', [760 130 800 190]);
for b = {ccsi, ccso}, trySet(b{1}, 'Initialize','off'); end
vmb  = addb(mdl, L.vm, 'V_bat', [840 130 870 190]);
bat  = addb(mdl, L.bat, 'Battery_3S', [920 120 1000 200]);
set_param(bat, 'BatType','Lithium-Ion');
set_param(bat, 'PresetModel','off');
set_param(bat, 'NomV','p.batt.Vnom', 'NomQ','p.batt.Q', 'MaxQ','p.batt.Q', ...
    'FullV','p.batt.Vfull', 'MinV','p.batt.Vmin', 'Dis_rate','p.batt.Inom', 'R','p.batt.R', ...
    'Normal_OP','p.batt.Qnomv', 'expZone','[p.batt.Vexp p.batt.Qexp]', ...
    'SOC','p.batt.SOC0', 'Batt_Tr','p.batt.Tr');

% Physical wiring
P = @(blk, nm, side, k) pport(blk, nm, side, k);
nodeA = [P(pv1,'+','RConn',1), P(cin,'',  'LConn',1), P(vmpv,'+','LConn',1), P(ccsi,'+','RConn',1)];
nodeB = [P(bat,'+','LConn',1), P(ccso,'+','RConn',1), P(vmb,'+','LConn',1)];
nodeG = [P(cmpv,'-','RConn',1), P(cin,'','RConn',1), P(vmpv,'-','LConn',2), P(ccsi,'-','LConn',1), ...
         P(bat,'-','LConn',2), P(ccso,'-','LConn',1), P(vmb,'-','LConn',2)];
try
    gnd = addb(mdl, L.gnd, 'Ground', [600 420 630 450]);
    nodeG(end+1) = P(gnd, '', 'LConn', 1);
catch
    warning('build_sev:gnd', 'No Ground block found; circuit left floating (usually fine in SPS).');
end
pline(mdl, P(pv1,'-','RConn',2), P(pv2,'+','RConn',1));
pline(mdl, P(pv2,'-','RConn',2), P(cmpv,'+','LConn',1));
pnode(mdl, nodeA); pnode(mdl, nodeB); pnode(mdl, nodeG);

%% Control and signals
add_line(mdl, 'G/1', 'PV1/1', 'autorouting','on'); add_line(mdl, 'G/1', 'PV2/1', 'autorouting','on');
add_line(mdl, 'T/1', 'PV1/2', 'autorouting','on'); add_line(mdl, 'T/1', 'PV2/2', 'autorouting','on');

addb(mdl, 'simulink/Sources/From Workspace', 'I_load', [620 520 710 550]);
set_param([mdl '/I_load'], 'VariableName','Iload_in', 'Interpolate','on', 'OutputAfterFinalValue','Holding final value');

blk = @(lib, nm, pos, varargin) setp(addb(mdl, lib, nm, pos), varargin{:});
blk('simulink/Math Operations/Gain', 'sgn_ipv', [380 420 420 450], 'Gain','p.sgn.ipv');
blk('simulink/Discrete/Unit Delay', 'z_vpv', [380 480 420 510], 'InitialCondition','0', 'SampleTime','p.sim.Ts');
blk('simulink/Discrete/Unit Delay', 'z_vbat', [380 540 420 570], 'InitialCondition','p.batt.Vnom', 'SampleTime','p.sim.Ts');
blk('simulink/Discrete/Unit Delay', 'z_ipv', [430 420 470 450], 'InitialCondition','0', 'SampleTime','p.sim.Ts');
blk('simulink/Discrete/Unit Delay', 'z_ic', [380 600 420 630], 'InitialCondition','0', 'SampleTime','p.sim.Ts');
mfun(mdl, L.mfn, 'MPPT', [470 460 580 620], ...
    'function D = fcn(v_pv, i_pv, v_bat, i_c, p)\nD = mppt_step(v_pv, i_pv, v_bat, i_c, p);\n');
mfun(mdl, L.mfn, 'Buck_avg', [640 460 740 560], ...
    'function [i_c, i_in] = fcn(D, v_in, v_bat, p)\n[i_c, i_in] = conv_avg(D, v_in, v_bat, p);\n');
blk('simulink/Math Operations/Gain', 'sgn_in', [470 60 510 90], 'Gain','-p.sgn.ccs');
blk('simulink/Math Operations/Sum', 'i_net', [780 470 800 510], 'Inputs','+-');
blk('simulink/Math Operations/Gain', 'sgn_out', [740 60 780 90], 'Gain','p.sgn.ccs');
blk('simulink/Discrete/Discrete-Time Integrator', 'SOC_cc', [840 560 880 600], ...
    'gainval','100/(3600*p.batt.Q)', 'InitialCondition','p.batt.SOC0', 'SampleTime','p.sim.Ts');
blk('simulink/Math Operations/Product', 'P_pv', [480 680 510 710]);
blk('simulink/Signal Routing/Mux', 'logmux', [960 400 965 700], 'Inputs','11');
blk('simulink/Sinks/To Workspace', 'simlog', [1000 535 1070 565], 'VariableName','simlog', ...
    'SaveFormat','Structure With Time', 'Decimation','max(1,round(p.sim.Tlog/p.sim.Ts))', 'MaxDataPoints','inf');
blk('simulink/Sinks/To Workspace', 'battlog', [1060 200 1130 230], 'VariableName','battlog', ...
    'SaveFormat','Timeseries', 'Decimation','max(1,round(p.sim.Tlog/p.sim.Ts))', 'MaxDataPoints','inf');

S = @(a, b) add_line(mdl, a, b, 'autorouting','on');
S('I_pv/1', 'sgn_ipv/1');
S('V_pv/1', 'z_vpv/1');   S('V_bat/1', 'z_vbat/1');
S('z_vpv/1', 'MPPT/1');   S('sgn_ipv/1', 'z_ipv/1');  S('z_ipv/1', 'MPPT/2');  S('z_vbat/1', 'MPPT/3');  S('z_ic/1', 'MPPT/4');
S('MPPT/1', 'Buck_avg/1'); S('z_vpv/1', 'Buck_avg/2'); S('z_vbat/1', 'Buck_avg/3');
S('Buck_avg/1', 'z_ic/1');
S('Buck_avg/2', 'sgn_in/1');  S('sgn_in/1', 'Buck_in/1');
S('Buck_avg/1', 'i_net/1');   S('I_load/1', 'i_net/2');
S('i_net/1', 'sgn_out/1');    S('sgn_out/1', 'Buck_out/1');
S('i_net/1', 'SOC_cc/1');
S('V_pv/1', 'P_pv/1');  S('sgn_ipv/1', 'P_pv/2');
S('Battery_3S/1', 'battlog/1');
% log columns: 1 G | 2 T | 3 v_pv | 4 i_pv | 5 p_pv | 6 D | 7 i_c | 8 i_in | 9 v_bat | 10 i_load | 11 SOC_cc
src = {'G/1','T/1','V_pv/1','sgn_ipv/1','P_pv/1','MPPT/1','Buck_avg/1','Buck_avg/2','V_bat/1','I_load/1','SOC_cc/1'};
for k = 1:numel(src), S(src{k}, sprintf('logmux/%d', k)); end
S('logmux/1', 'simlog/1');

try Simulink.BlockDiagram.arrangeSystem(mdl); catch, end
save_system(mdl, f);
fprintf('build_sev: wrote %s\n', f);
end

%% ---------------------------------------------------------------- helpers
function b = addb(mdl, srcs, name, pos)
if ischar(srcs), srcs = {srcs}; end
b = [mdl '/' name];  err = [];
for k = 1:numel(srcs)
    try add_block(srcs{k}, b, 'Position', pos); return; catch err, end
end
error('build_sev:lib', 'Could not add "%s". Tried: %s\nLast error: %s', name, strjoin(srcs, ' | '), err.message);
end

function b = setp(b, varargin)
if ~isempty(varargin), set_param(b, varargin{:}); end
end

function trySet(b, varargin)
try set_param(b, varargin{:}); catch e, warning('build_sev:param', '%s: %s', b, e.message); end
end

function setPV(b, p)
set_param(b, 'ModuleName','User-defined');
v = @(x) sprintf('%.10g', x);
set_param(b, 'Npar','1', 'Nser','1', 'Pm',v(p.pv.Pm), 'Ncell',v(p.pv.Ncell), 'Voc',v(p.pv.Voc), ...
    'Isc',v(p.pv.Isc), 'Vm',v(p.pv.Vm), 'Im',v(p.pv.Im), ...
    'beta_Voc_pc',v(p.pv.beta_Voc_pc), 'alpha_Isc_pc',v(p.pv.alpha_Isc_pc));
trySet(b, 'IL',v(p.pv.IL), 'I0',v(p.pv.I0), 'nI',v(p.pv.nI), 'Rsh',v(p.pv.Rsh), 'Rs',v(p.pv.Rs));
end

function mfun(mdl, lib, name, pos, code)
addb(mdl, lib, name, pos);
rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart', 'Path',[mdl '/' name]);
ch.Script = sprintf(code);
d = ch.find('-isa','Stateflow.Data', 'Name','p');
d.Scope = 'Parameter';
end

function h = pport(blk, name, side, k)
% Physical port handle, by port label when possible, else by side/index.
ph = get_param(blk, 'PortHandles');
if ~isempty(name)
    pm = find_system(blk, 'LookUnderMasks','all', 'FollowLinks','on', 'SearchDepth',1, 'BlockType','PMIOPort');
    for s = {'LConn','RConn'}
        sel = {};  num = [];
        for j = 1:numel(pm)
            sd = get_param(pm{j}, 'Side');
            if (strcmp(s{1}, 'LConn') && strcmpi(sd, 'Left')) || (strcmp(s{1}, 'RConn') && strcmpi(sd, 'Right'))
                sel{end+1} = pm{j}; num(end+1) = str2double(get_param(pm{j}, 'Port')); %#ok<AGROW>
            end
        end
        [~, o] = sort(num);  sel = sel(o);
        for j = 1:numel(sel)
            if strcmp(strtrim(get_param(sel{j}, 'Name')), name)
                h = ph.(s{1})(j);
                fprintf('  port %-22s %-2s -> %s(%d) [by label]\n', get_param(blk,'Name'), name, s{1}, j);
                return
            end
        end
    end
end
h = ph.(side)(k);
if ~isempty(name)
    fprintf('  port %-22s %-2s -> %s(%d) [by position - check]\n', get_param(blk,'Name'), name, side, k);
end
end

function pline(mdl, a, b)
add_line(mdl, a, b, 'autorouting','on');
end

function pnode(mdl, hs)
for j = 2:numel(hs), pline(mdl, hs(1), hs(j)); end
end
