function [ic, iin] = conv_avg(D, vin, vb, p)
%CONV_AVG  Averaged asynchronous buck (diode-rectified, no reverse current).
%   ic  = output (battery-side) current, iin = D*ic drawn from the PV node.
%   Losses: freewheel diode Vf*(1-D)*ic + conduction Req*ic^2.
%   The first-order lag p.conv.tau stands in for the inductor so the model
%   runs at 1e-4..1e-3 s steps (energy studies, not switching ripple).
%#codegen
persistent x
if isempty(x), x = 0; end
tgt = (D*vin - (1-D)*p.conv.Vf - vb)/p.conv.Req;
tgt = max(tgt, 0);
x = x + (tgt - x)*min(1, p.sim.Ts/p.conv.tau);
x = min(max(x, 0), p.conv.Imax);
ic  = x;
iin = D*x;
end
