function D = mppt_step(v, i, vb, ic, p)
%MPPT_STEP  MPPT + CC/CV charge limiter. Called every electrical step by the
%   MPPT block; updates the duty every p.mppt.Ts using averaged V and I.
%   Buck convention: raising D draws more current -> lowers PV voltage.
%#codegen
persistent Dalg Dlim cnt accV accI Pp Vp Ip dir Vref ocT inOC
if isempty(Dalg)
    Dalg = p.mppt.D0; Dlim = 1; cnt = 0; accV = 0; accI = 0;
    Pp = 0; Vp = 0; Ip = 0; dir = 1; Vref = p.mppt.Vref; ocT = 0; inOC = 1;
end
m = round(p.mppt.mode);
N = max(1, round(p.mppt.Ts/p.sim.Ts));

accV = accV + v;  accI = accI + i;  cnt = cnt + 1;
if cnt >= N
    V = accV/cnt;  I = accI/cnt;  P = V*I;  Tm = cnt*p.sim.Ts;
    accV = 0; accI = 0; cnt = 0;

    switch m
        case 1  % CV: integrate PV-voltage error to a fixed set-point
            Dalg = Dalg + p.mppt.Kcv*Tm*(V - p.mppt.Vref);
        case 2  % Perturb & Observe
            if P < Pp, dir = -dir; end
            Dalg = Dalg + dir*p.mppt.dD;
        case 3  % Incremental Conductance
            dV = V - Vp;  dI = I - Ip;
            if abs(dV) < 1e-3
                if dI > 1e-4, Dalg = Dalg - p.mppt.dD; elseif dI < -1e-4, Dalg = Dalg + p.mppt.dD; end
            else
                g = dI/dV + I/max(V, 1e-3);          % = (dP/dV)/V
                if g > p.mppt.eps, Dalg = Dalg - p.mppt.dD; elseif g < -p.mppt.eps, Dalg = Dalg + p.mppt.dD; end
            end
        case 4  % Fractional Voc: periodic open-circuit sample, then CV to k*Voc
            ocT = ocT + Tm;
            if inOC > 0
                if ocT >= p.mppt.Tocm, Vref = p.mppt.kfocv*V; inOC = 0; ocT = 0; end
            else
                if ocT >= p.mppt.Toc, inOC = 1; ocT = 0; end
                Dalg = Dalg + p.mppt.Kcv*Tm*(V - Vref);
            end
    end
    Dalg = min(max(Dalg, p.mppt.Dmin), p.mppt.Dmax);
    Pp = P; Vp = V; Ip = I;

    % CC/CV limiter (integrates down when Ic > Imax or Vb > Vcv)
    if m ~= 0 && p.chg.enable > 0
        e = min(p.chg.Imax - ic, p.chg.Kv*(p.chg.Vcv - vb));
        Dlim = min(max(Dlim + p.chg.K*Tm*e, 0), 1);
        if Dlim < Dalg, Dalg = Dlim; end      % anti-windup
    end
end

if m == 0
    D = 1;                       % MPPT off: panel tied to battery through the stage
elseif m == 4 && inOC > 0
    D = 0;                       % FOCV open-circuit window
elseif p.chg.enable > 0
    D = min(Dalg, Dlim);
else
    D = Dalg;
end
end
