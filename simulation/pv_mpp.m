function [Pmp, Vmp] = pv_mpp(G, T, p)
%PV_MPP  Available maximum power of the PV string for irradiance G [W/m^2]
%   and cell temperature T [degC] (vectors). Single-diode model with the same
%   parameters as the PV Array blocks. Exact at 25 degC; temperature scaling
%   is the standard De Soto form, so treat T ~= 25 degC as approximate.
q = 1.602176634e-19;  k = 1.380649e-23;  Eg = 1.121;  Tr = 298.15;
G = G(:);  T = T(:) + zeros(size(G));
key = [round(G), round(T*10)/10];
[u, ~, idx] = unique(key, 'rows');
Pu = zeros(size(u,1),1);  Vu = Pu;
Vm = linspace(0, 1.2*p.pv.Voc, 600);
for j = 1:size(u,1)
    g = u(j,1);  TK = u(j,2) + 273.15;
    if g <= 0, continue; end
    IL = g/1000*(p.pv.IL + p.pv.alpha_Isc*(u(j,2) - 25));
    I0 = p.pv.I0*(TK/Tr)^3*exp(q*Eg/(p.pv.nI*k)*(1/Tr - 1/TK));
    Vt = p.pv.nI*k*TK/q*p.pv.Ncell;
    I = IL*ones(size(Vm));
    for it = 1:40   % Newton on I for each module voltage
        e  = exp((Vm + I*p.pv.Rs)/Vt);
        f  = IL - I0*(e - 1) - (Vm + I*p.pv.Rs)/p.pv.Rsh - I;
        df = -I0*e*p.pv.Rs/Vt - p.pv.Rs/p.pv.Rsh - 1;
        I  = I - f./df;
    end
    P = p.pv.Nstr*Vm.*max(I, 0);
    [Pu(j), jm] = max(P);  Vu(j) = p.pv.Nstr*Vm(jm);
end
Pmp = Pu(idx);  Vmp = Vu(idx);
end
