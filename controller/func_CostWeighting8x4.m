function [Q,R,S,W,V,dFyfmax,dMFxmax,dMdmax,Fyfmax,MFxmax, ...
    Mdmax,Mdmin,Tb_u,Fdu,Fdl,Mdnom,Fxmax,dFxmax] = ...
    func_CostWeighting8x4(P,CW,Constraints,r_ssmax,ParaHAT,VehiclePara,Measured,DamperLimits) %#codegen
% Extend the established 7x3 weights without changing comparator tuning.
P7 = P;
P7.Ny = 7; P7.Nu = 3;
[Q7,R3,S3,W,V,dFyfmax,dMFxmax,dMdmax,Fyfmax,MFxmax, ...
    Mdmax,Mdmin,Tb_u,Fdu,Fdl,Mdnom] = ...
    func_CostWeightingRegulation_QuadSlacks(P7,CW,Constraints,r_ssmax, ...
    ParaHAT,VehiclePara,Measured,DamperLimits);
Fx_by_wheels = (Tb_u.Tb_L1 + Tb_u.Tb_R1 + Tb_u.Tb_L2 + Tb_u.Tb_R2) ...
    / VehiclePara.rt;
Fxmax = max(0,min(VehiclePara.m*Constraints.Long_a_max,Fx_by_wheels));
force_ref = max(VehiclePara.m*Constraints.Long_a_max,1);
dFxmax = force_ref; % release is never slowed when the reachable box shrinks
Q = zeros(8*P.Np);
for i = 1:P.Np
    i7 = (i-1)*7+(1:7);
    i8 = (i-1)*8+(1:8);
    Q(i8(1:7),i8(1:7)) = Q7(i7,i7);
    qf = 1;
    if i == P.Np, qf = CW.Qf_scale; end
    Q(i8(8),i8(8)) = qf*CW.Q8;
end
R = zeros(4*P.Nc); S = zeros(4*P.Nc);
for i = 1:P.Nc
    i3 = (i-1)*3+(1:3);
    i4 = (i-1)*4+(1:4);
    R(i4(1:3),i4(1:3)) = R3(i3,i3);
    S(i4(1:3),i4(1:3)) = S3(i3,i3);
    R(i4(4),i4(4)) = CW.R4/force_ref^2;
    S(i4(4),i4(4)) = CW.S4/force_ref^2;
end
end
