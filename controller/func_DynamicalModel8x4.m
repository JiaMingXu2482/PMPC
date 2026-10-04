function Model = func_DynamicalModel8x4(VehiclePara, P, Measured, Mode, LTV, Vx_schedule) %#codegen
% Eight-state/four-input extension of the established 7x3 vehicle model.
% State 8 is absolute Vx [m/s]; input 4 is total braking force [N] (>0).
% The old lateral/roll/damper matrices remain unchanged. The speed
% coupling terms linearize -Vx*r, Vx*e_psi and -Vx*kappa at each node.
P7 = P;
P7.Nx = 7; P7.Ny = 7; P7.Nu = 3;
if nargin < 5 || isempty(LTV) || ~P.LTV_on
    L7 = [];
else
    L7 = LTV;
    L7.x0 = LTV.x0(1:7);
    L7.u0 = LTV.u0(1:3);
    L7.dU = LTV.dU(1:3,:);
end
if nargin < 6 || isempty(Vx_schedule)
    Base = func_DynamicalModel(VehiclePara,P7,Measured,Mode,L7);
    Vx_schedule = Measured.x_dot*ones(P.Np,1);
else
    Base = func_DynamicalModel(VehiclePara,P7,Measured,Mode,L7,Vx_schedule);
end
Model.C_aug = [eye(8),zeros(8,4)];
Model.A_aug = zeros(12,12,P.Np);
Model.B_aug = zeros(12,4,P.Np);
Model.D_aug = zeros(12,5,P.Np);
Model.off = Base.off;
Model.off_valid = Base.off_valid;
for i = 1:P.Np
    dt = P.Ts;
    if i == 1 && P.first_Tc, dt = P.Ts_exec; end
    v0 = Vx_schedule(i);
    r0 = Measured.Yawrate;
    epsi0 = 0;
    kap = 0;
    if ~isempty(LTV)
        r0 = LTV.x0(2);
        epsi0 = LTV.x0(6);
        if i <= numel(LTV.kap), kap = LTV.kap(i); end
    end
    c = zeros(7,1);
    c(1) = -r0;
    c(5) = epsi0;
    c(6) = -kap;
    Aa = eye(12);
    Aa(1:7,1:7) = Base.A_aug(1:7,1:7,i);
    Aa(1:7,9:11) = Base.A_aug(1:7,8:10,i);
    Aa(1:7,8) = dt*c;
    Aa(1:7,12) = -0.5*dt^2*c/VehiclePara.m;
    Aa(8,12) = -dt/VehiclePara.m;
    Model.A_aug(:,:,i) = Aa;
    Ba = zeros(12,4);
    Ba(1:7,1:3) = Base.B_aug(1:7,:,i);
    Ba(1:7,4) = -0.5*dt^2*c/VehiclePara.m;
    Ba(8,4) = -dt/VehiclePara.m;
    Ba(9:12,:) = eye(4);
    Model.B_aug(:,:,i) = Ba;
    Da = zeros(12,5);
    Da(1:7,1:3) = Base.D_aug(1:7,:,i);
    Da(1:7,4) = 0.5*dt^2*c;
    Da(1:7,5) = -dt*c*v0;
    Da(8,4) = dt;
    Model.D_aug(:,:,i) = Da;
end
end
