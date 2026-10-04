function [LTR_real, LTR_calc, LTR_Npdc] = func_LTRDiagnosis( ...
        VehiclePara, MPCParameters, VehStateMeasured, ParaHAT, ...
        Y, x_opt, exitflag, AI, Ut, Md_next, Npdc, ay_off_Npdc)
% func_LTRDiagnosis  三路 LTR 对照（仅用于记录/画图，不参与控制）
% -------------------------------------------------------------------------
%   LTR_real : CarSim 四轮垂向载荷算出的真值
%   LTR_calc : 控制器公式代入 **实测** 状态 (a_y, phi, Md) 的当前步值
%   LTR_Npdc : 控制器公式代入 **预测** 第 Npdc 步状态的值，
%              ay_off_Npdc 为 QP 约束使用的轮胎仿射偏置加速度
%   Md_next 保留在调用接口中；当前步必须用实测 ParaHAT.Md，而不是该指令。
%
%   LTR_calc - LTR_real  => 公式误差
%   LTR_Npdc - LTR_calc  => 预测误差
%
% 采用的动态式（对所用 3-DOF 侧倾模型精确，无准静态假设）：
%   LTR = 2/(m*g*tw) * [ kappa_g*a_y + Kt*phi + Md ] ,  kappa_g = m*h_TL - ms*h_S2R
% -------------------------------------------------------------------------

m  = VehiclePara.m;   ms = VehiclePara.ms;  g = VehiclePara.g;
tf = VehiclePara.tf;  Kt = VehiclePara.Kt;
h_TL   = VehiclePara.h_TL;   h_S2R = VehiclePara.h_S2R;
CbF = VehiclePara.CbarF;  CbR = VehiclePara.CbarR;
lf_v = VehiclePara.lf;
lr_v   = VehiclePara.lr;

Nu = MPCParameters.Nu;  Nc = MPCParameters.Nc;
Np = MPCParameters.Np;  Nx = MPCParameters.Nx;  Ny = MPCParameters.Ny;

Vel     = VehStateMeasured.x_dot;
delta_f = VehStateMeasured.delta_f;
Roll    = ParaHAT.Roll;

kappa_g = m*h_TL - ms*h_S2R;
bta     = 2/(m*g*tf);

% a_y 对状态/输入的系数（与 func_Envelope 的 c_ay/d_ay 逐项同式）
cd_f = cos(delta_f);
c_ay = [ (CbF*cd_f + CbR)/(m*Vel), ...
         (CbF*lf_v*cd_f - CbR*lr_v)/(m*Vel), zeros(1,Nx-2) ];
d_ay = [ -CbF*cd_f/m, 0, 0 ];
if Nu == 4, d_ay = [d_ay, 0]; end

%% ---- CarSim 真值 ----
Fz_sum   = ParaHAT.Fz_l1 + ParaHAT.Fz_l2 + ParaHAT.Fz_r1 + ParaHAT.Fz_r2;
LTR_real = ( (ParaHAT.Fz_r1 + ParaHAT.Fz_r2) - (ParaHAT.Fz_l1 + ParaHAT.Fz_l2) ) / max(Fz_sum, 1);

%% ---- 当前步：同一公式代入实测状态 ----
ay_meas  = ( (ParaHAT.Fy_l1 + ParaHAT.Fy_r1)*cos(delta_f) ...
           +  ParaHAT.Fy_l2 + ParaHAT.Fy_r2 ) / m;
LTR_calc = bta*( kappa_g*ay_meas + Kt*Roll + ParaHAT.Md );

%% ---- 预测到第 Npdc 步 ----
if nargin < 11 || isempty(Npdc), Npdc = 4; end
if nargin < 12 || isempty(ay_off_Npdc), ay_off_Npdc = 0; end
Npdc   = max(1, round(Npdc));
Npdc_y = min(Npdc, Np);          % 状态序列最多 Np 步
Npdc_u = min(Npdc, Nc);          % 控制序列最多 Nc 步（超出保持最后一步）

if exitflag == 1
    U_pred_Nc = Ut + AI * x_opt(1:Nu*Nc);
else
    U_pred_Nc = Ut;              % 求解失败：保持上一控制
end

by       = Ny*(Npdc_y-1);
xi_Npdc  = Y(by+1 : by+Nx);           % [vy; r; phi; phidot; ey; epsi; Md_a]
phi_Npdc = xi_Npdc(3);

bu       = (Npdc_u-1)*Nu;
u_Npdc   = U_pred_Nc(bu+1 : bu+Nu);   % [Fyf; MFx; Md]
Md_Npdc  = u_Npdc(3);

ay_Npdc  = c_ay*xi_Npdc + d_ay*u_Npdc + ay_off_Npdc;
% Delay-aware predictors use the established aggregate moment. The six-state
% MPC deliberately has no actuator-delay state, so its no-delay command is used.
if Nx >= 7
    Md_Npdc = xi_Npdc(7);
else
    Md_Npdc = u_Npdc(3);
end
LTR_Npdc = bta*( kappa_g*ay_Npdc + Kt*phi_Npdc + Md_Npdc );

end
