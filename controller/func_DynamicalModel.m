function [StateSpaceModel] = func_DynamicalModel(VehiclePara, MPCParameters, VehStateMeasured, Mode, LTV)
% func_DynamicalModel
% -------------------------------------------------------------------------
% 生成离散增广状态空间模型。支持两种线性化方案:
%
%   方案二 (不给 LTV): 当前工作点线性化一次, 整个预测时域冻结。
%   方案三 (给 LTV):   沿名义预测轨迹逐节点线性化 (LTV-MPC)。
%                      每个节点用自己的 alpha 查轮胎表, 得到该节点的
%                      c_bar / f_bar, 因而每个节点有不同的 A_i,B_i,D_i。
%
% 输出 StateSpaceModel:
%   .A_aug : {Np x 1} cell, 每步的增广状态矩阵
%   .B_aug : {Np x 1} cell, 每步的增广输入矩阵 (对 DeltaU)
%   .D_aug : {Np x 1} cell, 每步的增广扰动矩阵
%   .C_aug : 输出矩阵
%   .off   : (2 x Np) 每节点仿射偏置 [F_off_f_eff; F_off_r]; 方案二时为空
%
% LTV 结构体字段:
%   .TireF .TireR        二维轮胎表 (func_TireTable 的 build 结果)
%   .delta_robot         机器人/驾驶员前轮转角 (rad)。标量 = 时域内保持常值;
%                        Np x 1 = 逐节点给定(用实测速率外推, 见 cq32019mpc)。
%                        鱼钩下驾驶员以 42.35 deg/s(前轮)运动, 时域 0.6 s 内
%                        漏掉 25 deg, 常值假设会让各节点的 alpha 全错。
%   .Fzf .Fzr            前/后轴载荷 (预测时域内视为常值, 见"已知简化")
%   .kap                 (Np x 1) 道路曲率, 用于推进名义轨迹
%   .x0                  (6 x 1) 当前状态 [Vy r phi phidot ey epsi]
%   .u0                  (3 x 1) 名义初始控制 [Ddelta MFx Md]
%   .dU                  (Nu x Nc) 名义控制增量序列 (热启动移位)
%   .Ctan_floor_frac     切线刚度下限比例
%
% 已知简化 (方案三 v1):
%   1) 车速 Vel 在时域内视为常值 —— 未预测制动引起的减速;
%   2) 轴载 Fzf/Fzr 视为常值 —— 未预测载荷转移;
%   两者都可后续加上, 但 alpha 引起的刚度变化是主导非线性, 先做这一项。
% -------------------------------------------------------------------------

%% -------------------- Vehicle / MPC Parameters -------------------- %%
P.M   = VehiclePara.m;
P.Ms  = VehiclePara.ms;
P.g   = VehiclePara.g;
P.lf  = VehiclePara.lf;
P.lr  = VehiclePara.lr;
P.Ix  = VehiclePara.Ix;
P.Iz  = VehiclePara.Iz;
P.h_S2R = VehiclePara.h_S2R;
P.Kt  = VehiclePara.Kt;

CbF = VehiclePara.CbarF;    % dFy/dalpha 前轴 (<0), 与 f_bar 同源
CbR = VehiclePara.CbarR;    % dFy/dalpha 后轴 (<0)

Vel     = VehStateMeasured.x_dot;
delta_f = VehStateMeasured.delta_f;

Np = MPCParameters.Np;
Nx = MPCParameters.Nx;
Nu = MPCParameters.Nu;
Ts = MPCParameters.Ts;

Vel_eff = max(abs(Vel), 0.5) * sign(Vel + 1e-6);   % 防止低速除零

C_aug = [eye(Nx) zeros(Nx,Nu)];
StateSpaceModel.C_aug = C_aug;
%  codegen: 同构 cell 改为三维数组 (每个元素尺寸固定, 本来就是同构的)
na_ = Nx + Nu;
StateSpaceModel.A_aug = zeros(na_, na_, Np);
StateSpaceModel.B_aug = zeros(na_, Nu,  Np);
StateSpaceModel.D_aug = zeros(na_, 3,   Np);
%  codegen: off 必须定尺寸(原来 [] 与 2xNp 两种尺寸冲突)。
%  原来调用方用 isempty(off) 区分方案二/三, 改成显式标志 off_valid。
StateSpaceModel.off   = zeros(2, Np);
StateSpaceModel.off_valid = false;

if nargin < 5 || isempty(LTV)
    %% ============ 方案二: 单工作点, 时域内冻结 ============
    [A,B,D] = local_ABD(P, CbF, CbR, Vel_eff, cos(delta_f));
    [A,B,D] = local_delay(A, B, D, MPCParameters.tau_d);   % + Md_a 时延状态
    [~,~,~, Aa,Ba,Da] = local_disc(A,B,D,Ts,Mode,Nx,Nu);
    for i = 1:Np
        StateSpaceModel.A_aug(:,:,i) = Aa;
        StateSpaceModel.B_aug(:,:,i) = Ba;
        StateSpaceModel.D_aug(:,:,i) = Da;
    end
    %  2026-09-26 对照试验: first_Tc = 1 时第一步按执行周期 Ts_exec 离散。
    %  动机: 只有第一步增量被执行, 且只执行 Ts_exec 就重解; 若仍按 Ts 离散,
    %  模型把它的作用高估 Ts/Ts_exec = 5 倍, 疑为 100 Hz 转向锯齿的来源。
    if MPCParameters.first_Tc
        [~,~,~, Aa1,Ba1,Da1] = local_disc(A,B,D,MPCParameters.Ts_exec,Mode,Nx,Nu);
        StateSpaceModel.A_aug(:,:,1) = Aa1;
        StateSpaceModel.B_aug(:,:,1) = Ba1;
        StateSpaceModel.D_aug(:,:,1) = Da1;
    end
    return
end

%% ============ 方案三: 沿名义轨迹逐节点线性化 ============
lf = P.lf;  lr = P.lr;
x   = LTV.x0(:);
u   = LTV.u0(:);
off = zeros(2, Np);
kap = LTV.kap(:);
if numel(kap) < Np, kap = [kap; zeros(Np-numel(kap),1)]; end
nDU = size(LTV.dU, 2);
kf  = LTV.Ctan_floor_frac;
dr  = LTV.delta_robot(:);
if numel(dr) == 1
    dr = repmat(dr, Np, 1);
elseif numel(dr) < Np
    dr = [dr; repmat(dr(end), Np-numel(dr), 1)];
end

for i = 1:Np
    % --- 该节点施加的名义控制 (超出 Nc 后保持) ---
    if i <= nDU
        u = u + LTV.dU(:,i);
    end
    dtot = dr(i) + u(1);                    % 该节点的总前轮转角(驾驶员已外推)

    % --- 该节点的侧偏角 (由名义状态给出) ---
    af = (x(1) + lf*x(2))/Vel_eff - dtot;   % alpha_f
    ar = (x(1) - lr*x(2))/Vel_eff;          % alpha_r

    % --- 查表得该节点的 f_bar / c_bar (同源), 并施加切线刚度下限 ---
    [fbf, cbf] = func_TireTable('eval', LTV.TireF, af, LTV.Fzf);
    [fbr, cbr] = func_TireTable('eval', LTV.TireR, ar, LTV.Fzr);
    cbf = -max(abs(cbf), kf*LTV.TireF.C0);
    cbr = -max(abs(cbr), kf*LTV.TireR.C0);

    % --- 该节点的仿射偏置 (delta_robot 折进前轴偏置, 与方案二同约定) ---
    off(1,i) = fbf - cbf*af - cbf*dr(i);
    off(2,i) = fbr - cbr*ar;

    % --- 该节点的状态空间矩阵 ---
    [A,B,D] = local_ABD(P, cbf, cbr, Vel_eff, cos(dtot));
    [A,B,D] = local_delay(A, B, D, MPCParameters.tau_d);
    [Ad,Bd,Dd, Aa,Ba,Da] = local_disc(A,B,D,Ts,Mode,Nx,Nu);
    StateSpaceModel.A_aug(:,:,i) = Aa;
    StateSpaceModel.B_aug(:,:,i) = Ba;
    StateSpaceModel.D_aug(:,:,i) = Da;

    % --- 推进名义状态到下一节点 ---
    x = Ad*x + Bd*u + Dd*[kap(i); off(1,i); off(2,i)];
end
StateSpaceModel.off = off;
StateSpaceModel.off_valid = true;

end

% =========================================================================
function [A,B,D] = local_ABD(P, CbF, CbR, Ve, cd)
%LOCAL_ABD  连续时间 A/B/D。状态 [Vy r phi phidot ey epsi], 输入 [Ddelta MFx Md]
M=P.M; Ms=P.Ms; g=P.g; lf=P.lf; lr=P.lr; Ix=P.Ix; Iz=P.Iz;
h=P.h_S2R; Kt=P.Kt;

A11 = (CbF*cd + CbR)/(M*Ve);
A12 = (CbF*lf*cd - CbR*lr)/(M*Ve) - Ve;
A21 = (lf*CbF*cd - lr*CbR)/(Iz*Ve);
A22 = (lf^2*CbF*cd + lr^2*CbR)/(Iz*Ve);
A41 = (Ms*h*(CbF*cd + CbR))/(M*Ix*Ve);
A42 = (Ms*h*(CbF*lf*cd - CbR*lr))/(M*Ix*Ve);
A43 = (Ms*g*h - Kt)/Ix;

A = [ A11  A12  0    0  0   0;
      A21  A22  0    0  0   0;
      0    0    0    1  0   0;
      A41  A42  A43  0  0   0;
      1    0    0    0  0   Ve;
      0    1    0    0  0   0 ];

B = [ -CbF*cd/M        0       0;
      -lf*CbF*cd/Iz    1/Iz    0;
       0               0       0;
      -(Ms*h*CbF*cd)/(M*Ix)  0  -1/Ix;
       0               0       0;
       0               0       0 ];

D = [ 0,     cd/M,               1/M;
      0,     lf*cd/Iz,          -lr/Iz;
      0,     0,                  0;
      0,     Ms*h*cd/(M*Ix),     Ms*h/(M*Ix);
      0,     0,                  0;
     -Ve,    0,                  0 ];
end

% =========================================================================
function [Ad,Bd,Dd, Aa,Ba,Da] = local_disc(A,B,D,T,Mode,Nx,Nu)
%LOCAL_DISC  离散化并增广。Ad/Bd/Dd 为非增广(用于推进名义轨迹)。
switch Mode
    case 1   % Euler
        Ad = eye(Nx) + T*A;   Bd = T*B;   Dd = T*D;
        Aa = [Ad Bd; zeros(Nu,Nx) eye(Nu)];
        Ba = [Bd; eye(Nu)];
        Da = [Dd; zeros(Nu,size(D,2))];
    case 2   % Taylor4
        %  幂增量复用: 原写法 A^2/A^3/A^4 各算一次, 后面 A*B,A^2*B,A^3*B 与
        %  A*D,A^2*D,A^3*D 又把幂重算两遍。方案三每拍建 Np=12 组矩阵,
        %  这部分是唯一的耗时增量(实测中位 0.62->3.05 ms), 故值得省。
        %  数学完全等价。
        A2 = A*A;  A3 = A2*A;  A4 = A3*A;
        c2 = T^2/2;  c3 = T^3/6;  c4 = T^4/24;
        Ad = eye(Nx) + T*A + c2*A2 + c3*A3 + c4*A4;
        Bd = T*B + c2*(A*B) + c3*(A2*B) + c4*(A3*B);
        Dd = T*D + c2*(A*D) + c3*(A2*D) + c4*(A3*D);
        Aa = [Ad Bd; zeros(Nu,Nx) eye(Nu)];
        Ba = [Bd; eye(Nu)];
        Da = [Dd; zeros(Nu,size(D,2))];
    case 3   % FOH
        Af  = eye(Nx) + T*A + (T^2/2)*A^2;
        Bf1 = B*T + A*B*T^2/2;
        Bf2 = B*T/2;
        Bf  = Bf1 - Bf2;
        Df  = D*T + A*D*T^2/2;
        Ad = Af;  Bd = Bf + Bf2;  Dd = Df;
        Aa = [Af (Bf + Bf2); zeros(Nu,Nx) eye(Nu)];
        Ba = [(Bf + 2*Bf2); eye(Nu)];
        Da = [Df; zeros(Nu,size(D,2))];
    case 4   % 精确 ZOH (2026-09-25): expm([A B D; 0 0 0]*T)
        %  时延行 -1/tau_d 下 T/tau_d = 3.3, Taylor4 把 e^{-3.3}=0.036 算成 2.19,
        %  时延状态会发散; 只有精确离散能保证 0 < e^{-T/tau} < 1(论文投影论证要用)。
        %  方案二每拍只算一次, 13x13 的 expm 开销可忽略。
        nd_ = size(D,2);
        Mz  = zeros(Nx+Nu+nd_);
        Mz(1:Nx, :) = [A, B, D];
        Ez  = expm(Mz*T);
        Ad = Ez(1:Nx, 1:Nx);
        Bd = Ez(1:Nx, Nx+(1:Nu));
        Dd = Ez(1:Nx, Nx+Nu+(1:nd_));
        Aa = [Ad Bd; zeros(Nu,Nx) eye(Nu)];
        Ba = [Bd; eye(Nu)];
        Da = [Dd; zeros(Nu,nd_)];
    otherwise
        error('func_DynamicalModel: invalid Mode=%d. Use 1(Euler),2(Taylor4),3(FOH),4(ZOH).', Mode);
end
end

% =========================================================================
function [A7,B7,D7] = local_delay(A, B, D, tau)
%LOCAL_DELAY  追加阻尼器时延状态 Md_a (论文 III-A 式 damper_delay / System_delay)
%   tau*dMd_a/dt = -Md_a + Md_c
%   原 B 第 3 列(Md 对侧倾的作用, 只有第 4 行 -1/Ix)挪到 A 的第 7 列: 侧倾只经
%   已建立的 Md_a 受阻尼器作用; 输入 Md_c 只驱动时延状态。
nx = size(A,1);  nd = size(D,2);
tau = max(tau, 1e-6);  % finite fixed-size placeholder when NLCSNN is active
aM = B(:,3);
Bb = B;  Bb(:,3) = 0;
A7 = [A, aM; zeros(1,nx), -1/tau];
B7 = [Bb; 0, 0, 1/tau];
D7 = [D; zeros(1,nd)];
end
