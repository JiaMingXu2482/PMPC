function [params, Reftraj, state0, meas0] = pmpc_base_init(maneuver, horizon, opts)
%PMPC_BASE_INIT 基线 MPC 的参数、参考轨迹与初始状态（独立于 pmpc_core）。
%
%   [P,R,s0,m0] = pmpc_base_init(maneuver)                 % 默认 DLC
%   [P,R,s0,m0] = pmpc_base_init(2, struct('Ts',0.05,'Np',12,'Nc',6))
%
% 与协调控制器（pmpc_core/pmpc_init）的区别：
%   状态  6 维 [Vy; r; phi; p; ey; epsi]，含侧倾，但不含实际 Md
%   控制  2 维 [delta; Mz]，不含 Md 命令
%   减振器固定电流 cdc_fixed_I，不是决策量
%   没有 Md 状态 -> 没有 tau_d 时滞建模（不是把时滞关掉，是这条链路根本不存在）
% 侧倾自由度（phi/p）、LTR 约束（|LTR|<=0.8）、Zeng 稳定域、道路包络
% 与协调控制器完全一致，被控对象是同一个模型。
%
% maneuver: 1=DLC(60/70/75/80) 2=Slalom(80) 3=J-turn 4=FishHook(80)
% horizon : 可选 struct('Ts',..,'Np',..,'Nc',..)；默认来自 case
% opts    : 可选 struct，覆盖 M 字段（如 .Q_norm .R_norm .slack_w）
%
% 输出 Reftraj 与 pmpc_core 同格式：[index, X, Y, heading, curvature, 0, station]
% 状态/测量 bus 由 pmpc_base_bus 定义。

if nargin<1 || isempty(maneuver), maneuver = 1; end
if nargin<2 || isempty(horizon),  horizon = struct(); end
if nargin<3 || isempty(opts),     opts = struct(); end

%% 1. 采样与控制时域
% 与 pmpc_core/pmpc_init 完全一致：Ts=0.1、Np=10、Nc=5（预测 1.0 s、控制 0.5 s）。
% 只有时域相同，两个控制器才是同一目标函数下的对照。
M.Ts      = 0.1;     % 预测步长 [s]
M.Np      = 10;      % 预测步数（1.0 s）
M.Nc      = 5;       % 控制步数（0.5 s）
M.Ts_exec = 0.01;    % 实际执行周期 [s]（100 Hz 控制）
if isfield(horizon,'Ts'),      M.Ts      = horizon.Ts;      end
if isfield(horizon,'Np'),      M.Np      = horizon.Np;      end
if isfield(horizon,'Nc'),      M.Nc      = horizon.Nc;      end
if isfield(horizon,'Ts_exec'), M.Ts_exec = horizon.Ts_exec; end

%% 2. 控制器参数
% 代价权重：状态 [Vy; r; phi; p; ey; epsi]；控制增量 [dDelta; dMz]。
M.Q_norm  = [0; 0; 0; 0; 50; 5];
M.R_norm  = [100; 200];
% 松弛权重：[横摆; 后轴侧偏; LTR; 道路包络]
M.slack_w = [5; 5; 5; 5];

% 命令速率限制的等效时间常数（与协调控制器同口径，便于比较）
M.tau_mz = 0.2;      % 横摆力矩全量程建立时间 [s]（与 pmpc_core 一致）

% 减振器：固定电流，不是决策量
M.cdc_fixed_I = 1.0; % 四角电流 [A]

% AFS / 差动制动开关（FishHook 需要关掉 AFS）
M.afs_on = true;
M.mz_on  = true;
% 外部转向：FishHook 由 CarSim 开环机器人给方向盘，Simulink 只输出 AFS 增量
M.steer_ext = false;

%% 3. 车辆参数（与 pmpc_core/pmpc_init 保持一致）
V.m   = 1830;        V.ms  = 1590;
V.Iz  = 3378.974;    V.Ix  = 1070.178;
V.h_TL= 0.676;       V.h_SL= 0.720;
V.L   = 2.95;        V.lf  = 1.219;   V.lr = V.L - V.lf;
V.tf  = 1.575;       V.tr  = 1.575;
V.rt  = 0.393;       V.g   = 9.807;
V.h_RC_f = 0.09372;  V.h_RC_r = 0.09050;
V.h_RC   = (V.lr/V.L)*V.h_RC_f + (V.lf/V.L)*V.h_RC_r;
V.h_S2R  = V.h_SL - V.h_RC;
V.isw    = 18.57;
V.delta_sw_max      = deg2rad(720);
V.delta_sw_rate_max = deg2rad(540);
V.kf = 82e3;  V.kr = 45e3;  V.rs_f = 0.6111;  V.rs_r = 0.7511;
V.rd_f = 0.614;  V.rd_r = 0.754;
V.Kaux_f = 384*180/pi;  V.Kaux_r = 384*180/pi;
V.kw = 502e3;  V.Kt = 81.5e3;
V.cs_fy  = -3.2e-6;
V.CafHat = -145.4e3;  V.CarHat = -103.8e3;
V.CbarF  = 0;  V.CbarR = 0;

%% 4. 工况
Road.width = 3.5;
switch maneuver
    case 1                       % DLC
        Road.mu  = 0.4;
        M.Q_norm = [0; 0; 10; 0; 50; 20];
        % R_norm(1): dDelta 的代价。100 时 DLC 80 会把方向盘打到 720° 机械极限
        % （ey 也跑到 2.0 m）；300 时 SW_max 228°、ey 0.875。LTR 在这三个算例里
        % 只有 0.34，从来没顶到 0.8，所以调 slack 对 SW 完全无效，只能动这里。
        M.R_norm = [300; 200];
        M.slack_w= 0.2e4*ones(4,1);
        X = (0:0.5:200)';
        Y = 3.556/2*(1+tanh(4/27.852*(X-80.059))) ...
          - 3.556/2*(1+tanh(4/24.008*(X-132.357)));
        station = [];

    case 2                       % Slalom（A=1.2, k=0.18, mu=0.85）
        Road.mu  = 0.85;
        M.Q_norm = [0; 0; 5; 0; 50; 5];
        M.R_norm = [0.1; 500];
        % slack_w = [r; alpha; LTR; env]。LTR 项原来只有 5："|LTR|<=0.8" 的松弛
        % 太便宜，约束形同虚设，实测 LTR 冲到 1.000（内侧轮离地）。
        % 扫到 1e3 时 LTR_max 0.828、ey 反而从 0.894 降到 0.834；1e4 与 1e3 结果
        % 完全一致（约束已等效为硬约束），所以取最小的 1e3。
        M.slack_w= [5; 5; 1e3; 5];
        A  = 1.2; k = 0.18; Lr = 20; Lo = 40;
        P  = 2*pi*A/k;
        Lx = Lr + 3*P + Lo;
        n  = ceil(Lx/0.5);
        X  = (0:n)'*0.5;  X(end) = Lx;
        Y  = k*(X - Lr);
        i2 = X > Lr & X <= Lr + 3*P;
        Y(i2) = A*sin(2*pi*(X(i2)-Lr)/P);
        i3 = X > Lr + 3*P;
        Y(i3) = k*(X(i3) - Lr - 3*P);
        station = [];

    case 3                       % J-turn
        Road.mu  = 0.85;
        M.Q_norm = [0; 0; 5; 0; 50; 5];
        M.R_norm = [0.1; 500];
        M.slack_w= [5; 5; 5; 5];
        ds = 0.25; L1 = 60; R = 70;
        a  = linspace(0,pi/2,round(pi/2*R/ds))';
        X  = [linspace(0,L1,round(L1/ds))'; L1+R*sin(a); L1+R+linspace(0,L1,round(L1/ds))'];
        Y  = [zeros(round(L1/ds),1);    R*(1-cos(a));  R*ones(round(L1/ds),1)];
        station = [];

    case 4                       % FishHook（无路径权限）
        Road.mu  = 0.9;
        M.afs_on = false;            % 转向由 CarSim 机器人给
        M.steer_ext = true;
        M.Q_norm = [0; 10; 20; 0; 0; 0];  % 无路径跟踪，只压横摆与侧倾
        M.R_norm = [100; 500];
        M.slack_w= [100; 100; 1e4; 1e-3];
        X = (-50:0.5:400)';
        Y = zeros(size(X));
        station = [];

    otherwise
        error('maneuver 只能为 1(DLC)、2(Slalom)、3(J-turn)、4(FishHook)');
end
V.mu = Road.mu;

%% 5. 轮胎参数
TireF.mu   = Road.mu;
TireF.cq   = [-2.321e-10, -9.294e-05, 1.394e+01];
TireF.mq   = Road.mu * [-1.906e-11, -3.381e-06, 8.958e-01];
TireF.fzlo = 5638.82;
TireF.fzhi = 45110.6;
TireF.kcs  = 0;
cq0 = TireF.cq;
TireF.cq = cq0*1.029;
TireR    = TireF;
TireR.cq = cq0*1.044;

%% 6. opts 覆盖
% 只覆盖 M 里已存在的数值/逻辑字段。opts 里可能夹带 prefix、stopTime 之类的
% 运行控制项；字符串/元胞一旦进了 params 结构体，Simulink 就无法把它映射成
% 运行时参数（报“不受支持的变量类”）。
flds = fieldnames(opts);
for i = 1:numel(flds)
    if ~isfield(M, flds{i}), continue; end
    v = opts.(flds{i});
    if ischar(v) || isstring(v) || iscell(v), continue; end
    M.(flds{i}) = v;
end

%% 7. 参考轨迹
heading   = atan2(gradient(Y), gradient(X));
curvature = zeros(numel(X),1);
for k = 2:numel(X)-1
    dpsi = mod(heading(k+1)-heading(k-1)+pi, 2*pi) - pi;
    ds   = hypot(X(k+1)-X(k-1), Y(k+1)-Y(k-1));
    curvature(k) = dpsi/max(ds,1e-6);
end
if maneuver == 1
    heading(X <= 20)   = 0;
    Y(X <= 20)         = 0;
    curvature(X <= 20) = 0;
end
if isempty(station)
    station = [0; cumsum(hypot(diff(X), diff(Y)))];
end
Reftraj = [(1:numel(X))', X, Y, unwrap(heading), curvature, ...
           zeros(numel(X),1), station];

%% 8. 参数结构体
params = struct('M', M, 'vehicle', V, 'road', Road, ...
                'tireF', TireF, 'tireR', TireR);

%% 9. 状态初值
% u_prev = [Delta delta; Mz]；plan 为备用增量序列（2*Nc）
% active_main 长度 = 4*Nc(控制) + 10*Np(状态) + 2*(2*Nc+4)(松弛上下界)
state0 = struct( ...
    'u_prev',       zeros(2,1), ...
    'plan',         zeros(2*M.Nc,1), ...
    'path_index',   1, ...
    'sw_prev',      0, ...
    'brake_prev',   zeros(4,1), ...
    'active_main',  false(8*M.Nc + 10*M.Np + 8,1), ...
    'active_brake', false(8,1));

%% 10. 测量初值（与 pmpc_core 同结构，四角顺序 [L1;L2;R1;R2]）
meas0 = struct('X',0,'Y',0,'Vx',0,'Vy',0,'yaw',0,'r',0,'phi',0,'p',0, ...
    'delta_front',zeros(2,1),'steering_wheel',0, ...
    'Fx',zeros(4,1),'Fy',zeros(4,1),'Fz',zeros(4,1),'Fd',zeros(4,1), ...
    'Td',zeros(4,1),'vd',zeros(4,1));
end
