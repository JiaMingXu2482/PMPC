function [params, Reftraj, state0, meas0] = pmpc_init()
%PMPC_INIT 共享参数、全局路径及控制器跨拍状态。只在仿真前执行。
% InitFcn: [params, Reftraj, state0, meas0] = pmpc_init();
% params/Reftraj 在控制块中声明为 Parameter；state0 用作 0.01 s Unit Delay 初值。
%% 1. MPC预测模型参数
M.Ts = 0.1;                     % 预测步长 [s]
M.Np = 10;                      % 预测步数
M.Nc = 5;                       % 控制步数
M.Ts_exec = 0.01;               % 执行周期，与 Simulink 的 Unit Delay 一致

%% 2.制动系统响参数
M.tau_mz = 0.2;                 % 横摆力矩全量程建立时间 [s]

%% 3.悬架系统响应参数
M.tau_d = 0.01;                % 命令到实际 Md 的等效一阶时间常数 [s]，待阶跃辨识 2.2tau=35ms
M.tau_md = 0.1;                 % Md 命令全范围变化时间 [s]；独立于实际响应 tau_d
% 预瞄力界参数：L 是取运动状态的时间偏移，不再叠加为动力学纯延迟。
M.preview_L = 0.03;             % [s] 暂定生效时刻，未辨识；范围 0 <= L <= Ts
M.preview_alpha = 0.5;          % 当前/预瞄力区间混合：0 关闭，0.5 折中，1 全预瞄
M.bounds_keep = 1.0;            % 区间半宽保留比例；1 不另加未经标定的收缩

%% 2. 车辆参数
V.m = 1860;                     % 整车质量 [kg]
V.ms = 1590;                    % 簧载质量 [kg]
V.Iz = 3438;                    % 横摆惯量 [kg*m^2]
V.Ix = 1038;                    % 侧倾惯量 [kg*m^2]
V.h_TL = 0.671694;              % 整车质心高度 [m]
V.h_SL = 0.720;                 % 簧载质心高度 [m]
V.L = 2.95;                     % 轴距 [m]
V.lf = 1.246613;                % 质心到前轴 [m]
V.lr = V.L - V.lf;              % 质心到后轴 [m]
V.tf = 1.575; V.tr = 1.575;     % 前、后轮距 [m]
V.lsf = 1.2; V.lsr = 1.2;       % 弹簧横向间距 [m]
V.ldf = 1.1; V.ldr = 1.1;       % 减振器横向间距 [m]
V.rt = 0.393;                   % 车轮半径 [m]

% 转向系统参数
V.isw = 18.57;                  % 转向传动比
V.delta_sw_max = deg2rad(720);  % 最大方向盘转角 [rad]
V.delta_sw_rate_max =...
                deg2rad(720);   % 最大方向盘角速度 [rad/s]
V.g = 9.80665;                  % 重力加速度 [m/s^2]


% ==================== 侧倾中心参数 ====================
V.h_RC_f = 0.09372;             % 前轴侧倾中心高度 [m]
V.h_RC_r = 0.09050;             % 后轴侧倾中心高度 [m]
V.h_RC = (V.lr/V.L)*V.h_RC_f + (V.lf/V.L)*V.h_RC_r; % 整车等效侧倾中心高度  0.0924m
V.h_S2R = V.h_SL - V.h_RC;      % 簧载质心到侧倾中心距离 [m]                0.6276m

% ==================== 悬架弹簧参数 ====================
V.kf = 82e3;                    % 前轴弹簧刚度 [N/m]
V.kr = 45e3;                    % 后轴弹簧刚度 [N/m]
V.rs_f = 0.6111;                % 前轴弹簧运动比
V.rs_r = 0.7511;                % 后轴弹簧运动比

% ==================== 减振器运动比 ====================
V.rd_f = 0.614;                 % 前轴减振器运动比
V.rd_r = 0.754;                 % 后轴减振器运动比
% 最新 eclass2.rtf：前 .614、后 .754；四角 [L1;L2;R1;R2]。
% 小角度、簧下侧倾变化忽略时，压缩速度对车身侧倾角速度的几何系数 [m/rad]。
% 使用轮距乘运动比，不再重复乘减振器安装间距；后续可用实测回归替换。
V.kv_roll = .5*[-V.rd_f*V.tf;-V.rd_r*V.tr;V.rd_f*V.tf;V.rd_r*V.tr];

% ==================== 辅助侧倾刚度 ====================
V.Kaux_f = 384 * 180 / pi;      % 前轴 [N*m/rad]
V.Kaux_r = 384 * 180 / pi;      % 后轴 [N*m/rad]

% ==================== 弹簧侧倾刚度 ====================
Kf_spring = 0.5 * V.kf * V.rs_f^2 * V.lsf^2;
Kr_spring = 0.5 * V.kr * V.rs_r^2 * V.lsr^2;

% ==================== 悬架总侧倾刚度 ====================
Kf_susp = Kf_spring + V.Kaux_f;
Kr_susp = Kr_spring + V.Kaux_r;

% ==================== 轮胎侧倾刚度 ====================
V.kw = 502e3;                   % 轮胎垂向刚度 [N/m]
Ktf = 0.5 * V.kw * V.tf^2;
Ktr = 0.5 * V.kw * V.tr^2;

% ==================== 整车等效侧倾刚度 ====================
V.Kt = 1/(1/Kf_susp + 1/Ktf) + 1/(1/Kr_susp + 1/Ktr);                      %78972

V.k1 = 14e3; V.k2 = 1.23e3;
V.b = 1916; V.k3 = 8.3e2; V.v0 = 0.15;
V.k_cs = 0;                    % 转向柔性默认关闭 [rad/N]
V.CafHat = -57772.51 * 2;      % 前轴名义侧偏刚度 [N/rad]
V.CarHat = -53484.51 * 2;      % 后轴名义侧偏刚度 [N/rad]
V.CbarF = 0; V.CbarR = 0;      % 在线更新的轮胎切线刚度
V.CafTan = 0; V.CarTan = 0;

%% 3. 道路共享参数
Road.mu = 0.5;                  % 路面附着系数：须与 CarSim 一致
Road.width = 3.5;               % 道路/车道宽度 [m]
V.mu = Road.mu;                 % 保持原车辆动力学字段
V.mu_eff = 0.855 * Road.mu;     % CarSim 轮胎有效附着标定

%% 4. 轮胎参数
% 不依赖 MAT 文件，不调用旧轮胎拟合函数。
TireF.mu = Road.mu;
TireF.cq = [-2.3214907389095473e-10, ...
            -9.2937263513579211e-05, ...
             1.3943365770101135e+01];
TireF.mq = Road.mu * [-1.9058537070743386e-11, ...
                       -3.3810642361284200e-06, ...
                        8.9580790781890640e-01];
TireF.fzlo = 5638.82;
TireF.fzhi = 45110.6;
TireF.kcs = V.k_cs;
fzs = linspace(TireF.fzlo,TireF.fzhi,200)';
Cs = fzs .* polyval(TireF.cq,fzs);
TireF.C0 = max(Cs ./ (1 + TireF.kcs*Cs));
TireR = TireF;                   % 前后轴采用相同基础轮胎标定

%% 5. 参考轨迹
% 仅决定使用哪条参考路径，不改变 PMPC 算法。
path_type = 1;                  % 1=DLC；2=Slalom；3=U-turn

switch path_type
    case 1                       % DLC，保留原拟合曲线
        X = (0:0.5:200)';
        Y = 3.5564696255156258/2 * ...
            (1+tanh(4/27.85220278516676*(X-80.058689040522395))) ...
            - 3.5564696255156258/2 * ...
            (1+tanh(4/24.007795423319223*(X-132.35698740891445)));
        station = [];

    case 2                       % Slalom，使用原 CarSim X-Y-Station 数据
        root = fileparts(mfilename('fullpath'));
        pathname = fullfile(root,'Slalom_CarSim_XYS.txt');
        if ~isfile(pathname)
            pathname = fullfile(fileparts(root),'data','Slalom_CarSim_XYS.txt');
        end
        assert(isfile(pathname), ...
            'Slalom 数据不存在：请将 Slalom_CarSim_XYS.txt 放在本文件旁或 data 文件夹');
        xys = readmatrix(pathname,'Delimiter',',');
        assert(size(xys,2)==3 && size(xys,1)>2 && ...
            all(isfinite(xys(:))) && all(diff(xys(:,3))>0), ...
            'Slalom X-Y-Station 数据无效');
        X = xys(:,1);
        Y = xys(:,2);
        station = xys(:,3);

    case 3                       % U-turn：直线20 m + R60 m半圆 + 出弯直线60 m
        ds = 0.25;
        R = 60;
        X1 = (0:ds:20)';
        Y1 = zeros(size(X1));
        theta = linspace(-pi/2,pi/2,ceil(pi*R/ds)+1)';
        X2 = 20 + R*cos(theta);
        Y2 = R + R*sin(theta);
        X3 = (X2(end):-ds:(X2(end)-60))';
        Y3 = Y2(end)*ones(size(X3));
        X = [X1; X2(2:end); X3(2:end)];
        Y = [Y1; Y2(2:end); Y3(2:end)];
        station = [];

    otherwise
        error('path_type 只能为 1(DLC)、2(Slalom) 或 3(U-turn)');
end

% 通用路径格式：[index, X, Y, heading, curvature, 0, station, 0, 0]
heading = atan2(gradient(Y),gradient(X));
if path_type == 1
    heading(X <= 20) = 0;
    Y(X <= 20) = 0;
end
if isempty(station)
    station = [0;cumsum(hypot(diff(X),diff(Y)))];
end
heading_unwrap = unwrap(heading);
curvature = zeros(numel(X),1);
for k = 2:numel(X)-1
    ds = station(k+1)-station(k-1);
    if ds >= 1e-6
        curvature(k) = (heading_unwrap(k+1)-heading_unwrap(k-1))/ds;
    end
end
curvature(1) = curvature(2);
curvature(end) = curvature(end-1);

Reftraj = zeros(numel(X),9);
Reftraj(:,1) = (1:numel(X))';
Reftraj(:,2:5) = [X,Y,heading,curvature];
Reftraj(:,7) = station;
assert(all(isfinite(Reftraj(:))) && all(diff(station)>0), ...
    '参考轨迹包含无效数据或非递增弧长');

params = struct('M',M,'vehicle',V,'road',Road,'tireF',TireF,'tireR',TireR);
% 决策量：[3*Nc 个控制增量；道路/侧倾松弛；稳定性松弛；DB激活量]。
% u_prev=[delta;Mz;Md]；plan 为备用增量序列；path_index 为路径进度。
% brake_prev/damper_prev 保存四角命令；active_* 保存 QP 活动约束。
% 分配活动集先为四个下界，再为四个上界，均按 L1/L2/R1/R2 排列。
state0 = struct('u_prev',zeros(3,1),'plan',zeros(3*M.Nc,1), ...
    'path_index',1,'brake_prev',zeros(4,1),'damper_prev',zeros(4,1), ...
    'active_main',false(6*M.Nc+12*M.Np+2*(3*M.Nc+3),1), ...
    'active_brake',false(8,1),'active_damper',false(8,1));
% 测量为 SI 单位，X/Y 为前轴参考点；四角测量及 brake_prev/damper_prev 顺序 [L1;L2;R1;R2]。
meas0 = struct('X',0,'Y',0,'Vx',0,'Vy',0,'yaw',0,'r',0,'phi',0,'p',0, ...
    'delta_front',zeros(2,1),'steering_wheel',0, ...
    'Fx',zeros(4,1),'Fy',zeros(4,1),'Fz',zeros(4,1), ...
    'Fd',zeros(4,1),'Td',zeros(4,1),'vd',zeros(4,1));
end
