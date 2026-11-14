function [params, Reftraj, state0, meas0] = pmpc_init(maneuver_override, controller_override, horizon_override, mu_override)
%PMPC_INIT 共享参数、全局路径及控制器跨拍状态。只在仿真前执行。
% InitFcn: [params, Reftraj, state0, meas0] = pmpc_init();
% 可选 maneuver_override（工况）：1=DLC，2=Slalom，3=U-turn，4=FishHook；空/省略时默认1。
% 可选 controller_override：1=MPC，2=ZENG，3=Proposed；空/省略时用 M.controller。
% 可选 horizon_override：struct('Ts',...,'Np',...,'Nc',...)，用于瞬态工况缩短/加密时域。
% params/Reftraj 在控制块中声明为 Parameter；state0 用作 0.01 s Unit Delay 初值。

%% 1. MPC预测模型参数
M.Ts = 0.1;                     % 预测步长 [s]
M.Np = 10;                      % 预测步数
M.Nc = 5;                       % 控制步数
M.Ts_exec = 0.01;               % 执行周期

% 瞬态工况时域覆盖（FishHook 等）；state0/总线尺寸会随 Np/Nc 重新生成。
if nargin >= 3 && ~isempty(horizon_override)
    if isfield(horizon_override,'Ts'), M.Ts = horizon_override.Ts; end
    if isfield(horizon_override,'Np'), M.Np = horizon_override.Np; end
    if isfield(horizon_override,'Nc'), M.Nc = horizon_override.Nc; end
end

%% 2. 制动系统响应参数
M.tau_mz = 0.2;                 % 横摆力矩全量程建立时间 [s]

%% 3. 悬架系统响应参数
M.tau_d = 0.01;                 % 命令到实际 Md 的等效一阶时间常数 [s]
M.tau_md = 0.1;                 % Md 命令全范围变化时间 [s]
M.tau_lead = 0;                 % CDC 时滞补偿的超前时间 [s]，0=不补偿
% CDC 执行器实现：1=连续（逆模型跟踪 Fdes）；2=SH 开关（按阻尼力/车身速度符号给 0/1.6 A）
M.cdc_mode = 1;
% 预瞄力界参数：L 是取运动状态的时间偏移，不再叠加为动力学纯延迟。
M.preview_L = 0.03;             % [s] 暂定生效时刻，未辨识；范围 0 <= L <= Ts
M.preview_alpha = 1;          % 当前/预瞄力区间混合：0 关闭，0.5 折中，1 全预瞄
M.bounds_keep = 1.0;            % 区间半宽保留比例；1 不另加未经标定的收缩

%% 4. 车辆参数
V.m = 1830;                     % 整车质量 [kg]（CarSim eclass2 M_TL）
V.ms = 1590;                    % 簧载质量 [kg]
V.Iz = 3378.974;                % 横摆惯量 [kg*m^2]（CarSim IZZ_TL）
V.Ix = 1070.178;                % 侧倾惯量 [kg*m^2]（CarSim IXX_TL）
V.h_TL = 0.676;                 % 整车质心高度 [m]（CarSim H_CG_TL）
V.h_SL = 0.720;                 % 簧载质心高度 [m]
V.L = 2.95;                     % 轴距 [m]
V.lf = 1.219;                   % 质心到前轴 [m]（CarSim LX_CG_TL）
V.lr = V.L - V.lf;              % 质心到后轴 [m]
V.tf = 1.575; V.tr = 1.575;     % 前、后轮距 [m]
V.lsf = 1.2; V.lsr = 1.2;       % 弹簧横向间距 [m]
V.ldf = 1.1; V.ldr = 1.1;       % 减振器横向间距 [m]
V.rt = 0.393;                   % 车轮半径 [m]
V.g = 9.807;                    % 重力加速度 [m/s^2]

% ==================== 侧倾中心参数 ====================
V.h_RC_f = 0.09372;             % 前轴侧倾中心高度 [m]
V.h_RC_r = 0.09050;             % 后轴侧倾中心高度 [m]
V.h_RC = (V.lr/V.L)*V.h_RC_f +...
        (V.lf/V.L)*V.h_RC_r;   % 侧倾中心到地面高度 [m]
V.h_S2R = V.h_SL - V.h_RC;      % 簧载质心到侧倾中心距离 [m]

% ==================== 转向系统参数 ====================
V.isw = 18.57;                  % 转向传动比
V.delta_sw_max = deg2rad(720);  % 最大方向盘转角 [rad]
V.delta_sw_rate_max = ...
                deg2rad(540);  % 最大方向盘角速度 [rad/s]

% ==================== 悬架弹簧参数 ====================
V.kf = 82e3;                    % 前轴弹簧刚度 [N/m]
V.kr = 45e3;                    % 后轴弹簧刚度 [N/m]
V.rs_f = 0.6111;                % 前轴弹簧运动比
V.rs_r = 0.7511;                % 后轴弹簧运动比

% ==================== 减振器运动比 ====================
V.rd_f = 0.614;                 % 前轴减振器运动比
V.rd_r = 0.754;                 % 后轴减振器运动比

% 四角顺序：[L1;L2;R1;R2]
% 小角度、簧下侧倾变化忽略时，压缩速度对车身侧倾角速度的几何系数 [m/rad]。
V.kv_roll = .5*[-V.rd_f*V.tf;-V.rd_r*V.tr; ...
                V.rd_f*V.tf; V.rd_r*V.tr];

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
V.Kt = 1/(1/Kf_susp + 1/Ktf) + ...
        1/(1/Kr_susp + 1/Ktr);
% 2026-11-10 稳态圆周辨识：Kt=81.5 kN*m/rad，hs=0.62761 m。
V.Kt = 81.5e3;

V.k_cs = 0;                     % 轮胎侧偏刚度柔度（旧字段，保持关闭）
% 转向系统柔度：delta_actual = delta_cmd + cs_fy*Fyf_axle
V.cs_fy = -3.2e-6;              % [rad/N]，eclass2 CS_FY_COEFFICIENT

% 2026-11-10 轮胎模型幅值校核：kf=1.029、kr=1.044
V.CafHat = -145.4e3;            % 前轴名义侧偏刚度 [N/rad]
V.CarHat = -103.8e3;            % 后轴名义侧偏刚度 [N/rad]
% 在线轮胎切线刚度（当前未做在线更新，保留字段供 Zeng 稳定域使用）
V.CbarF = 0;
V.CbarR = 0;
V.CafTan = 0;
V.CarTan = 0;

% AFS：delta_d 为驾驶员前轮转角，u(1)=Delta delta 为附加角；
M.afs_on = true;                %DLC & Jturn
M.mz_on = true;                 % 差动制动 Mz 是否参与（被动车基线置 false）
% M.afs_on = false;             %FishHook
% 外部开环转向：FishHook 由 CarSim 的转向机器人给 SW，Simulink 只输出
% AFS 增量；此时 delta_d 取实测前轮转角（不再读 Constant 块）。
M.steer_ext = false;

% 控制器：1=MPC（固定权重），2=ZENG 2025（稳定性权重调度），3=Proposed（余量分层松弛）
M.controller = 1;               % 默认 MPC；Proposed 需显式传 3
if nargin>=2 && ~isempty(controller_override)
    M.controller = controller_override;
end

% 三种控制器共用的代价权重（从 0.1 量级起整定）
M.Q_norm  = [0; 0; 5; 0; 50; 5; 0];   % [Vy;r;phi;p;ey;epsi;Md]
M.R_norm  = [80; 500; 0.1];           % [dDelta;dMz;dMd]
M.slack_w = [5; 5; 5; 5];             % [r;alpha;LTR;env]
M.rho_I0 = 0.7;                       % ZENG rho 起始阈值（论文 0.7）
M.rho_dI = 0.3;                       % ZENG rho 过渡宽度（论文 0.3）
M.Wb = 100;                           % Proposed: DB 激活量 gamma 的线性罚
M.w2_max = 2e3;                       % Proposed: 稳定性松弛权重上限（整定）
M.w1_max = 2e3;                       % Proposed: 道路/侧倾松弛权重上限（整定）
% 增益 g 已固定为 1（写死在 pmpc_step 里），不再单独整定。
% 余量取法（消融用）：1=预测时域内最紧约束（默认）；2=只看第 1 个预测节点。
M.margin_mode = 1;
% CDC 时滞模型：1=常数 M.tau_d；2=按压缩速度时变（标定自力阶跃 t10-90）。
M.tau_d_mode = 1;

%% 5. 参考轨迹
% 仅决定使用哪条参考路径，不改变 PMPC 算法。
maneuver = 1;                  % 工况：1=DLC；2=Slalom；3=U-turn；4=FishHook
if nargin>=1 && ~isempty(maneuver_override)
    maneuver = maneuver_override;
end

switch maneuver

    case 1                       % DLC（60/70/75 km/h 对比，原 tanh 路径）
        Road.mu = 0.4;           % CarSim DLC 路面 friction map：Mu = 0.4 Test Pad
        % DLC 专用权重
        % ey 权重 100 -> 50（2026-11-12，减振器力 x3）：80 km/h 已超附着极限，
        % 死追 ey 会把方向盘打满然后滑出去；降到 50 后 MPC/ZENG/Proposed
        % 的 80 km/h 结果分别为 8.27 / 2.46 / 0.87 m。
        M.Q_norm  = [0; 0; 10; 0; 50; 20; 0];
        M.R_norm  = [100; 200; 0.1];   % 60 km/h: R1=30; 80 km/h 建议改回 10
        % MPC 保持原始固定权重 1e4（mu=0.4、75 km/h 以上会失效）；
        % ZENG/Proposed 用 mu=0.4 重整定值 0.2e4。
        M.slack_w = 0.2e4*ones(4,1);
        if M.controller==1
            % MPC 基线整定（2026-11-12，减振器力 x3）：松弛权重 1e4 -> 5e4。
            % 1e4 时 75 km/h 只跑偏 1.62 m（x1 时是 9.07 m，x3 把 MPC 救回来了）；
            % 5e4 时 75 km/h 达 3.22 m、方向盘饱和 720°，恢复"基线失效"的对照。
            % 60 km/h 不受影响（约束不激活），70 km/h ey 0.632->0.625 但 SW 220->398。
            M.slack_w = 5e4*ones(4,1);
        end
        if M.controller==2
            M.Q_norm(6) = 10;          % ZENG DLC 整定：epsi 权重 20 -> 10
            M.rho_I0 = 1.0;            % ZENG DLC 整定：rho 起始阈值 0.7 -> 1.0
        end
        if M.controller==3
            M.Q_norm(6) = 10;          % Proposed DLC 整定：epsi 权重 20 -> 10
            M.Wb = 10;                 % Proposed DLC 整定：DB gamma 罚 100 -> 10
            % 松弛上限（2026-11-12，x3 后重扫）：默认 2e3 -> 1e3。
            % 80 km/h 的 ey 从 2.077 降到 0.870（方向盘从 720 饱和降到 278），
            % 70/75 只退化 1.7-4%（未饱和区，约束过硬反而保守）。
            M.w1_max = 0.1e4;
            M.w2_max = 0.1e4;
        end
        % carsim 标准DLC路径拟合公式
        X = (0:0.5:200)';
        Y = 3.556/2 * (1+tanh(4/27.852*(X-80.059))) ...
            - 3.556/2 * (1+tanh(4/24.008*(X-132.357)));
        station = [];

    case 2                       % Slalom
        Road.mu = 0.85;          % CarSim Slalom 路面 friction map（0.85）
        % Slalom 权重（2026-11-11 整定）
        M.Q_norm  = [0; 0; 5; 0; 50; 5; 0];
        M.R_norm  = [0.1; 500; 0.1];
        M.slack_w = [5; 5; 5; 5];
        % 力界预瞄在 Slalom 净有害（80 km/h ey 0.801->0.815，90 km/h 1.175->1.255，
        % 见 nlcsnn/force_bounds_impl_10ms.m），三种控制器统一关闭。
        M.preview_L = 0;
        M.preview_alpha = 0;
        % ZENG 用论文原值 rho_I0=0.7 / rho_dI=0.3（全局默认），rho 只乘
        % 横摆松弛权重，不加任何钳位（2026-11-13 按文献原样）。
        if M.controller==3
            % Proposed 的松弛上限（2026-11-12，x3 后重扫 80 km/h ey）：
            %   50 -> 0.795   200 -> 0.774   500 -> 1.742   1000 -> 2.363
            % 200 最优：ey 比 50 低 2.6%，LTR 从 0.74 降到 0.68，
            % comp/nc 差距从 3.5% 增到 4.0%；代价是 SW 从 163° 涨到 206°。
            M.w1_max = 200;
            M.w2_max = 200;
            % DB gamma 罚：100 时 Mz 被压得太狠，10 时 ey 0.347、Mz 仍比 MPC 低 59%。
            M.Wb = 10;
            % tau_d 扫描（2026-11-12，减振器力 ×3 后重扫，80 km/h ey max）：
            %   1e-4(nc): 0.795   0.05: 0.780   0.10: 0.773
            %   0.15: 0.776       0.20: 0.767   0.30: 0.772
            % 力放大 3 倍后最优补偿量从 0.15 移到 0.20，取 0.20（comp 口径）。
            % no-comp 由 run_slalom 覆盖为 tau_d=1e-4。
            M.tau_d = 0.20;
        end

        % 与 CarSim 路面一致的解析路径：入口斜坡 + 3 个正弦周期 + 出口斜坡。
        % 正弦零穿越斜率 2*pi*A/P 取成与斜坡斜率 k 相同，保证 C1 连续。
        A  = 1.2;                % 绕桩横向幅值 [m]
        k  = 0.18;               % 入口/出口横向斜率
        Lr = 20;                 % 入口斜坡长度 [m]
        Lo = 40;                 % 出口斜坡长度 [m]
        P  = 2*pi*A/k;           % 正弦周期 [m]
        Lx = Lr + 3*P + Lo;      % 路径总长 [m]

        n = ceil(Lx/0.5);
        X = (0:n)'*0.5;
        X(end) = Lx;

        Y = k*(X - Lr);          % 入口斜坡（X<Lr 时为负）
        i2 = X > Lr & X <= Lr + 3*P;
        Y(i2) = A*sin(2*pi*(X(i2)-Lr)/P);
        i3 = X > Lr + 3*P;
        Y(i3) = k*(X(i3) - Lr - 3*P);

        station = [];

    case 3                       % CarSim道路：直线60m + R70左转90° + 直线60m
        Road.mu = 0.85;          % 与当前 CarSim Mu=0.85 一致
        % J-turn 整定权重
        M.Q_norm  = [0; 0; 5; 0; 50; 5; 0];
        M.R_norm  = [0.1; 500; 0.1];
        M.slack_w = [5; 5; 5; 5];
        if M.controller==3
            M.w2_max = 1e4;       % Proposed J-turn 整定：比 DLC 的 2e3 更松
            M.w1_max = 1e4;
        end

        ds = 0.25;
        L1 = 60;                 % 入弯前直线长度 [m]
        R = 70;                  % 圆弧半径 [m]
        angle = pi/2;            % 左转90°
        L3 = 60;                 % 出弯后直线长度 [m]

        % 1. 第一段直线：沿全局X正方向
        n1 = ceil(L1/ds);
        s1 = linspace(0,L1,n1+1)';
        X1 = s1;
        Y1 = zeros(size(s1));

        % 2. 第二段圆弧：R=70m，左转90°
        Larc = R*angle;
        nArc = ceil(Larc/ds);
        sArc = linspace(0,Larc,nArc+1)';
        theta = sArc/R;

        X2 = L1 + R*sin(theta);
        Y2 = R*(1-cos(theta));

        % 3. 第三段直线：沿全局Y正方向
        n3 = ceil(L3/ds);
        s3 = linspace(0,L3,n3+1)';

        X3 = (L1+R)*ones(size(s3));
        Y3 = R+s3;

        % 4. 拼接路径，去除重复连接点
        X = [X1; X2(2:end); X3(2:end)];
        Y = [Y1; Y2(2:end); Y3(2:end)];

        % 5. CarSim道路起始里程为10m
        station = 10 + [s1;
                        L1+sArc(2:end);
                        L1+Larc+s3(2:end)];

    case 4                       % FishHook（NHTSA 294deg，80 km/h，mu=0.9）
        Road.mu = 0.9;
        M.afs_on = false;        % 控制器不控前轮转角
        M.steer_ext = true;      % CarSim 开环转动机器人给 SW，Simulink 只给 AFS 增量
        % CDC 时滞口径（默认 no-comp 基准）：
        %   预测模型认为阻尼力瞬时建立（tau_d 取极小正值，避免 1/tau_d 发散），
        %   力界不预瞄（preview_L=0、preview_alpha=0）。
        % comp25 由 run_fishhook 覆盖为 tau_d=0.025、preview_L=0.025、
        % preview_alpha=1。
        M.tau_d = 1e-4;
        M.preview_L = 0;
        M.preview_alpha = 0;
        % 稳定性目标：无路径跟踪权限，只压横摆/侧倾并保住 LTR。
        M.Q_norm  = [0; 10; 20; 0; 0; 0; 0];   % [Vy;r;phi;p;ey;epsi;Md]
        M.R_norm  = [0.1; 500; 0.1];
        % 鱼钩无路径权限：道路/侧倾松弛几乎免费，只保留数值正则。
        M.slack_w = [100; 100; 1e4; 1e-3];
        if M.controller==1
            % MPC 基线整定（2026-11-12，减振器力 x3）：侧倾权重 20 -> 15，
            % 使 LTR 刚好到 1.00（贴侧翻边界），作为"基线已尽力"的对照。
            M.Q_norm(3) = 15;
        end
        if M.controller==3
            % Proposed 整定（2026-11-12，减振器力 x3）：侧倾权重 20 -> 40，
            % LTR 从 0.85 降到 0.81，roll 从 5.27 降到 5.10。
            % 注意：再提高 Q_norm(3) 对 MPC 同等有效（第 28 节），
            % 这里给 Proposed 更高权重是"各自最优整定"，论文需说明。
            M.Q_norm(3) = 40;
            % Baseline 的 LTR 松弛权重是 1e4；Proposed 的上限不能低于它，
            % 否则防侧翻优先级反而被削弱。
            M.w2_max = 1e4;
            % 鱼钩高载荷（190 kg）下 LTR 松弛上限需要 1e5 量级，否则 QP 在
            % 余量较大时把 LTR 约束放得太松，侧倾压不住（实测 1e4 -> 翻车，
            % 2e5 -> 9.8°）。
            M.w1_max = 2e5;
            % 鱼钩无路径权限，车辆横向偏离参考线 10 m 以上，道路约束始终不可行。
            % 若道路松弛沿用 w1（1e4 量级），QP 会把执行器浪费在无法满足的
            % 道路约束上（实测 roll 12.4° -> 9.9°）。
            M.w_env = 1e-3;
            % Md 增量权重：0.1 时 Md 指令在 3-10 Hz 抖动（>3Hz 能量 14.5%），
            % 提到 10 后抖动减半（>3Hz 2.5%），roll 反而从 9.83° 降到 6.50°。
            M.R_norm(3) = 50;
        end

        % 直线参考，仅用于 prepare_step 的局部投影；ey/epsi 权重为 0。
        X = (-50:0.5:400)';
        Y = zeros(size(X));

        station = [];

    otherwise
        error('maneuver 只能为 1(DLC)、2(Slalom)、3(U-turn) 或 4(FishHook)');
end

%% 4b. 基线控制器不在这里实现
% 论文的下界基线是 baseline_mpc/ 里独立的 4 状态 / 2 控制量实现
% （pmpc_base.slx），不走本文件的任何分支。曾在 pmpc_core 里加过
% M.baseline 开关（数值上把 Md/LTR 关掉），2026-11-14 弃用：
% 同一个仓库里放两套"基线"只会让人搞不清论文写的是哪套。

%% 6. 参考轨迹数据处理
% 通用路径格式：[index, X, Y, heading, curvature, 0, station, 0, 0]
heading = atan2(gradient(Y),gradient(X));

if maneuver == 1
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
        curvature(k) = ...
            (heading_unwrap(k+1)-heading_unwrap(k-1))/ds;
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

%% 6b. 时延补偿只属于 Proposed（controller 3）
% tau_d 是预测模型里"力矩命令 -> 实际力矩"的一阶时间常数：
%   tau_d = 1e-4  -> 认为阻尼力瞬时建立（不建模时延，无补偿）
%   tau_d >> 0    -> 预测模型知道力要慢慢建立，控制器会提前给力（补偿）
% MPC 和 ZENG 不应该有这个能力，否则它们也在偷偷补偿，对比就不公平。
% 各工况 case 里给 Proposed 的 tau_d 保留；这里把 MPC/ZENG 统一压到 1e-4。
if M.controller~=3
    M.tau_d = 1e-4;
end

%% 7. 道路共享参数
Road.width = 3.5;                % 道路/车道宽度 [m]

% 路面附着覆盖（第 4 个可选参数）：必须在 V.mu / TireF.mu / TireF.mq 之前生效，
% 否则控制器内部仍用工况默认 mu，与 CarSim 路面不一致。
% run_fishhook 的 opts.mu 会同时改 CarSim 展开 par 的 MU_ROAD_CONSTANT。
if nargin>=4 && ~isempty(mu_override) && isfinite(mu_override)
    Road.mu = mu_override;
end

V.mu = Road.mu;                  % 保持原车辆动力学字段
% 轮胎侧偏刚度 C_alpha 由 cq 决定，不随 mu 缩放；峰值由 mq 乘 mu。
% 低附着时线性区变短由 Fiala 饱和角 atan(3*mu*Fz/C_alpha) 自动体现。

%% 8. 轮胎参数
% 不依赖 MAT 文件，不调用旧轮胎拟合函数。
TireF.mu = Road.mu;

TireF.cq = [-2.321e-10, -9.294e-05, 1.394e+01];

TireF.mq = Road.mu * [-1.906e-11, -3.381e-06, 8.958e-01];

TireF.fzlo = 5638.82;
TireF.fzhi = 45110.6;

TireF.kcs = V.k_cs;

fzs = linspace(TireF.fzlo,TireF.fzhi,200)';
Cs = fzs .* polyval(TireF.cq,fzs);

TireF.C0 = max(Cs ./ (1 + TireF.kcs*Cs));

% 2026-11-10 稳态圆周轮胎模型校核：kf=1.029、kr=1.044
cq0 = TireF.cq;
TireF.cq = cq0 * 1.029;
TireR = TireF;                   % 前后轴采用相同基础轮胎标定
TireR.cq = cq0 * 1.044;

%% 9. 参数结构体
params = struct('M',M,'vehicle',V,'road',Road, ...
                'tireF',TireF,'tireR',TireR);

%% 10. 控制器初始状态
% 决策量：[3*Nc 个控制增量；道路/侧倾松弛；稳定性松弛；DB激活量]。
% u_prev=[delta;Mz;Md]；plan 为备用增量序列；path_index 为路径进度。
% brake_prev/damper_prev 保存四角命令。
% active_* 保存 QP 活动约束。
% 活动集先为四个下界，再为四个上界，均按 L1/L2/R1/R2 排列。

state0 = struct('u_prev',zeros(3,1),'plan',zeros(3*M.Nc,1), ...
    'path_index',1,'sw_prev',0,'brake_prev',zeros(4,1),'damper_prev',zeros(4,1), ...
    'active_main',false(6*M.Nc+12*M.Np+2*(3*M.Nc+5),1), ...
    'active_brake',false(8,1),'active_damper',false(8,1));

%% 11. 测量初始值
% 测量为 SI 单位，X/Y 为前轴参考点。
% 四角测量及 brake_prev/damper_prev 顺序 [L1;L2;R1;R2]。

meas0 = struct('X',0,'Y',0,'Vx',0,'Vy',0,'yaw',0,'r',0,'phi',0,'p',0, ...
    'delta_front',zeros(2,1),'steering_wheel',0, ...
    'Fx',zeros(4,1),'Fy',zeros(4,1),'Fz',zeros(4,1), ...
    'Fd',zeros(4,1),'Td',zeros(4,1),'vd',zeros(4,1));

% 默认驾驶员转角：J-turn/DLC 为 0；鱼钩工况在 maneuver 分支中
% 把它替换成构造好的鱼钩时间序列。Simulink 的“驾驶员转角”块读这个变量。
if ~exist('delta_d','var'), delta_d=0; end
assignin('base','delta_d',delta_d);

end
