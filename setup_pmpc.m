function P = setup_pmpc()
%SETUP_PMPC  Assemble the common plant and selected controller tuning.
% Editable control defaults: config/Atuning_mpc.m, Atuning_zeng.m,
% Atuning_pmpc.m, Atuning_pmpc_nodelay.m. PMPC_* overrides take precedence.
% With no output, also export PMPC_P and NLCSNN to the base workspace.

startup_pmpc();

InitialParams = localInitialParams();

% ---- NLCSNN CDC model shared by MPC, ZENG, and PMPC ----
NLCSNN = struct();
NLCSNN.enabled = logical(localWsget('PMPC_NLCSNN', 1));
NLCSNN.net = nlcsnn_damper_init();
NLCSNN.i_max = 1.6;
NLCSNN.temp = 42.5;
NLCSNN.x_ref = 281.0645;
NLCSNN.dt_plant = 0.001;   % 1 kHz plant 步长 (多速率: 正模型在 plant 侧跑)
% 旧 NLCSNN.dt = 0.01 是 10 ms 控制器宏步, 新架构下正/逆模型不再共用, 已弃用
% Causal vehicle-side acceleration preprocessing only. It does not delay
% the 100 Hz current command or the NLCSNN hidden-state force response.
% 0.04 s suppresses the 15--16 Hz finite-difference feedback that produced
% late-run wheel-hop in DLC/J-turn while preserving the 1 kHz plant update.
NLCSNN.tau_accel = 0.04;
% Smooth 1 kHz dissipative projection after the neural forward model.
% c_min is the measured 1.6 A soft-state equivalent damping at 1.66 Hz;
% it prevents an out-of-training-band neural prediction from removing all
% wheel-hop damping. Units: c_min [N/(mm/s)], v_eps [mm/s],
% power_eps [N*mm/s].
NLCSNN.passivity = struct('c_min', 0.8, 'v_eps', 5, 'power_eps', 50);

%% ==== 运行配置 (详见 README_运行配置.md) ====
InitialParams.BaselineMode    = localWsget('PMPC_BASELINE',   0);   % 1=无控制baseline
InitialParams.BaselineCurrent = localWsget('PMPC_BASELINE_I', 2.0); % A, baseline 时 MR 电流
InitialParams.FishhookMode    = localWsget('PMPC_FISHHOOK',   0);   % 1=鱼钩纯防侧翻
%  工况类型和 mu 由数据集名解析；J-turn 半径以 CarSim 展开的道路为准。
%  DLC80_mu0.5_* 解析结果与 2026-09-26 前写死的 (DLC, mu = 0.5) 完全相同。
[~, ~, ds_mv] = func_RunMode();
MV = func_ManeuverFromName(ds_mv);
if MV.type == 5 || MV.type == 6
    MV = func_ManeuverFromRun(ds_mv,fullfile(func_CarSimResDir(),'Run_all.par'));
end
CombinedCourse = func_CombinedCourse();
if MV.combined, CombinedCourse = func_CombinedCourse(MV.R,MV.turn_x); end
if MV.combined
    Reftraj = func_WayPoints(MV.type,MV.R,false,MV.turn_x);
else
    Reftraj = func_WayPoints(MV.type,MV.R,false);
end % Runtime path is in memory; preserve MAT files.
PMPC_cargo = localWsget('PMPC_CARGO', 0);   % 与横幅同源
[VehiclePara] = func_VehicleParams(PMPC_cargo);
%  模型基准试验开关 (改进.md 18z, 默认全关 = erd_0926_pi 版本, 用户 2026-09-27 定):
%    PMPC_ISW     传动比 (CarSim 齿条表 18.57)
%    PMPC_KCS     转向柔性, 轴级 deg/kN (实测约 0.06; >0 时前轴用有效特性 func_TireTable 'comply')
%    PMPC_ARLIM_MU 1 = 后轴侧偏界 = mu * 参考表峰值角 (后轮静载)
VehiclePara.isw  = localWsget('PMPC_ISW', VehiclePara.isw);
VehiclePara.k_cs = deg2rad(localWsget('PMPC_KCS', 0))/1000;   % rad/N (轴级侧向力); 0 = 不建模柔性
% VehiclePara.mu = 0.3;
VehiclePara.mu = MV.mu;              % 路面设定值, 取自数据集名 (须与 CarSim 路面数据集一致)
VehiclePara.mu_eff = 0.855*MV.mu;    % 轮胎可实现的侧向附着 = 0.855*mu_road (mu = 0.5 时恰为 0.4275)
%   0.855 是实测 carpet 的峰值 mu_y (MU_REF=1): Fz=5639N 时峰值 9.5deg / mu_y=0.8557,
%   与 Magic Formula 拟合的 D_scale=0.8550 一致。mu=0.9 时为 0.770（原标定 0.78）。

% Load-calibrated Fiala tire model; C0 chooses measured origin stiffness.
fia_c0 = localWsget('PMPC_FIALA_C0', 1);
TireF = func_TireTable('carsim', VehiclePara.mu, 2, fia_c0);
TireR = func_TireTable('carsim', VehiclePara.mu, 2, fia_c0);
TireFHigh = TireF;
TireRHigh = TireR;
if MV.combined
    TireFHigh = func_TireTable('carsim',MV.mu_high,2,fia_c0);
    TireRHigh = func_TireTable('carsim',MV.mu_high,2,fia_c0);
end
if VehiclePara.k_cs > 0
    TireF = func_TireTable('comply', TireF, VehiclePara.k_cs);   % 前轴有效侧偏特性 (对运动学转角)
    TireFHigh = func_TireTable('comply', TireFHigh, VehiclePara.k_cs);
end
RoadMu = struct('enabled',MV.combined, ...
    'switch_x',MV.mu_switch_x, 'low',MV.mu, 'high',MV.mu_high, ...
    'switch_station',inf, 'arlim_low',0, 'arlim_high',0, ...
    'dlc_end_x',CombinedCourse.dlc_end_x, ...
    'target_kmh',CombinedCourse.target_kmh, ...
    'turn_target_kmh',CombinedCourse.turn_target_kmh, ...
    'speed_ramp_m',CombinedCourse.speed_ramp_m);
if MV.combined
    firstHigh = find(Reftraj(:,2)>=MV.mu_switch_x,1,'first');
    RoadMu.switch_station = interp1(Reftraj(1:firstHigh,2), ...
        Reftraj(1:firstHigh,7),MV.mu_switch_x);
end

%% ---- 运行配置横幅 (命令窗口可见, 避免跑错模式) ----
modeStr = {'PMPC 协调控制','Baseline 无控制'};
manvStr = {'DLC 路径跟踪','鱼钩 纯防侧翻'};
if MV.type == 2 && ~InitialParams.FishhookMode
    manvStr{1} = 'Slalom 路径跟踪';
end
disp(' ');
disp(['======== ' modeStr{InitialParams.BaselineMode+1} ...
      '  |  '    manvStr{InitialParams.FishhookMode+1} ' ========']);
disp(['  载荷 '  num2str(PMPC_cargo) ' kg' ...
      ' | mu '  num2str(VehiclePara.mu) ...
      ' | m '   num2str(VehiclePara.m) ...
      ' | lf '  num2str(VehiclePara.lf) ...
      ' | Ix '  num2str(VehiclePara.Ix) ...
      ' | h_S2R ' num2str(VehiclePara.h_S2R)]);
if InitialParams.BaselineMode
    disp(['  MR 标称电流 ' num2str(InitialParams.BaselineCurrent) ' A, AFS/DB 不介入']);
end
% 约束参数
Constraints.Roadwidth  = 3.5;
% ---- 车道约束: 车身四角点 (论文 III-B, 2026-09-25) ----
%  尺寸取自 CarSim E_Class_SUV 外形 body/details/body_trim.obj 的包围盒(原点在前轴):
%  车头在前轴前 0.860 m, 车尾在前轴后 3.972 m; 不含后视镜半宽 0.982 m。
%  a_f/a_r = 质心到车头/车尾, 由 lf(= LX_CG_TL) 推, 质心位置改了会自动跟上
%  (当前数据集 lf = 1.2466 -> a_f = 2.107, a_r = 2.725)。
%  四角横向偏移 e_y + a*e_psi +- W_v/2 (小角近似, |e_psi|<pi/2 时偏保守)。
%  与 es/emax 分开: emax 还用于 Q 的归一化和越界指标, 不随约束几何变。
%  localWsget 口子: 工作区给 PMPC_ENV_AF/AR/WV/ES 可覆盖(对照试验用)。
%  af=ar=0, Wv=tf, es=0.43 即退回旧的"质心 + 半轮距 + 0.43"约束。
Constraints.env_af = localWsget('PMPC_ENV_AF', 0.860 + VehiclePara.lf);   % m
Constraints.env_ar = localWsget('PMPC_ENV_AR', 3.972 - VehiclePara.lf);   % m
Constraints.env_Wv = localWsget('PMPC_ENV_WV', 1.96);                     % m
Constraints.env_es = localWsget('PMPC_ENV_ES', 0.2);                      % m, 安全余量
% Replace steering uses 720 deg/s from the fishhook robot command; this is
% not a measured EPS/SBW limit and should be replaced when hardware data exist.
Constraints.SW_rate_max_deg = 720;                     % deg/s @方向盘
Constraints.dlt_rate_max = deg2rad(Constraints.SW_rate_max_deg / VehiclePara.isw);
% AFS-add mode uses a separate incremental motor limit.
Constraints.AFS_rate_max = localWsget('PMPC_AFSRATE', 0.5);            % rad/s @前轮
if localWsget('PMPC_AFSADD', 0)
    Constraints.dlt_rate_max = Constraints.AFS_rate_max;
end
Constraints.SW_max     =deg2rad(240);
% Fishhook AFS-add steering authority; zero disables AFS for diagnostics.
Constraints.AFS_max_deg = localWsget('PMPC_AFSMAX', 5.73);   % 0 = AFS 不作用(诊断: 只看驾驶员)
Constraints.AFS_max    = deg2rad(Constraints.AFS_max_deg);
% Fixed 15 ms prediction lag is a literature estimate, not identified from
% this project's sinusoidal damper data. NLCSNN plant dynamics remain separate.
Constraints.tau_MR = 0.015;  % s, MR 力响应时间 (Delphi MagneRide)
% ZENG 运行模式由控制器变体决定；具体调参见 Atuning_zeng.m。
Constraints.ZengRho_on = 0;   % 占位, 真值在 ContrlMode 处按数据集名设定
ZengTuning = Atuning_zeng();
PmpcTuning = Atuning_pmpc();
Constraints.ZengLong_on  = ZengTuning.ZengLong_on;
% Shared diagnostic switches; defaults preserve the comparison setup.
Constraints.LongLim_diag = localWsget('PMPC_LONGLIM', 0);
Constraints.afs_box_exact = localWsget('PMPC_AFSBOX', 0);   % 1 = AFS 力限转角箱查表反解 (诊断, 改进.md 18z)
Constraints.lane_off = localWsget('PMPC_LANEOFF', 0);        % 1 = 去掉车道角点约束 (诊断, 改进.md 18z)
Constraints.Zg_rfloor    = ZengTuning.Zg_rfloor;
Constraints.Zg_Vmin      = ZengTuning.Zg_Vmin;
Constraints.Zg_tau       = ZengTuning.Zg_tau;
Constraints.Zg_tpid      = ZengTuning.Zg_tpid;
Constraints.Zg_cmax      = ZengTuning.Zg_cmax;
Constraints.Zg_ayfull    = ZengTuning.Zg_ayfull;
Constraints.Zg_kovs      = ZengTuning.Zg_kovs;
Constraints.Zg_Npv       = ZengTuning.Zg_Npv;
Constraints.Zg_apv       = ZengTuning.Zg_apv;
Constraints.Zg_dVdb      = ZengTuning.Zg_dVdb;
Constraints.QPA_lamd     = 0.003;    % 分配 QP 的增量惩罚 (0 = 关, 与原行为逐位等价)
Constraints.Gov_lamx     = 0.1;      % 分配器里纵向相对横摆的优先级 (<1 => 横摆优先)

%  gamma 二次正则系数: 优先级罚本身是线性的(Hajiloo 2023 Eq.27),
%  纯线性会使 H 的 gamma 块奇异、active-set 返回伪不可行(实测 28/799 拍)。
%  加 eta*Nc*V 保持正定; 实测 eta 从 1e-4 到 1e-1 都使 unsolved 归零,
%  取最小的 1e-4 (gamma=1 处二次项仅为线性项的 0.01%)。
Constraints.gamma_reg = 1e-4;

Constraints.Vel_ref = 25;    % m/s (90 km/h), 用于 r_ref = mu*g/Vel_ref
Constraints.Vd_ref  = 0.5;   % m/s, 参考减振器速度, 用于 Md_ref
% 切线刚度下限: |c_bar| >= Ctan_floor_frac * C0。
% 2026-09-05 改为 0(关闭)。该下限原是为 Fiala 的 c_bar 精确归零加的兜底;
% 换成 CarSim 实测 carpet 后它反而有害 —— 它强迫模型宣称轮胎有 39510 N/rad
% 的刚度, 而表里的 f_bar 已经饱和, f_bar 与 c_bar 不再同源, 仿射模型自相矛盾。
% 实测(鱼钩): floor 0 -> 799 拍全成功; 0.1 -> 16 拍撞迭代上限; 0.25 -> 43 拍 + 1 无解。
% 注意: cb_f = -max(abs(cb_f), kap*C0) 这个写法在 kap=0 时仍强制负号 ——
% 这一层必须留着, 因为实测表在约 10 deg 峰值之后 dFy/dalpha 变号,
% 而鱼钩前轴侧偏角能到 17.35 deg, 正的 CbF 会翻转控制增益的符号。
Constraints.Ctan_floor_frac = 0;
% sigma 耦合预算: sigma_AFS + sigma_DB <= sigma_budget。0 = 关闭。
% sqrt(2) 来自摩擦椭圆 a^2+d^2<=1 的切线线性化(见 func_BuildQPConstraints 3b)。
% 前轴摩擦耦合(方案 B): AFS 改变 Fyf 会吃掉前轮纵向余量。1=开, 0=关。
%
% ⚠️ 这两个放在 MPCParameters 而不是 Constraints, 是 codegen 的硬要求:
%   它们各自控制 func_BuildQPConstraints 里要不要**多拼几行约束**
%   (fx_couple 加 4*Nc 行, sigma_budget 加 1 行), 于是直接决定 A_cons 的行数。
%   Constraints 属于 S0(跨拍状态), 不是编译期常量 -> R2018a 认为行数可变 ->
%   传进 mpcqpsolver 时报
%     "Dimension 1 is fixed on the left-hand side but varies on the right"
%   MPCParameters 属于 Pm, 在块里是 coder.Constant, 分支在编译期就折掉,
%   行数随之固定。两者都恒为 0, 所以这么改**不影响任何数值**。
%  鱼钩模式也必须放一份到 MPCParameters: pmpc_step 里 4 处
%  `if InitialParams.FishhookMode` 决定路径曲率(原 Bezier_SK, 现 Kap_dyn/Kap_node)是 [](0x1) 还是 Np x 1,
%  以及 PrjP 的字段集。InitialParams 属于 S0(跨拍状态)=运行时值, 两支都活着,
%  Bezier_SK 就成了变尺寸。放进 MPCParameters(Pm, coder.Constant)后整支折掉。
%  Zeng 纵向限速里**决定数组尺寸**的两个量也必须放这儿(规律同上):
%    Zg_Npv   决定 r_pv = r_pv(1:min(Zg_Npv,numel(r_pv))) 的切片长度
%    Zg_ayfull 决定 c_pv 走哪条分支
%  放在 Constraints(S0 跨拍状态)里的话, R2018a 当运行时值 -> r_pv/c_pv/t_pv/V_cand
%  全变成变长 -> min(V_cand) 维度歧义, 运行到某一拍报
%    "The working dimension was selected automatically, is variable-length,
%     and has length 1 at run time."
MPCParameters.Zg_Npv       = Constraints.Zg_Npv;
MPCParameters.Zg_ayfull    = Constraints.Zg_ayfull;
MPCParameters.FishhookMode = InitialParams.FishhookMode;
%  AFS_add = 1: **辅助驾驶**架构 —— CarSim 闭环驾驶员模型给基底转角(IMP_STEER_SW = Add),
%  MPC 只叠加 AFS 增量(权限 AFS_max, 速率 AFS_rate_max)。= 0: 自动驾驶(Replace, 输出总转角)。
%  与 Chen 2026 (VSD, FSC-DMPC) 同一口径: 画出的前轮转角由驾驶员主导, AFS 只是小增量。
MPCParameters.AFS_add = localWsget('PMPC_AFSADD', 0);
MPCParameters.sigma_budget = 0;
MPCParameters.fx_couple    = 0;
Constraints.Tb_max = 3000;
Constraints.dTb_rate_max = 2e4;
Constraints.Vymax   = 2;             
% PMPC_ARLIM_MU=1 scales the rear slip-angle bound with road friction.
if localWsget('PMPC_ARLIM_MU', 0)
    Ccp   = load('TireCarpet_265_75R16.mat');
    Fzr0w = VehiclePara.m*VehiclePara.g*VehiclePara.lf/VehiclePara.L/2;     % 后轮单轮静载
    [~, ipk] = max(interp1(Ccp.FZ(:), Ccp.FY.', Fzr0w, 'linear'));
    Constraints.arlim = deg2rad(Ccp.AL(ipk))*VehiclePara.mu/Ccp.MU_REF;
    clear Ccp Fzr0w ipk
else
    Constraints.arlim = deg2rad(5);
end
RoadMu.arlim_low = Constraints.arlim;
RoadMu.arlim_high = Constraints.arlim;
if MV.combined && localWsget('PMPC_ARLIM_MU',0)
    RoadMu.arlim_high = Constraints.arlim*MV.mu_high/MV.mu;
end
Constraints.phimax  = deg2rad(4);    
Constraints.dphimax = deg2rad(50);          
Constraints.epsimax = deg2rad(8);  
Constraints.epsilon_r = 10*deg2rad(5);
Constraints.epsilon_alpha = 10*deg2rad(5);
Constraints.epsilon_LTR = 3;
Constraints.epsilon_e   = 8;
% Unbounded soft slacks preserve QP feasibility.
Constraints.eps_ub_scale = Inf;

% MPC 参数
MPCParameters.Nu = 3;
MPCParameters.Ne = 8;
MPCParameters.Nr = 3;    % 统一编译期 QP 结构: gamma 固定保留 3 维
% PMPC priority logic is selected at runtime; keep this field in Pm so
% code generation sees a fixed QP layout for all three controller variants.
MPCParameters.PrioMode = 0;
Constraints.prio_vartheta = PmpcTuning.prio_vartheta;
Constraints.prio_wmin  = PmpcTuning.prio_wmin;
Constraints.prio_wmax  = PmpcTuning.prio_wmax;
% 1 = MPC (6x3); 2 = ZENG (7x3); 3 = PMPC (7x3); 4 = PMPC-noDelay (6x3).
ControllerVariant = localWsget('PMPC_CONTROLLER_VARIANT', 0);
[md_auto, zg_auto, ds_auto] = func_RunMode();
if ControllerVariant == 1
    ContrlMode = 2; Constraints.ZengRho_on = 0;
elseif ControllerVariant == 2
    ContrlMode = 2; Constraints.ZengRho_on = 1;
elseif ControllerVariant == 3 || ControllerVariant == 4
    ContrlMode = 1; Constraints.ZengRho_on = 0;
elseif ControllerVariant == 0
    ContrlMode = localWsget('PMPC_MODE', md_auto);
    Constraints.ZengRho_on = localWsget('PMPC_ZENGRHO', zg_auto);
    if ContrlMode == 1
        ControllerVariant = 3;
    elseif Constraints.ZengRho_on
        ControllerVariant = 2;
    else
        ControllerVariant = 1;
    end
else
    error('setup_pmpc:InvalidControllerVariant', ...
        'PMPC_CONTROLLER_VARIANT must be 0 (auto), 1 (MPC), 2 (ZENG), 3 (PMPC), or 4 (PMPC-noDelay).');
end
switch ControllerVariant
    case 1, Tuning = Atuning_mpc();
    case 2, Tuning = ZengTuning;
    case 3, Tuning = PmpcTuning;
    case 4, Tuning = Atuning_pmpc_nodelay();
end
if ControllerVariant == 4
    Constraints.prio_vartheta = Tuning.prio_vartheta;
    Constraints.prio_wmin = Tuning.prio_wmin;
    Constraints.prio_wmax = Tuning.prio_wmax;
end
%  被控对象侧的阻尼器执行器模型(一阶滞后作用在"力在上下界间的位置"上, 保耗散):
%  1 = 开(与预测模型一致), 0 = 关(阻尼力当拍直达 CarSim, 即 2026-09-25 前的行为)
Constraints.dmp_act_on = localWsget('PMPC_DMP_ACT', double(~NLCSNN.enabled));
% Prediction grid (Ts/Np/Nc) comes from the selected Atuning_* file.
% PMPC_TS / PMPC_NS / PMPC_NC workspace overrides remain available.
MPCParameters.Ts = localWsget('PMPC_TS', Tuning.Ts);
MPCParameters.Ns = localWsget('PMPC_NS', Tuning.Np);
MPCParameters.Np = MPCParameters.Ns;
MPCParameters.Nc = localWsget('PMPC_NC', Tuning.Nc);
% 0 freezes the local tire linearization; 1 enables nodewise LTV.
MPCParameters.LTV_on = 0;
MPCParameters.QPSolver = localWsget('PMPC_QPSOLVER', 1);   % 0 = quadprog, 1 = KWIK(mpcqpsolver)
MPCParameters.Ts_exec = 0.01;   % 触发周期，速率限制按它缩放
Constraints.QPA_release_step = Constraints.dTb_rate_max*MPCParameters.Ts_exec;
%  第一预测步是否按 Ts_exec 离散(非均匀网格, 见 func_DynamicalModel / 规划器 Tk)。
%  0 = 原行为(全部按 Ts); 1 = 第一步 Ts_exec、其余 Ts。2026-09-26 抖动对照用。
MPCParameters.first_Tc = localWsget('PMPC_FIRSTTC', 0);
% ---- 约束时域（分类设置）----
%  远端预测的物理量不可靠，对其施加约束既无实际意义，又给 QP 增加大量
%  近乎等价的候选活跃集，是控制量不平滑的来源之一；缩短还能减少约束行数。
%  但三类约束的"可预测性"不同，不能一刀切：
%    sh (横摆角速度/后轴侧偏角) : 依赖轮胎线性化，远端最不可靠 -> 取 Nc
%    r  (LTR)                  : 同上，且超出 Nc 后输入已冻结          -> 取 Nc
%    env(车道边界)              : 由参考路径几何决定，远端仍然可靠，且
%                                100km/h 下 Nc*Ts=0.3s 只看到 8.3m，
%                                不足 DLC 一个门区(~13.5m) -> 必须取 Np
MPCParameters.Ncons_sh  = MPCParameters.Nc;
MPCParameters.Ncons_r   = MPCParameters.Nc;
MPCParameters.Ncons_env = localWsget('PMPC_NCONS_ENV', MPCParameters.Np);   % 工作区可覆盖(对照试验用)
MPCParameters.ConNodes_sh = (1:MPCParameters.Ncons_sh)';
MPCParameters.ConNodes_r = (1:MPCParameters.Ncons_r)';
MPCParameters.ConNodes_env = (1:MPCParameters.Ncons_env)';

% 0 removes per-step diagnostics from generated code (Pm is coder.Constant).
MPCParameters.Verbose = localWsget('PMPC_VERBOSE', 1);

DiscreteModle = 4;  % 1=Euler; 2=Taylor4; 3=FOH; 4=精确 ZOH(expm)
% The 15 ms lag state requires exact ZOH; Taylor4 is unstable at Ts/tau_d=3.3.
%% ---- 对比模式 ----
%  1 = 本文 PMPC       : 三层车辆级目标 + L3 制动介入代价 gamma_DB
%  2 = baseline MPC   : **权重与 PMPC 完全相同**, 唯一差别是去掉 sigma/gamma 机制
%      这样对比才能分离出"优先级机制"本身的贡献; 旧版那套单独调过的
%      baseline 权重会把"权重不同"和"机制不同"混在一起。
%  先进对比方法(Zeng 2025)在 baseline MPC 基础上开 Constraints.ZengRho_on。
if ControllerVariant == 1 || ControllerVariant == 4
    MPCParameters.Nx = 6;
    MPCParameters.Ny = 6;
    MPCParameters.tau_d = 0;       % 明确不调用 local_delay
elseif ControllerVariant == 3
    % Mainline PMPC: [Vy,r,phi,dphi,e_y,e_psi,Md_a] and three inputs.
    % The tagged 8x4 Vx/Fx experiment remains available separately.
    MPCParameters.Nx = 7;
    MPCParameters.Ny = 7;
    MPCParameters.tau_d = func_DamperDelayTau(zeros(4,1), zeros(4,1), ...
        zeros(4,1), Constraints.tau_MR);
else
    MPCParameters.Nx = 7;
    MPCParameters.Ny = 7;
    % NLCSNN 自身在 plant 侧以 1 kHz 正模型推进；控制器的 7 状态
    % 仍保留文献中的聚合阻尼器执行器滞后预测。
    % 当前固定为 15 ms；func_DamperDelayTau 预留后续按四角减振器状态
    % 查表得到 tau_j 后取 max(tau_j) 的保守聚合规则。
    MPCParameters.tau_d = func_DamperDelayTau(zeros(4,1), zeros(4,1), ...
        zeros(4,1), Constraints.tau_MR);
end
MPCParameters.ControllerVariant = ControllerVariant;
% PMPC-only road-feasibility speed coordination. MPC and ZENG never read it.
% Mode 0 restores the original PMPC longitudinal branch; mode 2 retains a
% curvature-only ablation for the comparison harness.
if ControllerVariant == 3 || ControllerVariant == 4
    LongTuning = Tuning;
    Constraints.LongCoordMode = localWsget('PMPC_LONGCOORD',LongTuning.LongCoordMode);
else
    LongTuning = PmpcTuning;
    Constraints.LongCoordMode = 0;
end
Constraints.Long_mu_reserve = LongTuning.Long_mu_reserve;
Constraints.Long_brake_reserve = LongTuning.Long_brake_reserve;
Constraints.Long_a_max = LongTuning.Long_a_max;
Constraints.Long_preview_nodes = LongTuning.Long_preview_nodes;
Constraints.Long_min_sustain_m = LongTuning.Long_min_sustain_m;
Constraints.Long_delay = LongTuning.Long_delay;
Constraints.Long_tau = LongTuning.Long_tau;
Constraints.Long_F_slew = LongTuning.Long_F_slew;
Constraints.Long_VdownRate = LongTuning.Long_VdownRate;
Constraints.Long_VupRate = LongTuning.Long_VupRate;
Constraints.Long_margin_trigger = LongTuning.Long_margin_trigger;
Constraints.Long_margin_recover = LongTuning.Long_margin_recover;
Constraints.Long_trigger_band = LongTuning.Long_trigger_band;
Constraints.Long_release_band = LongTuning.Long_release_band;
ctlStr = {'① 固定权重 MPC (无 sigma/gamma)','② Zeng rho 调权', ...
    '③ 本文 PMPC','④ PMPC-noDelay (无阻尼器时延预测)'};
if ControllerVariant==4
    ic=4;
elseif ContrlMode==1
    ic=3;
elseif Constraints.ZengRho_on
    ic=2;
else
    ic=1;
end
disp(['  数据集 ' ds_auto '   ->   控制器 ' ctlStr{ic}]);
%  ⚠️ 部署机(没有 CarSim / 没有 simfile.sim)上 func_RunMode 读不到数据集名,
%  会退回默认 mode=2,zeng=0 —— 也就是**baseline MPC**, 而不是本文方法。
%  不提示的话会拿着错的控制器去生成代码, 而且完全看不出来。
if strcmp(ds_auto, '<unknown>') && ~evalin('base','exist(''PMPC_MODE'',''var'')')
    warning('setup_pmpc:NoDataset', ...
        ['读不到 CarSim 数据集名(没有 simfile.sim?), 已退回默认控制器 "%s"。\n' ...
         '若这是部署机, 请先在工作区显式指定, 例如本文 PMPC:\n' ...
         '    PMPC_MODE = 1; PMPC_ZENGRHO = 0;   然后再跑 setup_pmpc'], ctlStr{ic});
end


% --- 根据模式设置 Nr 和 权重 ---
%  消融开关 (改进.md 第 14 条 / 7.2, 仅对 PMPC 生效; 工作区 PMPC_ABL 可覆盖, 默认 0):
%    0 = 本文 PMPC
%    1 = PMPC-noWb : 去掉 L3 的制动介入代价 (W_b = 0), 其余不变
%    2 = PMPC-noSAS: 减振器为标称电流下的被动阻尼; 控制器的 CDC 通道冻结在当前被动力矩
%    3 = [32] 风格  : 旧 PMPC 结构, 8 个松弛 + 3 个 gamma, 按 W_beta >> W_rho >> W_r >> Q 排序的固定权重
%  统一 QP 结构后, #3 不再改编译期维度, 仅改变运行时代价/约束语义。
MPCParameters.abl = localWsget('PMPC_ABL', 0) * double(ContrlMode == 1);
if ContrlMode == 1 || ContrlMode == 2
    % 统一尺寸: 模式切换不再改编译期维度, 仅运行时屏蔽/启用相应变量与约束。
    MPCParameters.PrioMode = 0;
    MPCParameters.Nr = 3;
    MPCParameters.Ne = 8;
    % Constraints.arlim   = deg2rad(5); 
    VehiclePara.CafHat = -57772.51*2;VehiclePara.CarHat = -53484.51*2;%mu=0.85
    % VehiclePara.CafHat = -53645.94*2;VehiclePara.CarHat = -49663.82*2;%mu=0.3
    Constraints.es = 0.43; Constraints.emax= 0.5*(Constraints.Roadwidth-VehiclePara.tf)-Constraints.es;  
    Constraints.LTR_lim = 0.8; 
    CostWeights.Q1=Tuning.Q1; CostWeights.Q2=Tuning.Q2;
    CostWeights.Q3=Tuning.Q3; CostWeights.Q4=Tuning.Q4;
    CostWeights.Q5=localWsget('PMPC_Q5', Tuning.Q5); CostWeights.Q6=localWsget('PMPC_Q6', Tuning.Q6);
    CostWeights.Q8 = 0;
    CostWeights.R4 = 0;
    CostWeights.S4 = 0;
    if InitialParams.FishhookMode
        % 纯防侧翻: 无路径、无偏航角速度参考
        CostWeights.Q1=0;    CostWeights.Q2=0;
        CostWeights.Q3=5e6;  CostWeights.Q4=2e3;   % 换实测轮胎表后由 2e6 重调
        CostWeights.Q5=0;    CostWeights.Q6=0;
    end
    CostWeights.R1=localWsget('PMPC_R1', Tuning.R1); CostWeights.R2=Tuning.R2; CostWeights.R3=Tuning.R3;
    CostWeights.S1=Tuning.S1; CostWeights.S2=Tuning.S2; CostWeights.S3=Tuning.S3;
    % 终端代价系数(末节点 Q 的倍数)。1 = 关闭。见 func_CostWeighting... 的说明。
    CostWeights.Qf_scale = Tuning.Qf_scale;
    % Gamma penalties; V2 is the differential-braking entry cost.
    CostWeights.V1=Tuning.V1; CostWeights.V2=localWsget('PMPC_V2', Tuning.V2); CostWeights.V3=Tuning.V3;
    % tau_gamma is the priority-switching time constant (0 disables lag).
    CostWeights.tau_gamma = localWsget('PMPC_TAUG', Tuning.tau_gamma);
    CostWeights.Wdg=Tuning.Wdg;
    CostWeights.W1=Tuning.W1; CostWeights.W2=Tuning.W2;
    % Constraint-slack weights (rollover/road penalties remain above tracking).
    CostWeights.W3=Tuning.W3; CostWeights.W4=Tuning.W4;
else
    error('Invalid ContrlMode');
end

% ---- 运行时模式字段(不决定编译期尺寸) ----
Constraints.ControllerMode = ContrlMode;                              % 1=PMPC, 2=baseline/ZENG
Constraints.PrioModeRT = double(ContrlMode == 1 && MPCParameters.abl ~= 3);  % PMPC 新优先级机制
Constraints.GammaDBIndex = 2;                                         % gamma(2) = DB

WarmStart = zeros(MPCParameters.Nc*MPCParameters.Nu + MPCParameters.Ne + MPCParameters.Nr, 1); 
DotPHI = zeros(MPCParameters.Np,1);
rho = diag([1 1 1]);
%% ---- codegen: Zeng 界在 pmpc_step 里每拍写入, 原为动态添加 ----
%  只有 ZENG 模式会真正用到; 预声明为 0 不影响任何分支。
if ~isfield(Constraints,'Zeng_bL'),   Constraints.Zeng_bL   = 0; end
if ~isfield(Constraints,'Zeng_bR'),   Constraints.Zeng_bR   = 0; end
if ~isfield(Constraints,'Zeng_rmax'), Constraints.Zeng_rmax = 0; end

%% ================= 打包 =================
P = struct();
P.InitialParams = InitialParams;
P.Reftraj = Reftraj;
P.VehiclePara = VehiclePara;
P.Constraints = Constraints;
P.MPCParameters = MPCParameters;
P.DiscreteModle = DiscreteModle;
P.ContrlMode = ContrlMode;
P.WarmStart = WarmStart;
P.DotPHI = DotPHI;
P.rho = rho;
P.CostWeights = CostWeights;
P.TireF = TireF;
P.TireR = TireR;
P.TireFHigh = TireFHigh;
P.TireRHigh = TireRHigh;
P.RoadMu = RoadMu;

%% ---- 阶段 D: 再打包成 MATLAB Function block 要的两个子结构 ----
%  Pm = 运行中不变的参数 (块里声明为 Scope=Parameter, build 时固化)
%  S0 = 跨拍状态的初值 (块里用 persistent, 首拍用它初始化)
%  上面那 16 个扁平字段保留不动 —— S-function 那条路还在用。
P.Pm = struct('MPCParameters',MPCParameters, 'CostWeights',CostWeights, ...
              'DiscreteModle',DiscreteModle, ...
              'Reftraj',Reftraj, 'TireF',TireF, 'TireR',TireR, ...
              'TireFHigh',TireFHigh, 'TireRHigh',TireRHigh, ...
              'RoadMu',RoadMu, ...
              'NLCSNN',NLCSNN);
longPrev = struct('Fx_prev',0,'Vset_prev',NaN,'trigger',false, ...
    'release_ticks',0,'allocationFailed',false,'achievedRatio',1);
LongCoord = struct('prev',longPrev,'margin_prev',1000, ...
    'diag',zeros(8,1),'Vx_pred',zeros(MPCParameters.Np,1), ...
    'Fx_request',0,'Fx_achieved',0,'speed_actual',0, ...
    'speed_planned',0,'Vset_pid',0,'speed_mismatch',0, ...
    'allocation_exitflag',1,'qp_exitflag',1,'brake_active',false, ...
    'safety_margin_prev',NaN,'m_plan',NaN,'m_candidate',NaN, ...
    'm_speed',NaN, ...
    'm_witness',NaN,'witness_unknown',true);
P.S0 = struct('InitialParams',InitialParams, 'WarmStart',WarmStart, 'rho',rho, ...
              'VehiclePara',VehiclePara, 'Constraints',Constraints, ...
              'cert', nan(36, 1),'LongCoord',LongCoord);

% 没有输出参数时, 顺手写进 base 工作区
if nargout == 0
    assignin('base','PMPC_P',P);
    % 多速率 (2026-09-29): NLCSNN_Plant/plant_fn 的 Scope=Parameter 叫 NLCSNN,
    % 值从 base workspace 取. 不导出的话 sim 会报
    % "参数 'NLCSNN' 中的 'NLCSNN_Plant/plant_fn' 设置无效 / 变量 'NLCSNN' 无法识别".
    assignin('base','NLCSNN',P.Pm.NLCSNN);
    fprintf('  PMPC_P 已写入工作区 (%d 个字段)\n', numel(fieldnames(P)));
    clear P
end
end

function Pa = localInitialParams()
Pa = struct();
Pa.InitialGapflag = 0;
Pa.WayPoints_IndexPre = 1;

Pa.failed_num = struct('solve',0, 'conv',0, 'unsolved',0, ...
    'NaN_or_Inf',0, 'else',0);
Pa.emax = struct('y',0, 'psi',0);
Pa.U = zeros(3,1);

Pa.prevstate = struct();
Pa.prevstate.sw = 0;
Pa.prevstate.Fyf = 0;
Pa.prevstate.MFx = 0;
Pa.prevstate.Md = 0;
Pa.prevstate.ey = 0;
Pa.prevstate.epsi = 0;
Pa.prevstate.Tb = zeros(4,1);
Pa.prevstate.Fd = zeros(4,1);
Pa.prevstate.s_act = -ones(4,1);
Pa.prevstate.Vd = zeros(4,1);
% The 1 kHz NLCSNN_Forward_1kHz subsystem owns h/v/a filter state.
% The 100 Hz controller retains only the previous inverse-solver current.
Pa.prevstate.nlcsnn = struct('i_prev', zeros(4,1));
Pa.prevstate.gamma = [0; 0; 1];
Pa.prevstate.Vpid = NaN;
Pa.prevstate.iA = false(0,1);

Pa.dr_prev = NaN;
Pa.dr_rate = 0;
Pa.t_solve = zeros(1,20000);
Pa.n_solve = 0;
Pa.ey_hist = zeros(1,20000);
Pa.epsi_hist = zeros(1,20000);
end

function v = localWsget(name, dflt)
% Read runtime configuration from CarSim, then base workspace, then default.
runall = '';
try
    par = fullfile(func_CarSimResDir(), 'Run_all.par');
    if exist(par,'file') == 2
        runall = fileread(par);
    end
catch
end
if ~isempty(runall)
    tok = regexp(runall, ['(?m)^\s*(?:DEFINE_PARAMETER\s+)?' name ...
        '\s*=\s*([-\d.eE+]+)'], 'tokens', 'once');
    if ~isempty(tok)
        v = str2double(tok{1});
        if ~isnan(v), return; end
    end
end
if evalin('base', ['exist(''' name ''',''var'')'])
    v = evalin('base', name);
else
    v = dflt;
end
end
