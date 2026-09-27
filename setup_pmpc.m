function P = setup_pmpc()
%SETUP_PMPC  算出 PMPC 控制器的全部参数与初始状态, 打包成一个 struct
%
%   P = setup_pmpc                 % 只返回, 不写工作区
%   setup_pmpc                     % 同时把 P 写到 base 工作区的 PMPC_P
%   模式选择仍走原来的 wsget 机制(CarSim 数据集名 / base 工作区)。
%
%   这是面向 HIL 的第一步: 初始化不进 Simulink。
%   用法: 先跑一次 setup_pmpc, 工作区就有 PMPC_P 了, 再跑仿真。
%   彻底迁到 MATLAB Function block 后, P 的各字段以块参数(Scope=Parameter)
%   形式传入, 在开发机 build 时解析并固化进生成代码, 目标机不需要工作区。
%
%   注: 本函数里的 wsget / func_RunMode 会读 CarSim 的 simfile.sim 与 Run_all.par,
%   这些都是 codegen 不支持的操作 —— 正因为它们在块外, 所以无所谓。

startup_pmpc();

[~, ~] = func_RLSFilter_Calpha_f('initial', 0.99, 10, 50);
[~, ~] = func_RLSFilter_Calpha_r('initial', 0.99, 10, 50);

[InitialParams] = func_InitialParams;

%% ==== 运行配置 (详见 README_运行配置.md) ====
InitialParams.BaselineMode    = wsget('PMPC_BASELINE',   0);   % 1=无控制baseline
InitialParams.BaselineCurrent = wsget('PMPC_BASELINE_I', 2.0); % A, baseline 时 MR 电流
InitialParams.FishhookMode    = wsget('PMPC_FISHHOOK',   0);   % 1=鱼钩纯防侧翻
%  工况(路径类型 / J-turn 半径 / 路面 mu)从 CarSim 数据集名解析, 见 func_ManeuverFromName。
%  DLC80_mu0.5_* 解析结果与 2026-09-26 前写死的 (DLC, mu = 0.5) 完全相同。
[~, ~, ds_mv] = func_RunMode();
MV = func_ManeuverFromName(ds_mv);
[Reftraj] = func_WayPoints(MV.type, MV.R); % 1-DLC, 2-Slalom, 3-U-Turn, 5-J-turn
C_table = func_tire_init_Calpha(1,8);
% 等效轴刚度(含悬架/转向柔性), Wang et al. IEEE TVT 72(10) 2023 Table I

Cf0=func_Fz_Calpha(4740);
Cr0=func_Fz_Calpha(4380);%4380->后轮单轮垂向力
PMPC_cargo = wsget('PMPC_CARGO', 0);   % 与横幅同源
[VehiclePara] = func_VehicleParams(PMPC_cargo);
%  模型基准试验开关 (改进.md 18z, 默认全关 = erd_0926_pi 版本, 用户 2026-09-27 定):
%    PMPC_ISW     传动比 (CarSim 齿条表 18.57)
%    PMPC_KCS     转向柔性, 轴级 deg/kN (实测约 0.06; >0 时前轴用有效特性 func_TireTable 'comply')
%    PMPC_ARLIM_MU 1 = 后轴侧偏界 = mu * 参考表峰值角 (后轮静载)
VehiclePara.isw  = wsget('PMPC_ISW', VehiclePara.isw);
VehiclePara.k_cs = deg2rad(wsget('PMPC_KCS', 0))/1000;   % rad/N (轴级侧向力); 0 = 不建模柔性
% VehiclePara.mu = 0.3;
VehiclePara.mu = MV.mu;              % 路面设定值, 取自数据集名 (须与 CarSim 路面数据集一致)
VehiclePara.mu_eff = 0.855*MV.mu;    % 轮胎可实现的侧向附着 = 0.855*mu_road (mu = 0.5 时恰为 0.4275)
%   0.855 是实测 carpet 的峰值 mu_y (MU_REF=1): Fz=5639N 时峰值 9.5deg / mu_y=0.8557,
%   与 Magic Formula 拟合的 D_scale=0.8550 一致。mu=0.9 时为 0.770（原标定 0.78）。

% ---- 轮胎模型: 按载荷标定的 Fiala 解析式 (2026-09-27, 改进.md 18z) ----
%  演进: (1) Fiala + Wang 2023 表一的 (90.7e3, 109e3) + 路面 mu=0.9 —— Wang 的值是他那台车的,
%  实测轴级初始切线 121e3(前)/99e3(后), 前轴低估 34%; (2) 改查 CarSim 实测 carpet 的二维表;
%  (3) 现回到 Fiala, 但参数由 carpet **按载荷**标定(峰值力与峰值角对齐), 不再用单一 (C0, mu)。
%  当初否定 Fiala 的理由("@3deg 轴级切线实测 82648, Fiala 只给 55~64e3")正是单一参数所致:
%  按载荷标定后 3deg 轴级切线 74735 vs 实测 77737 (-3.9%), 0-7deg 最大误差 1.6-3.0% 峰值。
%  收益: 去掉 141x70 的 Fbar/Cbar 两张网格, 只留两条二次多项式; 力与切线刚度都是闭式。
%  carpet 出自 CarSim "Tire: Lateral Force / 265-75 R16 / Touring Tires"
%  (8 载荷 x 51 侧偏角, 单胎, MU_REF_Y=1.0), 见 TireCarpet_265_75R16.mat。
%  四条轮胎相同, 前后轴的差别只来自载荷, 故两个模型用同一份标定。
%  Fiala 标定: 1 = C 取实测原点刚度(Tire Tester 实测轴级 145256, 本式给 144808, 差 0.3%),
%  0 = 对齐饱和角(C 偏硬 17%)。闭环两者都测过, =1 更好(出车道 5.8% vs 10.9%), 取 1。
fia_c0 = wsget('PMPC_FIALA_C0', 1);
TireF = func_TireTable('carsim', VehiclePara.mu, 2, fia_c0);
TireR = func_TireTable('carsim', VehiclePara.mu, 2, fia_c0);
if VehiclePara.k_cs > 0
    TireF = func_TireTable('comply', TireF, VehiclePara.k_cs);   % 前轴有效侧偏特性 (对运动学转角)
end

%% ---- 运行配置横幅 (命令窗口可见, 避免跑错模式) ----
modeStr = {'PMPC 协调控制','Baseline 无控制'};
manvStr = {'DLC 路径跟踪','鱼钩 纯防侧翻'};
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
%  wsget 口子: 工作区给 PMPC_ENV_AF/AR/WV/ES 可覆盖(对照试验用)。
%  af=ar=0, Wv=tf, es=0.43 即退回旧的"质心 + 半轮距 + 0.43"约束。
Constraints.env_af = wsget('PMPC_ENV_AF', 0.860 + VehiclePara.lf);   % m
Constraints.env_ar = wsget('PMPC_ENV_AR', 3.972 - VehiclePara.lf);   % m
Constraints.env_Wv = wsget('PMPC_ENV_WV', 1.96);                     % m
Constraints.env_es = wsget('PMPC_ENV_ES', 0.2);                      % m, 安全余量
% ---- 转向速率上限: 执行器的物理能力, 不是可调参数 ----
%  依据(本项目自带, 可直接引用): 鱼钩工况的 NHTSA 转向机器人指令
%    0 -> 294 deg 用 0.4083 s, 294 -> -294 deg 用 0.817 s
%  (CarSim STEER_SW_TABLE 原始数据), 两段都恰为 720 deg/s @方向盘,
%  即 FMVSS/NHTSA 鱼钩规范值。AFS 指令经同一套转向系统执行, 故取同值。
%  折算到前轮: 720/isw = 720/17 = 42.35 deg/s。
%
%  原值 20 deg/s @前轮 只有该能力的一半不到, 是「扫工况扫出来的」——
%  与 R2=1e9 同一类错误(把被控对象的属性当成代价函数的旋钮)。
%  实测它在 DLC 全程 53.2%、失稳末段 71.1% 的时间顶死, 造成相位滞后极限环。
%  ---- 转角速率限 ----
%  本项目是**自动驾驶**控制：DLC 下 IMP_STEER_SW = Replace，
%  控制器指挥的是**总前轮转角**，不是叠加在驾驶员输入上的修正量。
%  所以这里约束的是**整个转向系统（EPS/SBW）**的速率能力；
%  AFS 叠加电机的速率（Bemporad 2013 的 0.5 rad/s）属于**辅助驾驶**架构，不适用。
%
%  【来源存疑】720 deg/s 实际是 **NHTSA 鱼钩试验台转向机器人**的斜坡速率
%  （鱼钩 Run 的 STEER_SW_TABLE: 294 deg / 0.4083 s = 720 deg/s），描述的是工况定义，
%  不是车上执行器规格。尚需补一个真实的 EPS/SBW 速率来源。
%  实测：该值是 DLC 转角 1-3 Hz 自激的主控变量（README 2026-09-05 “车身拖动回归”）。
Constraints.SW_rate_max_deg = 720;                     % deg/s @方向盘
Constraints.dlt_rate_max = deg2rad(Constraints.SW_rate_max_deg / VehiclePara.isw);
%  辅助驾驶(AFS_add = 1)下约束的是 **AFS 叠加电机**, 不是整个转向系统。
%  取 Bemporad 2013 的 0.5 rad/s @前轮 (= 28.6 deg/s), 工作区 PMPC_AFSRATE 可覆盖。
Constraints.AFS_rate_max = wsget('PMPC_AFSRATE', 0.5);            % rad/s @前轮
if wsget('PMPC_AFSADD', 0)
    Constraints.dlt_rate_max = Constraints.AFS_rate_max;
end
Constraints.SW_max     =deg2rad(240);
% 鱼钩(Add 模式)下 AFS 的增量权限, Wang 2023 表一 delta_fm = 0.1 rad = 5.73 deg。
% 拆成 _deg 便于扫描: 降到 0 即「只用 DB+MR, 完全不削驾驶员转角」,
% 用于量化防侧翻里有多少来自削转角 (任务 #9 / 缺陷 A3)。
Constraints.AFS_max_deg = wsget('PMPC_AFSMAX', 5.73);   % 0 = AFS 不作用(诊断: 只看驾驶员)
Constraints.AFS_max    = deg2rad(Constraints.AFS_max_deg);
% MR 力矩变化率 = 可调制幅度 / 响应时间。原写死 500/0.03, 其中 500 N*m 无来源。
% 幅度改用 Md_ref (由阻尼器上下界模型算出, 见 func_CostWeighting... 第 0 节)。
% tau_MR: 本项目的 MR 实测数据(试验\MR减振器, 试验\MR1219)全是正弦激励的
% F-V/F-X 特性试验(行程 100mm, 5 个速度, 电流 0~2.5A), 没有阶跃电流响应测试,
% 辨识出的也是准静态 F(v,I) 模型 —— 所以响应时间无法从自有数据标定, 取文献值。
%   取值依据: Delphi MagneRide (量产车用 MR 减振器, 悬架应用)
%             时间常数 15 ms, 力范围 +-4000 N, 行程 40 mm
%             [Smart Mater. Struct. 28(10) 105028, 2019, doi:10.1088/1361-665X/ab39f2]
%   旁证: MR 减振器一般 5~15 ms 完成约 10:1 的阻尼力变化; 半主动悬架控制需求
%         通常要求 <10 ms; 时滞主因是线圈电感与涡流(磁流变液本身约 1 ms),
%         个别单元因涡流可达 23 ms。
%   原值 30 ms 无出处且偏保守一倍。本项目阻尼器力范围(0.5 m/s 时约 2100 N)
%   与 MagneRide 同量级, 故取 15 ms。
%   若要坐实, 需补一次「定速激励 + 阶跃电流」的响应时间测试。
Constraints.tau_MR = 0.015;  % s, MR 力响应时间 (Delphi MagneRide)
Constraints.I_nom = InitialParams.BaselineCurrent;  % sigma_MR 的中心电流, 与 baseline 同源
% 归一化基准 (任务 #6)。这两个是"代价尺度"的参考量, 不是工况参数:
% 换工况不要改它们, 否则同一个权重数字又会代表不同含义。
%% ---- 先进对比方法: Zeng 2025 稳定性指标调权 ----
%  Zeng et al., IEEE Trans. Transportation Electrification, 2025, DOI 10.1109/TTE.2025.3542558
%  Eq.(14) 的 rho 乘在**稳定性松弛变量的权重**上 (原文 Eq.(26) 的 rho*sigma_s'*W_s*sigma_s):
%    稳定时 rho=0 -> 稳定性约束被衰减, 控制器全力跟踪
%    失稳时 rho=1 -> 稳定性约束被严格执行
%  本项目松弛顺序 [r+ r- alpha+ alpha- LTR+ LTR- ey+ ey-], 权重
%  Wshl = diag([W1 W1 W2 W2 W3 W3 W4 W4]), 故 Zeng 的 sigma_s 对应 W1(横摆)与 W2(侧偏)。
%  0 = 关（用本项目的稳定性约束）
%  1 = Zeng 约束 + 自适应 rho（完整 Zeng 方法）
%  2 = Zeng 约束 + rho 固定为 1（用于分离 rho 本身的贡献）
Constraints.ZengRho_on = 0;   % 占位, 真值在 ContrlMode 处按数据集名设定
%  --- Zeng 方法的**纵向控制**(仅 ZengRho_on 时启用) ---
%  原文的控制量 u = [F_xf, F_xr, F_yf] 含前后轴纵向力, 代价含纵向速度
%  跟踪项, 约束含 u_x ∈ [1,25] m/s。原文明确写着:
%    "the controller determines ... collision avoidance would be unfeasible without
%     reducing speed. Thus, it immediately begins braking from an initial speed of 17 m/s"
%  即 |r| <= mu*g/Vx 这条约束是靠**降低 Vx** 来满足的(Vx 降 -> r_max 抬高)。
%  本项目 MPC 无纵向自由度, 故用外环等效实现: 直接把该约束解出 Vx 上限
%     |r| <= mu*g/Vx   <=>   Vx <= mu*g/|r|
%  路径跟踪工况下 r = Vx*kappa, 故 Vx <= sqrt(mu*g/kappa), 有良性不动点。
Constraints.ZengLong_on  = 1;        % 1 = 给 ZENG 配套纵向控制(忠实复现所必需)
%  诊断开关 (2026-09-26, 改进.md 18z): 让 MPC/PMPC 也用这套纵向限速, 检验"J-turn 偏出车道
%  是因为没有联合纵向车速"。默认 0 = 只有 ZENG 用, 行为不变。工作区 PMPC_LONGLIM 可覆盖。
Constraints.LongLim_diag = wsget('PMPC_LONGLIM', 0);
Constraints.afs_box_exact = wsget('PMPC_AFSBOX', 0);   % 1 = AFS 力限转角箱查表反解 (诊断, 改进.md 18z)
Constraints.lane_off = wsget('PMPC_LANEOFF', 0);        % 1 = 去掉车道角点约束 (诊断, 改进.md 18z)
Constraints.Zg_rfloor    = 0.05;     % rad/s, 防除零
Constraints.Zg_Vmin      = 30/3.6;   % m/s,  速度下限(对应原文 u_x >= 1 m/s 的工程取值)
Constraints.Zg_tau       = 2.5;      % s,    速度误差转减速度的时间常数
%  τ 扫描 0.5/1.0/1.5/2.5/4/6/10 (lam_x=0.1, DLC80 mu=0.5):
%    尾段|SW|峰 23.1/29.7/19.6/**15.9**/34.2/17.2/20.0 deg
%    |r|>mu*g/Vx  10.6/10.6/11.2/**10.0**/17.5/18.1/18.8 %
%  τ>=4 后纵向太弱, 压不住原文的横摆约束; τ=2.5 是拐点。
%  τ 是外环速度 P 增益(Fx = m*dV/τ)的倒数, 属控制律参数, 可调。
Constraints.Zg_tpid      = 0.3;      % s, 送给 CarSim 速度 PID 的设定值一阶滤波时间常数 (0 = 不滤)
%  扫 0/0.15/0.3/0.5/0.8: 油门 0<->1 翻转 9/3/3/1/0 次, 速率RMS 4.56/2.28/2.55/0.90/0.67,
%  尾段|SW|峰 7.5/5.5/5.7/13.5/13.8。取 0.3: 翻转 9->3 且**其余指标全数不退**。
%  注: 即使滤波, 油门仍会占着轨 —— PID 比例带只有 10 km/h, 而限速目标
%  本来就要在 60~80 km/h 之间真实摆动; 要去掉饱和只能减小降速幅度本身。
%  CarSim 的速度 PID: Kp=0.1, Ki=0.005, 输出饱和 [0,1] 且**无制动通道**
%  => 比例带只有 10 km/h, 设定值一跳油门就在 0/1 之间砼砼响。
%  这里只滤**送给 PID 的跟随目标**(驾驶员/动力系统带宽有限, 不可能响应
%  10 Hz 的设定值抖动); 约束执行走的 Fx_dem 仍用**未滤波**的 Vx_lim,
%  所以制动响应一点不延迟, 忠实度不受影响。
Constraints.Zg_cmax      = 3;        % 修正因子 c 的上限 (1 = 不修正, 等价于 ayfull=0)
Constraints.Zg_ayfull    = 0;        % 0 = 原文稳态近似 a_y ~ Vx*r; 1 = 完整 a_y = dVy + Vx*r
%  原文 Eq.(26e) 的 |r| <= mu*g/Vx 是从 a_y <= mu*g 代入稳态近似 a_y ~ Vx*r 得来的。
%  实测(DLC80, mu=0.5): Vx*r 峰 / 真实 Ay 峰 = 1.58(MPC) / 1.67(ZENG) / 1.83(PMPC),
%  而三者真实 Ay 峰均为 0.42 g, 恰好顶在轮胎能力 0.4275 g 上——谁都没超附着极限。
%  即该判据在换道瞬态里比真实 a_y 严 1.6~1.8 倍, ZENG 让出的 ~8 km/h 全数
%  来自这个近似误差, 不是物理限制。ayfull=1 只是把近似换回完整表达式,
%  **物理约束 a_y <= mu*g 一字未改**。做忠实复现时用 0。
Constraints.Zg_kovs      = 1.3;      % 预测 r 的物理可达性剪切: r <= kovs*mu_eff*g/Vx
%  kovs = 横摆瞬态超调系数。稳态上 a_y <= mu_eff*g => r <= mu_eff*g/Vx;
%  转向入弯瞬态 r 会超调。实测全程 |r| 峰 0.3225 rad/s, 而当时
%  mu_eff*g/Vx = 0.251 => 超调 1.28 倍, 故取 1.3。
%  这**不是放松约束**: 被剪掉的那些预测值(峰 0.90 rad/s <=> a_y=1.56 g)
%  在 mu=0.5 上根本不可能发生, 是线性化模型的外推失真。
Constraints.Zg_Npv       = 12;       % 预瞄窗口(预测步数)。pmpc_step 里按 min(Zg_Npv, Np) 截断,
%  故 Ts=0.01/Np=10 时实际为 10 步 = 0.1 s(整个时域); 原 Ts=0.05 时 12 步 = 0.6 s。
%  实测: 预测 horizon-max 相对**实际未来 0.6 s 内真实最大 |r|** 均值高估
%  1.24 倍, 62% 的拍高估>5%, 90 分位 2.04 倍; 预测峰 0.90 rad/s 而实测 |r|
%  全程不超 0.3225。高估集中在时域末端(线性化模型在转向反向处外推失真),
%  t=2.41/4.41 s 两处把 Vset 砸到 38/40 km/h。缩短窗口即可避开。
Constraints.Zg_apv       = 0;        % 预瞄可达性折扣系数 (0 = horizon-max 逐位等价, 1 = 完全可达性)
Constraints.Zg_dVdb      = 0.3;      % m/s,  死区
Constraints.QPA_lamd     = 0.003;    % 分配 QP 的增量惩罚 (0 = 关, 与原行为逐位等价)
%  制动分配器本来就差这一项: MR 分配器 func_QPA_CDC 早已有 rho_c=0.05 的
%  同类项(注释里写的诊断与此处一致), 当时漏了制动路。
%  锚点: MR 那边 rho_c/||Bd'Bd|| = 0.05/0.30 = 0.17; 制动这边
%  ||Bb'Bb||=5.1, (M_ref/T_ref)^2=20.4, 同比例即 lam_d ~ 0.04。
%  扫 0/0.003/0.006/0.01/0.02/0.03/0.04/0.06 (ZENG, R2=1e1):
%    dTb RMS 2335/2096/2295/1911/1927/1843/1812/1861  (最多 -22%)
%    |r|>mu*g/Vx 10.0/6.9/9.4/10.0/11.9/12.5/13.8/15.6 (单调变差)
%  最终取 lam_d=0.003 (R2 不动): ①② 两个控制器**没有任何回退**——
%  dTb -8%/-10%/-35%, 尾段|SW|峰 5.6->4.6 / 15.9->13.1, |r|超限 10.0->6.9%。
%  备选(未采用) R2=3e2+lam_d=0.01: dTb -13%、|r|超限 5.0%, 但尾段|SW|峰
%  回升到 20.8 deg, 把上一轮 30.7->15.9 的修复推了回去, 故否决。
Constraints.Gov_lamx     = 0.1;      % 分配器里纵向相对横摆的优先级 (<1 => 横摆优先)
%  lam_x 已归一化: =1 时横摆残差与纵向残差同价。0.1 = 比纵向高一个
%  数量级, 沿用 Hajiloo et al. (VSD 2023) 每降一级优先级差一个数量级的约定。
%  实测(dbg_zg): t=6.48 s 限速器触发瞬间左/右制动力矩 372/795 N·m,
%  这个不均直接注入横摆扰动, 控制器再用转角去纠 -> 尾段摆动。
%  lam_x 扫 0.5/0.3/0.1/0.03/0.01/0.003: 尾段|SW|峰 30.7/24.5/23.1/24.7/24.2/26.7,
%  0.3 以下已饱和 —— 剩下的不是分配不均造成的, 而是制动**幅值**, 故配合 Zg_tau。
%  否定的方案: 给 Vx_lim 加释放限速率(只限上升沿)。实测 a_rise=10/5/2 m/s^2
%  尾段|SW|峰 112/80/205 deg, 反而爆炸: 按住限速值把偶发制动脉冲变成了
%  近乎连续的制动(差动制动 RMS 543->826), 而制动本身就是横摆扰动源。

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
MPCParameters.AFS_add = wsget('PMPC_AFSADD', 0);
MPCParameters.sigma_budget = 0;
MPCParameters.fx_couple    = 0;
Constraints.Tb_max = 3000;
Constraints.dTb_rate_max = 2e4;
Constraints.Vymax   = 2;             
%  后轴侧偏界(第二层稳定包络)。2026-09-26 实测(CarSim, mu=0.5 DLC): 后轮 |Fy|/Fz 在
%  4.5-5 deg 已走平(0.424), 即 5 deg 已贴近真实饱和。放宽到 5.5/6 deg 的对照(改进.md 18u):
%  跟踪几乎不变, 质心侧偏角峰 +15%/+32%, 故保持 5 deg。
%  PMPC_ARLIM_MU = 1: arlim = (mu/MU_REF)*参考表在后轮静载下的峰值角 (相似缩放, 改进.md 18z)。
if wsget('PMPC_ARLIM_MU', 0)
    Ccp   = load('TireCarpet_265_75R16.mat');
    Fzr0w = VehiclePara.m*VehiclePara.g*VehiclePara.lf/VehiclePara.L/2;     % 后轮单轮静载
    [~, ipk] = max(interp1(Ccp.FZ(:), Ccp.FY.', Fzr0w, 'linear'));
    Constraints.arlim = deg2rad(Ccp.AL(ipk))*VehiclePara.mu/Ccp.MU_REF;
    clear Ccp Fzr0w ipk
else
    Constraints.arlim = deg2rad(5);
end
Constraints.phimax  = deg2rad(4);    
Constraints.dphimax = deg2rad(50);          
Constraints.epsimax = deg2rad(8);  
Constraints.epsilon_r = 10*deg2rad(5);
Constraints.epsilon_alpha = 10*deg2rad(5);
Constraints.epsilon_LTR = 3;
Constraints.epsilon_e   = 8;
% 松弛变量上界 = eps_ub_scale * 各自的 epsilon_*。Inf = 无界(原行为)。
% 目的(任务 #7 / 缺陷 B2): 原来 eps_ub = inf, 约束违反量在理论上无界,
% 论文里只能说"软约束", 说不出任何违反上限。改为有限值后可以陈述
% 「约束违反被 eps_ub_scale 倍的参考松弛所界定」。
% 取值依据(DLC 实测): scale=10 与 3 的结果与 Inf **逐位相同**(最大差 0),
% scale=1 开始咬合。即实际需要的松弛量在 1~3 倍之间。
% 取 10 —— 留 3 倍以上余量, 实质上保住原设计的"QP 恒可行", 同时得到有限上界。
%  2026-09-26 改回 Inf(无界): 论文命题 1(松弛 QP 逐点可行)要求松弛无上界。
%  上面记录的实测(10、3 与 Inf 逐位相同)说明该界从未咬合, 改回不影响结果。
Constraints.eps_ub_scale = Inf;

% MPC 参数
MPCParameters.Nu = 3;
MPCParameters.Ne = 8;
%  PrioMode = 1: 论文 III-D/III-E 的三层车辆级目标(2026-09-26):
%    L1 防侧翻与车道安全 > L2 横向稳定 > L3 路径跟踪与执行器代价; 执行器不排序。
%  L1/L2 各一个违反量 s1/s2(归一化约束行的最大违反), 安全权重按候选裕度在线给定
%  (式 weight_rule); gamma_DB 只出现在 L3 的制动介入代价 W_b*gamma_DB 里, 不是优先级层。
%  只用于 PMPC; MPC/ZENG 是对比方法, 保持原 8 松弛结构。
%  必须放在 MPCParameters(编译期常量)里, 各处的 if 才折得掉。
MPCParameters.PrioMode = 0;
%  权重规则参数(论文式 weight_rule, 系数记为 vartheta): w2 = vartheta*F/delta2,
%  w1 = vartheta*(F+w2[-d2]+ +rho[-d2]+^2)/delta1, 再截断到 [prio_wmin, prio_wmax(l)]。
%  rho = gamma_reg*W_b(s1、s2、gamma 共用, 与论文式 MPC_problem 一致), 只影响不可达时的
%  折中, 不影响精确性(引理 1 对任意 rho>=0 成立)。
Constraints.prio_vartheta = 2;
Constraints.prio_wmin  = 1;
Constraints.prio_wmax  = [1e10; 1e8];     % [w1 上限; w2 上限], w1 须能压过 w2*|delta2|/delta1
%  状态 [Vy r phi dphi ey epsi Md_a] (2026-09-25 加阻尼器时延状态 Md_a, 论文 III-A):
%  tau_d*dMd_a/dt = -Md_a + Md_c; 侧倾与 LTR 只经 Md_a 受阻尼器作用。
MPCParameters.Nx = 7;
MPCParameters.Ny = 7;
%  tau_d: 暂用 MR 文献值占位(tau_MR), 待 CDC 阶跃电流试验出查表 T(v,dI) 后替换
MPCParameters.tau_d = Constraints.tau_MR;
%  被控对象侧的阻尼器执行器模型(一阶滞后作用在"力在上下界间的位置"上, 保耗散):
%  1 = 开(与预测模型一致), 0 = 关(阻尼力当拍直达 CarSim, 即 2026-09-25 前的行为)
Constraints.dmp_act_on = wsget('PMPC_DMP_ACT', 1);
%  预测步长 T_p = 0.05 s (触发子系统每 T_c = 0.01 s 重解一次, 滚动时域), N_p = 20 (1.0 s), N_c = 6。
%  N_p 扫描(改进.md 18z, Fiala-C0 标定): 16/20/24/28 -> e_y RMS 0.3031/0.2531/0.2772/0.2709,
%  出车道 5.8%/0/0/0。跟踪在 N_p = 20 取极值; 再加长跟踪略退而稳定性继续改善(beta 峰 3.49->2.45),
%  但 certA 由 69.1% 单调降到 53.0%。取 20。
%  2026-09-27 对照(改进.md 18z): 试过 T_p = T_c = 0.01 以消除"预测里的转向权限只有真实速率
%  1/5", 但 N_p = N_c = 10 只剩 0.1 s 预瞄, DLC 直接失稳(出车道 56%, beta 峰 15 deg);
%  拉到 N_p = 80 (同为 0.8 s 预瞄) 仍差于本配置, 差在控制时域时间 N_c*T_p 由 300 ms 缩到 100 ms。
%  工作区 PMPC_TS / PMPC_NS / PMPC_NC 可覆盖(对照用)。
MPCParameters.Ts = wsget('PMPC_TS', 0.05);
MPCParameters.Ns = wsget('PMPC_NS', 20);
MPCParameters.Np = MPCParameters.Ns;
MPCParameters.Nc = wsget('PMPC_NC', 6);
%  历史对照(改进.md 18r/18v, 均在 T_p = 0.05 下): N_p 12(0.6 s) -> 16(0.8 s) 使换道滞后
%  158-218 ms 降到 108 ms; N_c = 16 跟踪不变但最坏耗时约 3.7 倍; 时域内第 2..N_c 步速率限
%  按 T_p 放宽则跟踪与侧偏全面变差。
% 轮胎线性化方案 (见 极限工况轮胎模型与MPC三种实现方案.md):
%   0 = 方案二: 当前工作点线性化一次, 预测时域内冻结 (Ataei 2020)
%   1 = 方案三: 沿名义预测轨迹逐节点线性化 (LTV-MPC)
%
% 现状(2026-09-01): 方案三的**模型精度**已验证更好(DLC: r +15.5%/Vy +21.1%;
% 鱼钩配合驾驶员转角外推后 r +11.4%/phi +14.5%), 实时代价基本为零(均值 +0.1%,
% 最坏情况反而好 3.3 倍)。但**闭环**上是权衡: DLC 防侧翻更好(|LTR|峰 -9.1%)
% 而路径误差 +6%; 鱼钩防侧翻更差(抬轮 +46%)。
% 判断: 现有权重全是在方案二的模型上调出来的, 换模型需重调权重才公平;
% 另有一处 v1 简化未做(Fz/Vel 时域内冻结 => 模型看不见制动的载荷转移代价,
% 横向变准而纵向仍瞎, 控制器会过度制动)。
% 故默认保持 0(不引入回归), 方案三作为已验证的可选项保留。
MPCParameters.LTV_on = 0;
MPCParameters.QPSolver = wsget('PMPC_QPSOLVER', 1);   % 0 = quadprog, 1 = KWIK(mpcqpsolver)
%  2026-09-17 阶段 S 已切成 KWIK(默认 1)。实测对比 quadprog:
%    安全类指标全部持平或改善(|LTR|峰 三个控制器全降), ey RMS 变化 <0.1%
%    QP 失败次数 1/1/4 -> 0/0/0
%    PMPC 最大迭代 2500(撞上限) -> 28; ZENG -> 77; 均值 71.8 -> 2.5
%  原因是 KWIK 的热启动传的是**活动集**, 比 quadprog 的原始点热启动强得多。
%  且 R2018a 的 quadprog 本就不支持 codegen, HIL 上只能用这条路。
%  R2018a 的 quadprog 不支持 codegen(R2020a 才加), HIL 必须用 KWIK。
%  见 PLAN_HIL迁移方案.md §1。默认 0 保证与基准逐字节一致。
MPCParameters.Ts_exec = 0.01;   % 触发周期，速率限制按它缩放
%  第一预测步是否按 Ts_exec 离散(非均匀网格, 见 func_DynamicalModel / 规划器 Tk)。
%  0 = 原行为(全部按 Ts); 1 = 第一步 Ts_exec、其余 Ts。2026-09-26 抖动对照用。
MPCParameters.first_Tc = wsget('PMPC_FIRSTTC', 0);
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
MPCParameters.Ncons_env = wsget('PMPC_NCONS_ENV', MPCParameters.Np);   % 工作区可覆盖(对照试验用)

%% ---- 运行期打印开关 (阶段 E) ----
%  0 = 关掉每拍的 fprintf 和求解器 warning。HIL 上必须关, 理由是:
%    生成的代码里 fprintf/warning 是纯累赘, 实时目标上还可能阻塞。
%  ⚠️ 不是为了提速 —— 2026-09-18 实测: 关掉打印后 CarSim 的
%     RTIME 是 1.039/1.065/2.129 (MPC/ZENG/PMPC), 开着是 1.098/1.056/2.116,
%     差异在运行间噪声范围内(±1%)。PMPC 那 2.13 倍实时是**真实计算开销**,
%     要靠限迭代/生成代码解决, 不是靠少打几行字。
%  放在 MPCParameters 里而不是 Constraints 里, 是因为 MPCParameters 属于 Pm,
%  在块里是 coder.Constant —— Verbose=0 时整个打印分支在生成代码阶段就被折掉,
%  一行 C 代码都不会留。Constraints 属于状态(S0), 折不掉。
MPCParameters.Verbose = wsget('PMPC_VERBOSE', 1);    

DiscreteModle = 4;  % 1=Euler; 2=Taylor4; 3=FOH; 4=精确 ZOH(expm)
%  2026-09-25 由 2 改 4: 时延行 -1/tau_d, Ts/tau_d = 3.3 时 Taylor4 给出的
%  e^{-Ts/tau} = 0.036 被近似成 2.19 > 1(发散), 必须精确离散。论文 III-A 写的也是 ZOH。
%% ---- 对比模式 ----
%  1 = 本文 PMPC       : 三层车辆级目标 + L3 制动介入代价 gamma_DB (PrioMode=1, Nr=1)
%  2 = baseline MPC   : **权重与 PMPC 完全相同**, 唯一差别是去掉 sigma/gamma (Nr=0)
%      这样对比才能分离出"优先级机制"本身的贡献; 旧版那套单独调过的
%      baseline 权重会把"权重不同"和"机制不同"混在一起。
%  先进对比方法(Zeng 2025)在 baseline MPC 基础上开 Constraints.ZengRho_on。
[md_auto, zg_auto, ds_auto] = func_RunMode();   % 从数据集名自动识别
ContrlMode    = wsget('PMPC_MODE', md_auto);
Constraints.ZengRho_on = wsget('PMPC_ZENGRHO', zg_auto);
ctlStr = {'① 固定权重 MPC (无 sigma/gamma)','② Zeng rho 调权','③ 本文 PMPC'};
if ContrlMode==1, ic=3; elseif Constraints.ZengRho_on, ic=2; else ic=1; end
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
%  #3 改变 QP 维度, 所以放 MPCParameters(编译期常量)。
MPCParameters.abl = wsget('PMPC_ABL', 0) * double(ContrlMode == 1);
if ContrlMode == 1 || ContrlMode == 2
    newP = double(ContrlMode == 1 && MPCParameters.abl ~= 3);
    MPCParameters.PrioMode = newP;                     % PMPC 用新结构, 见上方 PrioMode 说明
    MPCParameters.Nr = 1*newP + 3*double(ContrlMode == 1 && MPCParameters.abl == 3);   % 新结构只留 gamma_DB; 旧结构 3 个; MPC/ZENG 0
    MPCParameters.Ne = 8 - 6*newP;                     % 新结构 s1, s2; 旧结构与 MPC/ZENG 为 8 个松弛
    % Constraints.arlim   = deg2rad(5); 
    VehiclePara.CafHat = -57772.51*2;VehiclePara.CarHat = -53484.51*2;%mu=0.85
    % VehiclePara.CafHat = -53645.94*2;VehiclePara.CarHat = -49663.82*2;%mu=0.3
    Constraints.es = 0.43; Constraints.emax= 0.5*(Constraints.Roadwidth-VehiclePara.tf)-Constraints.es;  
    Constraints.LTR_lim = 0.8; 
    CostWeights.Q1=0; CostWeights.Q2=0; 
    CostWeights.Q3=1e2; CostWeights.Q4=0; 
    % Q5 随轮胎表换成 CarSim 实测 carpet 后重调: 2e3 -> 2e4。
    % 新表的前轴刚度更高、Flim 更紧(mu_eff=0.78 而非 0.9), 控制器打方向更少,
    % 需提高路径权重补回。扫描 2e3/6e3/2e4/6e4 后 2e4 的 |LTR|峰与侧倾峰最好。
    CostWeights.Q5=wsget('PMPC_Q5', 2e4); CostWeights.Q6=wsget('PMPC_Q6', 2e2);   % 工作区可覆盖 (新模型基准下重标定, 18z)
    if InitialParams.FishhookMode
        % 纯防侧翻: 无路径、无偏航角速度参考
        CostWeights.Q1=0;    CostWeights.Q2=0;
        % Q3 扫描 2e3~2e8: 侧倾峰 4.98->4.41 deg, 抬轮 4.06->2.19%, 代价是制动占空
        % 65.6->92.8%。取 2e6 —— 归一化空间里必须守住 Q3 < W3(=5e6):
        % 跟踪权重不能超过约束违反权重, 否则等于说「侧倾大一点」比「违反 LTR 约束」
        % 更严重。2e6 是保持该序关系的最大值, 且已拿到大部分收益
        % (侧倾峰 -7.9%, 抬轮 -23%, |LTR|>0.99 归零)。
        CostWeights.Q3=5e6;  CostWeights.Q4=2e3;   % 换实测轮胎表后由 2e6 重调
        CostWeights.Q5=0;    CostWeights.Q6=0;
    end
    % 执行器增量代价。分母统一为常量后(任务 #6), 这三个数字含义相同:
    % 「用满该执行器每拍速率能力时的代价」, 所以必须量级可比。
    % 原 R2=1e9 意味着一次制动增量抵十亿次 AFS 增量, 把差动制动掐死了。
    % 鱼钩扫描 1e9/1e7/1e5/1e3/1e1: 1e7 以下完全平坦, 侧倾峰 6.70->4.81 deg,
    % |LTR|>0.99 占比 6.88%->0.31%, 抬轮 12.2%->4.4%, 车速反而更快。
    CostWeights.R1=wsget('PMPC_R1', 1e0); CostWeights.R2=1e1; CostWeights.R3=5;   % R1 工作区可覆盖(抖动对照)
    % R2 扫 1e1/3e1/1e2/3e2/1e3 (ZENG, DLC80 mu=0.5): 对轮缸力矩抖振
    % (dTb RMS 2335/2384/2419/2254/2370) **几乎无效** —— 因为 R2 罚的是
    % 上层 Δ(MFx), 而抖振产生在下层分配 QP; 但它显著改善约束满足:
    % |r|>mu*g/Vx 10.0->5.6%, 侧倾峰 1.605->1.551, |ey|峰 0.5016->0.4756。
    CostWeights.S1=0; CostWeights.S2=0; CostWeights.S3=0;
    % 终端代价系数(末节点 Q 的倍数)。1 = 关闭。见 func_CostWeighting... 的说明。
    CostWeights.Qf_scale = 1;
    % sigma 价格: MR 最便宜(不消耗轮胎力、不掉车速), DB 最贵(掉车速+占纵向摩擦)
    % V3 扫描 0/1/10/1e3/1e5: 制动占空 74->97%, 侧倾峰 6.12->7.11deg, 见任务 #2
    CostWeights.V1=100; CostWeights.V2=wsget('PMPC_V2', 8e4);CostWeights.V3=1;   % V2 工作区可覆盖(DB 启用阈值对照试验)
    % Delta-gamma 惩罚。不直接给裸数字 —— 它的物理含义是优先级的切换时间常数:
    %   gamma 子问题 min Nc*V*g^2 + Wdg*(g-g_prev)^2  s.t. g >= g_req
    %   无约束最优 g* = Wdg*g_prev/(Nc*V+Wdg), 故每拍记忆保持率 = Wdg/(Nc*V+Wdg)。
    %   令其 = exp(-Ts_exec/tau_gamma)  =>  Wdg = rho/(1-rho)*Nc*V。
    %   这样 Wdg 自动随 V 缩放, 设计者只需给一个有物理意义的 tau_gamma。
    CostWeights.tau_gamma = wsget('PMPC_TAUG', 0);   % s, 优先级切换时间常数; 0 = 关闭迟滞(工作区可覆盖)
    CostWeights.Wdg=[0 0 0];     % 仅在 tau_gamma=0 时生效的直接指定
    CostWeights.W1=5e4; CostWeights.W2=5e4; 
    % epsilon 权重(归一化后含义: 松弛达到各自参考值 epsilon_* 时的代价)。
    % W4 原为 5e7, 即「位置约束违反」的代价是「LTR 约束违反」的 10 倍 ——
    % 对一篇防侧翻论文, 这个次序是反的。DLC 扫描 5e4~5e7 显示 W4 不敏感
    % (ey_RMS 0.248~0.256, 全在散度带内), 且它是 diag(H) 最大项(1.5e8),
    % 降下来同时消除次序倒置、改善条件数。取 W4 = W3 = 5e6。
    CostWeights.W3=5e6; CostWeights.W4=5e6;
else
    error('Invalid ContrlMode');
end

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
P.C_table = C_table;
P.Cf0 = Cf0;
P.Cr0 = Cr0;
P.TireF = TireF;
P.TireR = TireR;

%% ---- 阶段 D: 再打包成 MATLAB Function block 要的两个子结构 ----
%  Pm = 运行中不变的参数 (块里声明为 Scope=Parameter, build 时固化)
%  S0 = 跨拍状态的初值 (块里用 persistent, 首拍用它初始化)
%  上面那 16 个扁平字段保留不动 —— S-function 那条路还在用。
P.Pm = struct('MPCParameters',MPCParameters, 'CostWeights',CostWeights, ...
              'DiscreteModle',DiscreteModle, 'ContrlMode',ContrlMode, ...
              'Reftraj',Reftraj, 'Cf0',Cf0, 'Cr0',Cr0, 'C_table',C_table, ...
              'TireF',TireF, 'TireR',TireR);
P.S0 = struct('InitialParams',InitialParams, 'WarmStart',WarmStart, 'rho',rho, ...
              'VehiclePara',VehiclePara, 'Constraints',Constraints, ...
              'cert', nan(24, 1));   % 记录用(不参与控制), 布局随 PrioMode 而异, 见 cert_replay.m

% 没有输出参数时, 顺手写进 base 工作区
if nargout == 0
    assignin('base','PMPC_P',P);
    fprintf('  PMPC_P 已写入工作区 (%d 个字段)\n', numel(fieldnames(P)));
    clear P
end
end
