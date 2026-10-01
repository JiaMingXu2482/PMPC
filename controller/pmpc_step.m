function [sys, St, i_cmd] = pmpc_step(u, Pm, St, h_plant, x_plant, v_plant, a_plant) %#codegen
%PMPC_STEP  每拍的控制运算 (原 cq32019mpc 的 mdlOutputs 实体)
%
%   注: 变量名用 Pm/St 而非 P/S —— body 里 'S' 已被代价权重矩阵占用(实测踩坑)。
%   u : 54x1 从 CarSim 来的测量 (顺序 = CarSim Export 列表)
%   Pm: 参数 (setup_pmpc 产出, 运行中不变)
%   St: 跨拍状态 (InitialParams / WarmStart / rho), 进出都要
%   h_plant/x_plant/v_plant/a_plant: 1 kHz plant 在拍首采样的状态
%     (控制器角序 [L1;R1;L2;R2]; h_plant 8x4, 其余 4x1)
%   sys : 54x1 输出
%   i_cmd : 4x1 电流指令 [A], 控制器角序; plant 侧 ZOH 保持到下一拍
%
%   多速率架构 (2026-09-29): NLCSNN 正模型(plant)在 1000 Hz 独立积分,
%   拥有隐状态 h; 本函数只做 100 Hz 逆解, 不再推进 h。
%
%   阶段 C+D (2026-09-17): 从 S-function 里抽出来, 目标是让 MATLAB Function block
%   里只写一行 `[sys,St] = pmpc_step(u,Pm,St);`。原 mdlOutputs 用 t/x 为 0 次, 故不传。

%% ---- 参数(只读) ----
MPCParameters = Pm.MPCParameters;
Nx = coder.const(MPCParameters.Nx);
CostWeights = Pm.CostWeights;
DiscreteModle = Pm.DiscreteModle;
Reftraj = Pm.Reftraj;
TireF = Pm.TireF;
TireR = Pm.TireR;
NLCSNN = Pm.NLCSNN;

%% ---- 既是参数也是状态: 拍内被改写且**跨拍保留** ----
%  VehiclePara 的 CbarF/CbarR/CafTan/CarTan (在线切线刚度)
%  Constraints 的 Zeng_bL/bR/rmax (鞍点界)
%  原来它们是 global, 改写会留到下一拍; 做成本地副本会改变行为(已实测踩坑)
VehiclePara = St.VehiclePara;
Constraints = St.Constraints;
prio_on = isfield(Constraints,'PrioModeRT') && (Constraints.PrioModeRT == 1);

%% ---- 跨拍状态 ----
InitialParams = St.InitialParams;
WarmStart = St.WarmStart;
rho = St.rho;

sys = zeros(54,1);
%初始化sys输出信号
if InitialParams.InitialGapflag == 0 
    InitialParams.InitialGapflag = InitialParams.InitialGapflag + 1;
    %  提前返回也必须回写状态(原来是 global, 自动持久; 改成出参后必须显式写)
    St.InitialParams = InitialParams;
    St.WarmStart = WarmStart;
    St.rho = rho;
    St.VehiclePara = VehiclePara;
    St.Constraints = Constraints;
    i_cmd = zeros(4,1);   % 首拍无逆解, plant 保持 0 A
    return; % 早期直接返回，减少缩进
end 
InitialParams.InitialGapflag = InitialParams.InitialGapflag + 1;

% --- 1. 状态估计与预处理 ---
[VehStateMeasured, ParaHAT] = func_StateEstimation(u,VehiclePara);  
Vel = VehStateMeasured.x_dot;
Vy  = VehStateMeasured.y_dot; 
yawrate = VehStateMeasured.Yawrate; 
LTR = ParaHAT.LTR;
alpha_l2=ParaHAT.alpha_l2;
alpha_r2=ParaHAT.alpha_r2;
Roll = ParaHAT.Roll; Rollrate = ParaHAT.Rollrate;

% NLCSNN coordinates and reachable force box, sampled from the 1 kHz plant
% at the tick. Corner order is [L1;R1;L2;R2] (allocation order); the plant
% owns the hidden state h, the controller never integrates it.
DamperContext = struct('x',zeros(4,1), 'v',zeros(4,1), ...
    'a',zeros(4,1), 'h',zeros(8,4), ...
    'F_lo',zeros(4,1), 'F_hi',zeros(4,1), ...
    'F_center',zeros(4,1));
DamperLimits = struct('Fdu',zeros(4,1), 'Fdl',zeros(4,1), ...
    'Fd_center',zeros(4,1), 'external',false);
i_cmd = zeros(4,1);
if NLCSNN.enabled
    DamperContext = func_NLCSNNContext(NLCSNN.net, ...
        x_plant, v_plant, a_plant, h_plant, NLCSNN);
    DamperLimits.Fdu = DamperContext.F_hi;
    DamperLimits.Fdl = DamperContext.F_lo;
    DamperLimits.Fd_center = DamperContext.F_center;
    DamperLimits.external = true;
end

%%
% ---- 仿射轮胎线性化（二维查表，f_bar 与 c_bar 同源）----
%   Fy = f_bar + c_bar*(alpha - alpha_bar) = c_bar*alpha + F_off
%   alpha_bar 用模型自己的定义算，避免依赖 CarSim 的侧偏角符号约定。
% delta_robot: 外部(驾驶员/转向机器人)给的前轮转角; 控制量 u 是其上的增量, 总转角 = delta_robot + u
%  Add 模式 (鱼钩的转向机器人, 或 DLC 的 CarSim 闭环驾驶员模型):
%     CarSim 侧 IMP_STEER_SW = Add, 我们只输出增量, 实测总转角减去上一拍增量即基底。
%  Replace 模式: 自动驾驶, 控制器就是驾驶员, 无外部转角输入。
if MPCParameters.FishhookMode || MPCParameters.AFS_add
    delta_robot = VehStateMeasured.delta_f - InitialParams.U(1);
else
    delta_robot = 0;
end

% ---- 驾驶员转角在预测时域内的外推 ----
%  原来(以及 Ataei 2020 原文)把 delta_robot 在整个时域内按常值保持:
%    "the driver's steering angle is assumed constant during the prediction horizon"
%  鱼钩转向机器人以 720 deg/s 运动, 时域 Np*Ts = 0.6 s, 于是有
%    42.35 deg/s(前轮) * 0.6 s = 25.4 deg 的驾驶员转角完全没进预测模型。
%  后果: 方案三每个节点的 alpha 都建立在"驾驶员不再动方向盘"上, 名义轨迹失准,
%  实测鱼钩下逐节点线性化反而比冻结单工作点更差(Vy -12.6%, r -16.8%)。
%  处理: 用实测速率一阶外推, 速率本身按执行器物理上限限幅并做低通。
dr_seq = delta_robot;                 % 方案二: 时域内保持常值(标量)
if MPCParameters.LTV_on
if isnan(InitialParams.dr_prev)          % codegen: 原为 ~isfield 判首拍
    InitialParams.dr_prev = delta_robot;
    InitialParams.dr_rate = 0;
end
dr_raw  = (delta_robot - InitialParams.dr_prev) / MPCParameters.Ts_exec;
dr_raw  = max(min(dr_raw, Constraints.dlt_rate_max), -Constraints.dlt_rate_max);
a_lp    = 0.3;                                   % 一阶低通, 时间常数约 30 ms
InitialParams.dr_rate = (1-a_lp)*InitialParams.dr_rate + a_lp*dr_raw;
InitialParams.dr_prev = delta_robot;
dr_seq  = delta_robot + InitialParams.dr_rate * (1:MPCParameters.Np).' * MPCParameters.Ts;
dr_seq  = max(min(dr_seq, deg2rad(30)), -deg2rad(30));   % 机械限的宽松兜底
end
ab_f = (Vy + VehiclePara.lf*yawrate)/max(abs(Vel),1) - VehStateMeasured.delta_f;
ab_r = (Vy - VehiclePara.lr*yawrate)/max(abs(Vel),1);
[fb_f, cb_f] = func_TireTable('eval', TireF, ab_f, ParaHAT.Fzf);
[fb_r, cb_r] = func_TireTable('eval', TireR, ab_r, ParaHAT.Fzr);
% ---- 复合滑移: 摩擦椭圆降额 (R1-6, 2026-09-25) ----
%  FY_TIRE_CARPET 是纯侧偏表, 制动/驱动时同一 alpha 下侧向力会下降, 纯侧偏表
%  感知不到(实测后轮 Fx/(mu*Fz) 达 0.62 -> 侧向能力只剩约 78%)。按摩擦椭圆
%  (Hajiloo 2021 T-ITS 式(5)) 每轮降额 xi_w = sqrt(1-(Fx/(mu*Fz))^2), 轴级取
%  Fz 加权平均(表是轴级, 差动制动时两侧 Fx 差别大, 不能用轴合力)。
%  fb、cb 同乘 xi: 仿射模型仍过当前工作点, 截距 F_off 在下面用降额后的值重算。
%  xi 在时域内冻结, 与 Fz、上下界的冻结方式一致。mu 用路面值, 与建表缩放同源。
xi_f = local_xi(ParaHAT.Fx_l1, ParaHAT.Fz_l1, ParaHAT.Fx_r1, ParaHAT.Fz_r1, VehiclePara.mu);
xi_r = local_xi(ParaHAT.Fx_l2, ParaHAT.Fz_l2, ParaHAT.Fx_r2, ParaHAT.Fz_r2, VehiclePara.mu);
fb_f = xi_f*fb_f;   cb_f = xi_f*cb_f;
fb_r = xi_r*fb_r;   cb_r = xi_r*cb_r;
% ---- 切线刚度下限 (trust region) ----
%  方案二(单工作点仿射)的已知风险: c_bar -> 0 时模型认为"转向对侧向力没有影响",
%  预测模型在整个时域里失去控制权限, 控制器随之放弃修正。
%  实测(DLC, 2026-09-01): Station 145~200 m 处后轴 CarTan 掉到名义值的 2%,
%  最后两轴精确归零, 车辆在该段出现明显振荡; 全程前轴 |C|<5%名义 占 10.7%,
%  后轴 19.6%, 后轴精确为 0 占 9.5%。
%  处理: 把 |c_bar| 钳到名义 C0 的一个比例。仿射模型仍过工作点 (ab, fb),
%  只是斜率不许塌到 0 —— 即"承认轮胎饱和, 但不相信它完全失去回正能力"。
%  f_bar 保持查表原值, 截距 F_off 用钳后的 c_bar 重算, 三者保持自洽。
%
%  定位: 这是安全兜底, 不是性能修复。末段振荡的真正原因是转角速率限
%  (见 Constraints.dlt_rate_max_deg)。速率限改成 60 deg/s 之后, 本钳位
%  开/关的差别已落到噪声级(ey_RMS 0.2776 vs 0.2774), 保留是为了防止
%  其它工况下 c_bar 塌到 0 导致预测模型失去控制权限。
kap  = Constraints.Ctan_floor_frac;
cb_f = -max(abs(cb_f), kap*TireF.C0);   % c_bar < 0 (Fy 与 alpha 反号)
cb_r = -max(abs(cb_r), kap*TireR.C0);
VehiclePara.CbarF = cb_f;   VehiclePara.CbarR = cb_r;
VehiclePara.CafTan = cb_f;  VehiclePara.CarTan = cb_r;   % 兼容旧引用
F_off_f = fb_f - cb_f*ab_f - cb_f*delta_robot;           % 已折进 delta_robot
F_off_r = fb_r - cb_r*ab_r;

%% ==== Baseline: 跳过 MPC, 四角输出实测 MR 标称电流下的阻尼力 ===
if InitialParams.BaselineMode
    Ib = InitialParams.BaselineCurrent;
    sys(5) = func_MRDamper(VehStateMeasured.V_l1, Ib);   % Fd_L1 前左
    sys(6) = func_MRDamper(VehStateMeasured.V_l2, Ib);   % Fd_L2 后左
    sys(7) = func_MRDamper(VehStateMeasured.V_r1, Ib);   % Fd_R1 前右
    sys(8) = func_MRDamper(VehStateMeasured.V_r2, Ib);   % Fd_R2 后右
    sys(9) = 0;                                          % AFS 增量
    sys(29) = LTR;      sys(31) = Vel*3.6;
    sys(32) = VehStateMeasured.VxTarget;
    sys(34) = sqrt(Vel^2 + Vy^2);   sys(35) = LTR;
    sys(36) = (rad2deg(alpha_l2)+rad2deg(alpha_r2))/2;
    sys(37) = rad2deg(VehStateMeasured.Yaw);
    sys(38) = VehStateMeasured.beta;
    sys(39) = cb_f;     sys(40) = cb_r;
    sys(41) = Roll;     sys(42) = Rollrate;
    sys(43:46) = [ParaHAT.Fx_l1;ParaHAT.Fx_l2;ParaHAT.Fx_r1;ParaHAT.Fx_r2];
    sys(47:50) = [ParaHAT.Fy_l1;ParaHAT.Fy_l2;ParaHAT.Fy_r1;ParaHAT.Fy_r2];
    sys(51:54) = [ParaHAT.Fz_l1;ParaHAT.Fz_l2;ParaHAT.Fz_r1;ParaHAT.Fz_r2];
    %  提前返回也必须回写状态(原来是 global, 自动持久; 改成出参后必须显式写)
    St.InitialParams = InitialParams;
    St.WarmStart = WarmStart;
    St.rho = rho;
    St.VehiclePara = VehiclePara;
    St.Constraints = Constraints;
    i_cmd = zeros(4,1);   % baseline 不用 NLCSNN, plant 保持 0 A
    return
end

%% ---------- 开始计时  ------------
%  这里原来有 t_Start=tic, 末尾配 toc 量单拍耗时。已移除, 三个原因:
%   1. R2018a 的 MATLAB Coder 不支持 tic/toc, 部署机上直接报
%      "Function 'tic' is not supported for code generation."(R2024b 能过, 一直没暴露)
%   2. 试过用 coder.target 或常量条件把 tic 包起来, MATLAB Function 块一律
%      推不出输出维度(报 "无法确定模块 PMPC_MF 的输出大小和/或类型"), 走不通
%   3. 这个数本来也没参考价值 —— 开发机上解释执行的耗时推不出实时目标上的耗时
%  要量单拍耗时用 time_step.m: 在**外部**循环调 pmpc_step, tic/toc 在函数外面。

%% 路径规划
longActive = MPCParameters.ControllerVariant == 3 ...
    && Constraints.LongCoordMode ~= 0 && ~MPCParameters.FishhookMode;
Vx_pred = Vel*ones(MPCParameters.Np,1);
longFx = 0;
longVset = VehStateMeasured.VxTarget;
projDefault = struct('ey',0,'epsi',0,'Velr',Vel, ...
    'xr',VehStateMeasured.X,'yr',VehStateMeasured.Y, ...
    'psir',VehStateMeasured.Yaw);
Projection = struct('WPIndex',-1,'PrjP',projDefault, ...
    's0',0,'valid',false);
if longActive
    if ~isfinite(St.LongCoord.prev.Vset_prev)
        St.LongCoord.prev.Vset_prev = VehStateMeasured.VxTarget;
    end
    if St.LongCoord.speed_planned > 0
        St.LongCoord.speed_mismatch = Vel-St.LongCoord.Vx_pred(1);
    end
    Projection = func_PathProjection(VehiclePara, ...
        InitialParams.WayPoints_IndexPre,Reftraj,VehStateMeasured);
    [longPlan,longNext] = func_PMPCSpeedCoordinator( ...
        MPCParameters,VehiclePara,Constraints,Reftraj,Projection, ...
        VehStateMeasured,ParaHAT,St.LongCoord.margin_prev,St.LongCoord.prev);
    St.LongCoord.prev = longNext;
    St.LongCoord.diag = longPlan.diag;
    St.LongCoord.Vx_pred = longPlan.Vx_pred;
    St.LongCoord.Fx_request = longPlan.Fx_dem;
    St.LongCoord.speed_actual = Vel;
    St.LongCoord.speed_planned = longPlan.Vx_pred(1);
    St.LongCoord.Vset_pid = longPlan.Vset_pid;
    Vx_pred = longPlan.Vx_pred;
    longFx = longPlan.Fx_dem;
    longVset = longPlan.Vset_pid;
end
if MPCParameters.FishhookMode          % 鱼钩: 无路径
    Kap_dyn = [];  Kap_node = [];
    PrjP   = struct('ey',0,'epsi',0,'Velr',Vel,'xr',VehStateMeasured.X, ...
                    'yr',VehStateMeasured.Y,'psir',VehStateMeasured.Yaw);
    ref_Vy = zeros(MPCParameters.Np,1);
    ref_r  = zeros(MPCParameters.Np,1);
else
    %  2026-09-26: e_y/e_psi 在质心处量; 曲率取路径真值按节点弧长采样 ——
    %  Kap_dyn 进预测模型(每步平均曲率), Kap_node 进车道约束(节点曲率)。见 改进.md 18q。
    if longActive
        [WPIndex,RefU,Kap_dyn,PrjP,Kap_node] = func_RefTraj_LocalPlanning( ...
            MPCParameters,VehiclePara,InitialParams.WayPoints_IndexPre, ...
            Reftraj,VehStateMeasured,ParaHAT,Projection,Vx_pred);
    else
        [WPIndex,RefU,Kap_dyn,PrjP,Kap_node] = func_RefTraj_LocalPlanning( ...
            MPCParameters,VehiclePara,InitialParams.WayPoints_IndexPre, ...
            Reftraj,VehStateMeasured,ParaHAT);
    end
    if WPIndex > 0, InitialParams.WayPoints_IndexPre = WPIndex; end
    ref_Vy = RefU(2:3:end);     
    ref_r  = RefU(3:3:end);
end
% 线性化模型与包络
% 方案三的输入无条件构造(开关关闭时也给预测精度诊断用)
x0_ltv = zeros(Nx,1);
x0_ltv(1:6) = [Vy; yawrate; Roll; Rollrate; PrjP.ey; PrjP.epsi];
if Nx == 7
    x0_ltv(7) = ParaHAT.Md;
end
LTVin = struct('TireF',TireF, 'TireR',TireR,...
'delta_robot',dr_seq,... % Np x 1, 见上方外推
'Fzf',ParaHAT.Fzf, 'Fzr',ParaHAT.Fzr,...
'kap',Kap_dyn,...
'x0',x0_ltv,...
'u0',[VehStateMeasured.delta_f - delta_robot;...
InitialParams.U(2); InitialParams.U(3)],...
'dU',reshape(WarmStart(1:MPCParameters.Nc*MPCParameters.Nu),...
MPCParameters.Nu, MPCParameters.Nc),...
'Ctan_floor_frac',Constraints.Ctan_floor_frac);
if MPCParameters.LTV_on
% 方案三: 名义轨迹由上一拍解移位(热启动)给出, 逐节点重算 alpha -> c_bar/f_bar
if longActive
    [StateSpaceModel] = func_DynamicalModel(VehiclePara, MPCParameters, ...
        VehStateMeasured, DiscreteModle, LTVin, Vx_pred);
else
    [StateSpaceModel] = func_DynamicalModel(VehiclePara, MPCParameters, ...
        VehStateMeasured, DiscreteModle, LTVin);
end
else
if longActive
    [StateSpaceModel] = func_DynamicalModel(VehiclePara, MPCParameters, ...
        VehStateMeasured, DiscreteModle, [], Vx_pred);
else
    [StateSpaceModel] = func_DynamicalModel(VehiclePara, MPCParameters, ...
        VehStateMeasured, DiscreteModle);
end
end
% ---- Zeng 2025 先进对比: 先算 rho 与稳定性边界, 供 Envelope 和代价共用 ----
rho_zeng = 1; Iind_zeng = 0; Ib_zeng = 0; Ir_zeng = 0;
if Constraints.ZengRho_on
[rho_zeng, Iind_zeng, Ib_zeng, Ir_zeng, bL_z, bR_z, rmax_z] = func_ZengRho(...
Vel, VehiclePara.mu, VehStateMeasured.delta_f,...
VehStateMeasured.beta, yawrate, VehiclePara.g);
Constraints.Zeng_bL = bL_z; Constraints.Zeng_bR = bR_z; Constraints.Zeng_rmax = rmax_z;
if Constraints.ZengRho_on == 2, rho_zeng = 1; end % 约束用 Zeng 的, 但不调度
end

[Envelope7,r_ssmax] = func_Envelope(VehiclePara, Constraints, VehStateMeasured);
if Nx == 6
    % func_Envelope is legacy 7-state. Trim its Md_a column and move the
    % no-delay damper moment into the direct LTR input term for MPC.
    Henv = Envelope7.Henv(:,1:6);
    Hsh  = Envelope7.Hsh(:,1:6);
    Hr   = Envelope7.Hr(:,1:6);
    ltrGain = 2/(VehiclePara.m*VehiclePara.g*VehiclePara.tf);
    Or = Envelope7.Or + [0 0 ltrGain; 0 0 -ltrGain];
else
    Henv = Envelope7.Henv;
    Hsh  = Envelope7.Hsh;
    Hr   = Envelope7.Hr;
    Or   = Envelope7.Or;
end
Envelope = struct( ...
    'Henv',Henv, 'Genv',Envelope7.Genv, 'Eenv',Envelope7.Eenv, ...
    'gkap',Envelope7.gkap, 'Hsh',Hsh, 'Gsh',Envelope7.Gsh, ...
    'Hr',Hr, 'Or',Or, 'Gr',Envelope7.Gr, ...
    'sc_sh',Envelope7.sc_sh, 'sc_r',Envelope7.sc_r, ...
    'sc_env',Envelope7.sc_env);
% ---- Zeng 2025: 用 rho 调度稳定性松弛权重 (仅先进对比方法启用) ----
CW = CostWeights;
if Constraints.ZengRho_on
CW.W1 = CostWeights.W1 * rho_zeng; % sigma_s: 横摆角速度松弛
CW.W2 = CostWeights.W2 * rho_zeng; % sigma_s: 侧偏角松弛
end
[Q,R,S,W,V,dFyfmax,dMFxmax,dMdmax,Fyfmax,MFxmax,Mdmax,Mdmin,Tb_u,Fdu,Fdl,Mdnom] = func_CostWeightingRegulation_QuadSlacks(MPCParameters,CW,Constraints,r_ssmax,ParaHAT,VehiclePara,VehStateMeasured,DamperLimits);

%% ==================================================================%
%---------------- quadprog solver compute begin --------------------%

% --- 2. QP问题构建 ---
Nu=MPCParameters.Nu; Ny=MPCParameters.Ny;
Nc=MPCParameters.Nc; Np=MPCParameters.Np; Ne=MPCParameters.Ne;
Nr=MPCParameters.Nr; % 注意这里Nr可能为0 (Case 2)

Fyf_0 = InitialParams.U(1); MFx_0 = InitialParams.U(2); Md_0 = InitialParams.U(3);

% ---- 阻尼器时延状态的初值与投影 (论文 III-C 式 Md_projection, 2026-09-25) ----
% Md_a(k): 由实测阻尼力按 Md = 0.5*[-df df -dr dr]*Fd 合成(func_StateEstimation),
% 再投影到当前速度下的可达区间 [Mdmin, Mdmax]。上一拍指令 Md_0 同样投影:
% 阻尼器在当前速度下出不了区间外的力矩, 投影后 box_relax 不再放宽 CDC 通道,
% 离散一阶滞后是凸组合 -> 整个时域内预测的 Md_a 都落在区间内(满足耗散性),
% 速度全为零时区间退化为 {0}, Md_a 与 Md_0 均被置零。
Md_lo = min(Mdmin, Mdmax); Md_hi = max(Mdmin, Mdmax);
Md_0 = min(max(Md_0, Md_lo), Md_hi);
if MPCParameters.abl == 2
% 消融 PMPC-noSAS: 减振器是标称电流下的被动阻尼, 不可控。控制器的 CDC 通道冻结在
% 当前速度下的被动力矩 Mdnom 上(增量限幅为 0), 与 Fz、上下界一样在时域内冻结。
Md_0 = min(max(Mdnom, Md_lo), Md_hi);
dMdmax = 0;
end

% MPC is deliberately no-delay (six vehicle states). ZENG/PMPC append the
% established aggregate damper moment Md_a for their 15 ms predictor.
zeta = zeros(Nx + Nu,1);
zeta(1:6) = [Vy; yawrate; Roll; Rollrate; PrjP.ey; PrjP.epsi];
if Nx == 7
    Mda_0 = min(max(ParaHAT.Md, Md_lo), Md_hi);
    zeta(7) = Mda_0;
end
zeta(Nx+(1:Nu)) = [Fyf_0; MFx_0; Md_0];

% 预测矩阵
% 方案三时偏置逐节点, 方案二时用当前工作点的常值
if ~StateSpaceModel.off_valid % codegen: 原为 isempty(off)
distIn = [F_off_f; F_off_r];
else
distIn = StateSpaceModel.off;
end
[PSI, THETA, GAMMA, PHI] = func_SystemFurture(MPCParameters, StateSpaceModel, Kap_dyn, distIn);

% 累加矩阵 A_I (用于计算控制量 U = U_prev + A_I * DeltaU)
A_t = tril(ones(Nc)); % 下三角全1
AI  = kron(A_t, eye(Nu));
Ut  = kron(ones(Nc,1), zeta(Nx+(1:Nu))); 


%% ---- 打包传给下层函数的参数 ----
Pred = struct('PSI',PSI, 'THETA',THETA, 'PHI',PHI, 'GAMMA',GAMMA);
Wts = struct('Q',Q, 'R',R, 'S',S, 'W',W, 'V',V);
% ---- AFS 控制量的幅值界: 机械限 + 前轴摩擦圆 ----
% 预测模型 (func_DynamicalModel 第 53 行):
% Fyf = CbF*(vy+lf*r)/vx - CbF*u + F_off_f_eff
% 代入 (vy+lf*r)/vx = ab_f + delta_f 与 F_off_f_eff = fb_f - cb_f*ab_f - cb_f*delta_robot,
% 并记当前工作点 u_op = delta_f - delta_robot, 得到关于当前点的形式:
% Fyf(u) = fb_f + |cb_f|*(u - u_op) <- cb_f<0, -CbF*u = +|cb_f|*u
% 要求 |Fyf| <= Flim 即给出以 u_op 为中心的余量界, 见下。
% 力限用**实测的可实现侧向附着** mu_eff, 不是路面设定的 mu(=0.9)。
% CarSim ERD 实测 |Fy|/Fz 的上限: 轴级 0.751/0.754(前) 0.740/0.698(后),
% 单轮最大 0.785, 合力最大 0.788 —— 轮胎的可实现附着约 0.78, 明显低于
% 路面设定的 0.9(载荷敏感性)。用 0.9 会高估约 15~20% 的可用力。
% 也不用轮胎表的 mu(0.70/0.75) —— 那是为了让 Fiala 的**形状**贴合而拟出的
% 参数, 不是真实饱和水平; 用它会把 AFS 掐死(实测 DLC 路径误差 +54%)。
Flim = 0.95*VehiclePara.mu_eff*xi_f*ParaHAT.Fzf; % 纵向力占用的附着同样扣除, 与 fb_f 的降额一致
Cm = max(abs(cb_f), 5e3);
% DLC(Replace): u 是**总转角**, 界为整车转向上限;
% 鱼钩(Add): u 是 **AFS 增量**, 界为 AFS 叠加电机权限。
% 上一版把两者统一成 AFS_max(5.73deg) 是 09-12
% "改成辅助驾驶"那次改动的残留; 回滚到自动驾驶时漏改了这一处。
% 后果: DLC 下总转角被卡在 5.73deg(方向盘 97.4deg), 仅为应有权限的 41%,
% 实测三个控制器的 |SW| 峰值都精确等于 97.41deg —— 长时间贴顶、反复撞限,
% 造成限幅相位滞后型振荡。
if MPCParameters.FishhookMode || MPCParameters.AFS_add
dlt_mec = Constraints.AFS_max; % AFS 叠加电机权限
else
dlt_mec = min(Constraints.SW_max/VehiclePara.isw, deg2rad(20)); % 整车转向上限
end
% ---- 力限转角界: 以当前工作点为中心表达余量, 不外推到 u=0 ----
% 原写法 dlt_ub=(Flim-Fy0)/Cm 把切线外推到 u=0, 而 Fy0 是幻觉力:
% |Fy0| 轻易超过 Flim, 于是 dlt_ub<0 或 dlt_lb>0, 箱体变单边,
% AFS 只能往一个方向打, 最终被钳死 (2026-08-31 实测 DLC t>5.5s
% SW 恒为 0、横向漂移 -2.97 m; dlt_ub==0 占 6.3%, dlt_lb==0 占 27.6%)。
% 改为以 u_op 为中心: Fyf(u) = fb_f + |cb_f|*(u - u_op)
% => u in [u_op - (Flim+fb_f)/Cm, u_op + (Flim-fb_f)/Cm]
% 余量取 max(.,0), 箱体恒含当前转角; cb_f->0 (轮胎饱和) 时余量发散,
% 力限自动退出、只剩机械限 —— 饱和时切线模型本就说不出话。
u_op = VehStateMeasured.delta_f - delta_robot; % 当前 u 的实测值
head_up = max(Flim - fb_f, 0)/Cm; % 正方向余量
head_dn = max(Flim + fb_f, 0)/Cm; % 负方向余量
if Constraints.afs_box_exact
% 诊断开关 (改进.md 18z): 切线外推在轮胎曲线的凹段偏保守 —— 相似缩放表的
% 小角刚度是旧表的约 1.8 倍, 外推余量中位数只剩约 1 deg。改为查表反解:
% |xi*Fbar(alpha_lim)| = Flim, 余量 = 从当前 ab_f 走到 -/+alpha_lim 的转角。
a_lim = local_alim(TireF, ParaHAT.Fzf, Flim/max(xi_f, 1e-3));
head_up = max(ab_f + a_lim, 0); % u 增 -> ab_f 减, 到 -a_lim 时 Fyf = +Flim
head_dn = max(a_lim - ab_f, 0);
end
dlt_ub = min( dlt_mec, u_op + head_up);
dlt_lb = max(-dlt_mec, u_op - head_dn);
% 再保证含 0: gamma->0 要能退到 u=0, 且"不打方向"绝不违反摩擦极限
dlt_ub = max(dlt_ub, 0);
dlt_lb = min(dlt_lb, 0);

% LTR 约束里 a_y 的常值部分 (F_off_f*cos(delta) + F_off_r)/m 移到右端
bta_l = 2/(VehiclePara.m*VehiclePara.g*VehiclePara.tf);
kap_g = VehiclePara.m*VehiclePara.h_TL - VehiclePara.ms*VehiclePara.h_S2R;
ay_off = (F_off_f*cos(VehStateMeasured.delta_f) + F_off_r)/VehiclePara.m;
Envelope.Gr = Envelope.Gr - bta_l*kap_g*ay_off*[1; -1];

% ---- 前轴摩擦耦合系数 (方案 B) ----
% MFxmax 用的是**实测**的 Fy 算摩擦余量: Fx_u = sqrt((mu*Fz)^2 - Fy^2)。
% 但 AFS 打算改变 Fyf —— 这部分没被计入, 即模型以为制动能力比实际多。
% 一阶修正: dFx_u/dFy = -Fy/Fx_u, 而 AFS 使前轴侧向力变化
% dFy_axle = |cb_f| * (u_AFS - u_op), 单个前轮取一半。
% 差动制动的横摆力矩 MFx = (tf/2)*Fx, 故可用力矩的削减量为
% kappa * |u_AFS - u_op|, kappa = (tf/2)*(|Fy_f|/Fx_u_f)*|cb_f|/2
% 取轴平均并对称施加(保守: 不会高估可用能力), 见 func_BuildQPConstraints 3c。
Fy_f_avg = 0.5*(abs(ParaHAT.Fy_l1) + abs(ParaHAT.Fy_r1));
Fz_f_avg = 0.5*(ParaHAT.Fz_l1 + ParaHAT.Fz_r1);
Fxu_f = sqrt(max((VehiclePara.mu*Fz_f_avg)^2 - Fy_f_avg^2, 1));
kappa_fx = (VehiclePara.tf/2) * (Fy_f_avg/Fxu_f) * abs(cb_f)/2;

% 优先级因子的惯性/耦合都要用到上一拍的 gamma, 初始化必须在 Lim 之前
% codegen: gamma 已在 setup_pmpc 里初始化为 [0;0;1], 无需惰性创建

% 各预测节点处的路径曲率, 供四角点弯道修正 -kappa*a^2/2 用(动力学用的是每步平均曲率 Kap_dyn)。
% 鱼钩模式下为空(无路径), 与 func_SystemFurture 一样补零。
kap_env = zeros(MPCParameters.Np,1);
nkap_ = min(numel(Kap_node), MPCParameters.Np);
kap_env(1:nkap_) = Kap_node(1:nkap_);

if prio_on
eps_ub_ = zeros(MPCParameters.Ne, 1);
eps_ub_(1:2) = inf; % 仅 s1/s2 参与 PMPC 新优先级层
else
eps_ub_ = Constraints.eps_ub_scale *...
[Constraints.epsilon_r*ones(2,1); Constraints.epsilon_alpha*ones(2,1);...
Constraints.epsilon_LTR*ones(2,1); Constraints.epsilon_e*ones(2,1)];
end
Lim = struct('kap',kap_env,...
'Fyfmax',Fyfmax, 'MFxmax',MFxmax, 'Mdmax',Mdmax, 'Mdmin',Mdmin,...
'Mdnom',Mdnom,...
'eps_ub', eps_ub_,...
'sigma_budget',MPCParameters.sigma_budget,...
'gamma_prev',InitialParams.prevstate.gamma,...
'kappa_fx',kappa_fx, 'u_op',u_op,...
'fx_couple',MPCParameters.fx_couple,...
'dFyfmax',dFyfmax, 'dMFxmax',dMFxmax, 'dMdmax',dMdmax,...
'dlt_ub',dlt_ub, 'dlt_lb',dlt_lb);

%% ---- 代价 ----
lambda_L1 = 1.0; % L1 精确罚强度；0 = 退回纯二次罚

% ---- 优先级因子的惯性 ----
gamma_prev = InitialParams.prevstate.gamma;
if isfield(CostWeights,'tau_gamma') && ~isempty(CostWeights.tau_gamma) && CostWeights.tau_gamma > 0
rho_g = exp(-MPCParameters.Ts_exec / CostWeights.tau_gamma);
Wdg_v = rho_g/(1-rho_g) * MPCParameters.Nc *...
[CostWeights.V1, CostWeights.V2, CostWeights.V3];
else
Wdg_v = CostWeights.Wdg;
end
W_dgamma = diag(Wdg_v); % Delta-gamma 惩罚, 见 mdlInitializeSizes

[H, f] = func_BuildQPCost(MPCParameters, Constraints, Pred, Wts,...
zeta, AI, Ut, ref_Vy, ref_r, lambda_L1,...
gamma_prev, W_dgamma);

%% ---- 约束 ----
[A_cons, b_cons, lb, ub, umax, umin] = func_BuildQPConstraints(...
MPCParameters, Constraints, Envelope, Pred, AI, Ut, zeta, Lim);

%% ---- 优先级认证与安全权重 (论文 III-E, 仅 PMPC) ----
% 解 QP 之前由候选 z^(上一拍解后移、投影进 Z)算裕度 delta_1/delta_2 与 F(z^),
% 按式 weight_rule 给出 w1, w2 写进 f 的 s1, s2 分量; 同时判定定理 1 (a)(b)(c)。
pc = nan(14,1); dUc = zeros(Nu*Nc,1); gc = 0; fb_used = 0;
if prio_on
[wpr, pc, dUc, gc] = func_PriorityCert(MPCParameters, Constraints, Pred, Wts,...
zeta, Ut, ref_Vy, ref_r, H, f, A_cons, b_cons, WarmStart, umin, umax, Lim);
f(Nu*Nc + (1:2)) = wpr;
end

%% ---- 求解 ----
[x_opt, exitflag, delta_U_first, rho_val, epsilon_val, WarmStart, cert] =...
func_SolveMPCQP(H, f, A_cons, b_cons, lb, ub, WarmStart, MPCParameters, Constraints);
% 求解失败(如超迭代上限)且 delta_1, delta_2 > 0 时, 施加候选的第一拍输入:
% 它在预测上满足第一、二层约束(论文注 rem:compute)。其余情况沿用原做法(增量为零)。
if prio_on && exitflag ~= 1 && pc(1) > 0 && pc(2) > 0
delta_U_first = dUc(1:Nu);
rho_val = [1; gc; 1];
gtmp = ones(Nr,1);
gtmp(local_db_gamma_idx(Constraints, Nr)) = gc;
WarmStart = func_WarmStart_shiftHorizon([dUc; zeros(Ne,1); gtmp], MPCParameters);
fb_used = 1;
end
% 在线证书 cert(论文 III-E)只做记录, 不参与控制。
% MATLAB Function 状态必须固定尺寸；统一布局为 36x1:
% [先验认证14; 后验证书11; 松弛8; gamma_DB1; exitflag; 兜底标志]。
if prio_on
% PMPC: 14 先验认证 + 11 后验证书 + 8 松弛 + gamma_DB + 两个状态标志
St.cert = [pc; cert; epsilon_val; rho_val(2); double(exitflag); fb_used];
else
% 其它控制器没有先验认证，前 14 项用 NaN 占位；gamma 只记录 DB 项。
St.cert = [nan(14,1); cert; epsilon_val; rho_val(2); nan(2,1)];
end

if exitflag == 1
rho = diag(rho_val); % 更新全局变量供绘图使用
InitialParams.prevstate.gamma = rho_val(:); % 供下一拍的 Delta-gamma 惩罚
end


% 更新控制量
InitialParams.U(1) = zeta(Nx+1) + delta_U_first(1); % Fyf
InitialParams.U(2) = zeta(Nx+2) + delta_U_first(2); % MFx
InitialParams.U(3) = zeta(Nx+3) + delta_U_first(3); % Md

Fyf_next = InitialParams.U(1);
MFx_next = InitialParams.U(2);
Md_next = InitialParams.U(3);
%% ---- 预测输出轨迹 Y (ZENG 纵向控制要用预测 r, 故提到分配之前) ----
if exitflag == 1
    Y   = PSI*zeta + THETA*x_opt(1:Nu*Nc) + PHI*GAMMA;
    rho = diag(rho_val);   % 原为 x_opt 的末 3 位: Nr<3 时取到的是松弛量(rho 只记录, 不参与计算)
else
    Y   = PSI*zeta + PHI*GAMMA;      % 忽略控制增量项
end
if longActive
    St.LongCoord.qp_exitflag = exitflag;
    if all(isfinite(Y))
        roadMargin = inf;
        for p = 1:MPCParameters.Np
            yp = Y((p-1)*Ny+(1:Nx));
            residual = Envelope.Genv + kap_env(p)*Envelope.gkap ...
                - Envelope.Henv*yp;
            roadMargin = min(roadMargin,min(residual));
        end
        St.LongCoord.margin_prev = roadMargin;
    else
        St.LongCoord.margin_prev = -inf;
    end
end

%% ============ Zeng 方法的纵向控制 (仅 ZENG 控制器) ============
%  把 Zeng 的稳定性约束 |r| <= mu*g/Vx 解为速度上限 Vx <= mu*g/|r|,
%  用预测时域内最大 |r| (保守), 再经速度误差转成减速度需求。
%  减速度受**摩擦菱形**限幅 (Wischnewski 2023 Eq.10a): 横向顶满时自动归零,
%  避免抢走正需要的横向附着。执行走差动制动分配器(唯一有制动权限的通路)。
Fx_dem = 0;   Vset_zg = VehStateMeasured.VxTarget;   Vset_pid = Vset_zg;
if ~longActive && (Constraints.ZengRho_on || Constraints.LongLim_diag) ...
        && Constraints.ZengLong_on && exitflag == 1
    %  取**预测时域内最大** |r| —— 实测必需。
    %  原文 Eq.(26e) 是逐步约束且无需额外预瞄, 因为其 MPC **内部就有纵向自由度**
    %  (u 含 F_xf/F_xr), 预测时域里速度是决策变量, 自然会提前减速。
    %  本项目用外环近似, 必须自己补这个提前量。
    %  实测改用当前 |r| 的后果: ey RMS 0.195->0.311, 越界 0->9.4%,
    %  |r|超限 11%->40%, 尾段 |SW| 峰 31->198 deg (失控)。
    %  摩擦菱形给出的**当前可用纵向减速度** (Wischnewski 2023 Eq.10a),
    %  横向顶满时自动归零。既用于限幅制动需求, 也用于下面的可达性折扣。
    ay_meas = ( (ParaHAT.Fy_l1 + ParaHAT.Fy_r1)*cos(VehStateMeasured.delta_f) ...
              + (ParaHAT.Fy_l2 + ParaHAT.Fy_r2) ) / VehiclePara.m;
    ax_bar  = VehiclePara.mu_eff*VehiclePara.g;
    ay_bar  = ax_bar;                                     % 纯横向能力(同附着圆)
    ax_av   = ax_bar * max(1 - abs(ay_meas)/ay_bar, 0);

    %  ---- 可达性折扣的速度上限 ----
    %  原文 Eq.(26e) 是**逐步**约束: 第 k+i 步的 |r| 约束的是第 k+i 步的车速。
    %  直接取 horizon-max 等于把 0.6 s 内最坏的一步立刻施加到**当前**车速上,
    %  过度保守。正确的放松是可达性条件——当前速度只要**来得及**在第 i 步
    %  之前减到那个界就合法:
    %      V(k) <= min_i [ mu*g/|r(k+i)| + a_brk*(i*Ts) ],  i = 0..Np
    %  a_brk 不是旋钮: 就是上面的 ax_av。横向顶满时 ax_av->0, 公式自动
    %  退化回 horizon-max —— 没有富余附着可用来减速时, 本来就必须已经够慢。
    %  Zg_apv = 0 时 min_i(mu*g/r_i) = mu*g/max_i(r_i), **逐位等价于原 horizon-max**。
    %  实测过的另一个极端(只用当前 |r|, 无预瞄): ey RMS 0.195->0.311,
    %  越界 0->9.4%, |r|超限 11%->40%, 尾段 |SW| 峰 31->198 deg (失控)。
    r_pv   = max(abs(Y(2:Ny:end)), Constraints.Zg_rfloor);      % Np x 1
    r_pv   = r_pv(1:min(MPCParameters.Zg_Npv, numel(r_pv)));      % 只用前 Npv 步
    r_att  = Constraints.Zg_kovs * VehiclePara.mu_eff*VehiclePara.g / max(Vel,1);
    r_pv   = min(r_pv, r_att);                                  % 物理可达性剪切

    %  ---- 稳态近似的修正因子 ----
    %  约束 a_y <= mu*g 写成对 Vx*r 的界时, 应乘上 c = |Vx*r| / |a_y|。
    %  c>1 说明稳态近似高估了真实侧向加速度(换道瞬态 dVy 与 Vx*r 反号),
    %  c<1 时自动**收紧**。取 1 即退化为原文。
    c_pv = ones(size(r_pv));
    if MPCParameters.Zg_ayfull
        vy_pv = Y(1:Ny:end);  rs_pv = Y(2:Ny:end);       % 状态 1=Vy, 2=r
        dvy   = [ (vy_pv(1)-Vy)/MPCParameters.Ts ; diff(vy_pv)/MPCParameters.Ts ];
        ay_pv = dvy + Vel*rs_pv;                          % 完整侧向加速度
        cc    = abs(Vel*rs_pv) ./ max(abs(ay_pv), 0.1);
        cc    = min(max(cc, 0.5), Constraints.Zg_cmax);
        c_pv  = cc(1:numel(r_pv));
        c_now = min(max(abs(Vel*yawrate)/max(abs(ay_meas),0.1), 0.5), Constraints.Zg_cmax);
    else
        c_now = 1;
    end
    t_pv   = (1:numel(r_pv))' * MPCParameters.Ts;               % 预瞄时间
    a_brk  = Constraints.Zg_apv * ax_av;
    V_cand = [ c_now*VehiclePara.mu*VehiclePara.g / max(abs(yawrate), Constraints.Zg_rfloor); ...
               c_pv.*(VehiclePara.mu*VehiclePara.g) ./ r_pv + a_brk*t_pv ];
    Vx_lim = max(min(V_cand(:)), Constraints.Zg_Vmin);   % (:) 消除维度歧义, 见 func_RefTraj_LocalPlanning                         % m/s
    Vset_zg = min(VehStateMeasured.VxTarget, Vx_lim*3.6);                   % km/h
    dV = Vel - Vset_zg/3.6;              % 制动用**未滤波**值
    if dV > Constraints.Zg_dVdb
        Fx_dem = VehiclePara.m * min(dV/Constraints.Zg_tau, ax_av);
    end

    %  ---- 只给 PID 那一路加一阶滞后 ----
    Vset_pid = Vset_zg;
    if Constraints.Zg_tpid > 0
        if isnan(InitialParams.prevstate.Vpid)   % codegen: 原为 ~isfield 判首拍
            InitialParams.prevstate.Vpid = Vset_zg;
        end
        kf = min(MPCParameters.Ts_exec / Constraints.Zg_tpid, 1);
        Vset_pid = InitialParams.prevstate.Vpid + kf*(Vset_zg - InitialParams.prevstate.Vpid);
        InitialParams.prevstate.Vpid = Vset_pid;
    end
end
if longActive
    Fx_dem = longFx;
    Vset_pid = longVset;
end

%% ==================================================================%
%--------------------- QPA control allocation -----------------------%
% --- 5. 底层分配 ---
% Replace 输出总转角; Add 只输出增量, 否则外部转角被叠加两次
if MPCParameters.FishhookMode || MPCParameters.AFS_add
    [Steer_Wheel, delta_wheel] = func_AFS(VehiclePara,VehStateMeasured,MPCParameters,ParaHAT, Fyf_next);
    delta_wheel = delta_robot + Fyf_next;   % 差动制动分配要用总转角
else
    [Steer_Wheel, delta_wheel] = func_AFS(VehiclePara,VehStateMeasured,MPCParameters,ParaHAT, delta_robot + Fyf_next);   % 总转角
end
[Tb_L1,Tb_L2,Tb_R1,Tb_R2,exitflag_DB] = func_QPA_DB( ...
    VehiclePara,InitialParams,Constraints,ParaHAT,MFx_next, ...
    delta_wheel,Tb_u,Fx_dem,MPCParameters.Verbose);
if longActive
    St.LongCoord.allocation_exitflag = exitflag_DB;
    St.LongCoord.prev.allocationFailed = exitflag_DB <= 0;
    St.LongCoord.Fx_achieved = ...
        ((Tb_L1+Tb_R1)*cos(delta_wheel)+Tb_L2+Tb_R2)/VehiclePara.rt;
    if St.LongCoord.Fx_request > 1
        St.LongCoord.prev.achievedRatio = min(max( ...
            St.LongCoord.Fx_achieved/St.LongCoord.Fx_request,0),1);
    else
        St.LongCoord.prev.achievedRatio = 1;
    end
    St.LongCoord.brake_active = ...
        Tb_L1+Tb_R1+Tb_L2+Tb_R2 > 1;
end
InitialParams.prevstate.Tb = [Tb_L1;Tb_R1;Tb_L2;Tb_R2];
[Fd_L1,Fd_L2,Fd_R1,Fd_R2,Md_real,exitflag_Fd]  = func_QPA_CDC(VehiclePara,InitialParams,VehStateMeasured,Md_next,Fdu,Fdl,MPCParameters.Verbose);
Fd_cmd = [Fd_L1;Fd_R1;Fd_L2;Fd_R2];
if MPCParameters.abl == 2
    if NLCSNN.enabled
        % noSAS keeps the force-space midpoint; no fabricated nominal current.
        Fd_cmd = DamperContext.F_center;
    else
        Ib_nom = 2.0;
        if isfield(Constraints,'I_nom') && ~isempty(Constraints.I_nom), Ib_nom = Constraints.I_nom; end
        Fd_cmd = [func_MRDamper(VehStateMeasured.V_l1, Ib_nom); ...
                  func_MRDamper(VehStateMeasured.V_r1, Ib_nom); ...
                  func_MRDamper(VehStateMeasured.V_l2, Ib_nom); ...
                  func_MRDamper(VehStateMeasured.V_r2, Ib_nom)];
    end
end
InitialParams.prevstate.Fd = Fd_cmd;   % command force, QPA warm start
if NLCSNN.enabled
    % Inverse only: the 1 kHz plant owns h and applies i_cmd over the next
    % 10 ms. F_pred is the controller's force prediction at the tick state
    % for logging (sys 5:8); the force that actually reaches CarSim is
    % F_plant from the plant subsystem -- do not wire F_pred to CarSim.
    [i_cmd, F_pred, InitialParams.prevstate.nlcsnn.i_prev] = ...
        func_NLCSNNApply(NLCSNN.net, DamperContext, Fd_cmd, ...
        InitialParams.prevstate.nlcsnn.i_prev, NLCSNN);
    Fd_L1 = F_pred(1);  Fd_R1 = F_pred(2);  Fd_L2 = F_pred(3);  Fd_R2 = F_pred(4);
    Bd_act = 0.5*[-VehiclePara.ldf, VehiclePara.ldf, ...
                  -VehiclePara.ldr, VehiclePara.ldr];
    Md_real = Bd_act*F_pred;
elseif Constraints.dmp_act_on && MPCParameters.abl ~= 2
    % Explicit rollback path for the legacy first-order actuator.
    [Fd_act, InitialParams.prevstate.s_act] = func_DamperActuator( ...
        Fd_cmd, Fdl, Fdu, InitialParams.prevstate.s_act, ...
        MPCParameters.Ts_exec, MPCParameters.tau_d);
    Fd_L1 = Fd_act(1);  Fd_R1 = Fd_act(2);  Fd_L2 = Fd_act(3);  Fd_R2 = Fd_act(4);
else
    Fd_L1 = Fd_cmd(1);  Fd_R1 = Fd_cmd(2);  Fd_L2 = Fd_cmd(3);  Fd_R2 = Fd_cmd(4);
end
% if abs(ParaHAT.LTR)>=0.8
%     disp('abs(LTR)已达到0.8，停止仿真');
%     ssSetStopRequested(ssGetSFunctionBlock, 1);
% end
t_Elapsed = 0;    % 见上: 计时已移出本函数, 见 time_step.m


InitialParams = func_ReportStatus(InitialParams, exitflag, PrjP, Vel, t_Elapsed, MPCParameters.Verbose);



%% ==========================  输出信号  ============================%
% --- 7. 系统输出 ---
% 确保 epsilon_val 和 rho_val 存在
eps_out = zeros(4,1);
if prio_on
    %  [s2 s2 s1 s1]: 横摆、后轴侧偏同属第二层, LTR、车道同属第一层(归一化违反量)
    eps_out = [epsilon_val(2); epsilon_val(2); epsilon_val(1); epsilon_val(1)];
elseif length(epsilon_val) >= 8
    eps_out(1) = epsilon_val(1); % yaw rate
    eps_out(2) = epsilon_val(3); % alpha_r
    eps_out(3) = epsilon_val(5); % LTR
    eps_out(4) = epsilon_val(7); % ey
end
% 
% % [Steer_Wheel, state_sw] = func_realTimeSignalFilter(Steer_Wheel,200,15,InitialParams.prevstate.sw);
% % InitialParams.prevstate.sw=state_sw;%必须要滤波
% 



%% ============ LTR 三路对照（记录用，不参与控制） ============
Npdc = 6;               % 预测提前量（步）
[LTR_real, LTR_calc, LTR_Npdc] = func_LTRDiagnosis( ...
        VehiclePara, MPCParameters, VehStateMeasured, ParaHAT, ...
        Y, x_opt, exitflag, AI, Ut, Md_next, Npdc);

% 诊断通道 14:16 共用固定接口：ZENG 输出其自适应稳定性指标，
% MPC/PMPC 保持原来的执行器优先级因子。
diag_rho = rho_val;
if Constraints.ZengRho_on
    diag_rho = [rho_zeng; Ib_zeng; Ir_zeng];
end

sys = [ Tb_L1;  Tb_L2;  Tb_R1;  Tb_R2; 
        Fd_L1;  Fd_L2;  Fd_R1;  Fd_R2; 
        Steer_Wheel;
        eps_out(1);   
        eps_out(2);         
        eps_out(3);   
        eps_out(4);   
        diag_rho(1);   % ZENG: rho_zeng；其它: rho_AFS
        diag_rho(2);   % ZENG: I_beta； 其它: rho_DB
        diag_rho(3);   % ZENG: I_r；    其它: rho_CDC
        MFxmax; MFx_next; -MFxmax;
        Mdmax;  Md_next;   Mdmin;
        Fyfmax; Fyf_next; -Fyfmax;
        r_ssmax; yawrate; -r_ssmax;
        % fval;
        LTR_real;
        LTR_Npdc;
        % LTR_calc;
        % PrjP.ey;
        % rad2deg(PrjP.epsi);
        % VehStateMeasured.V_r2;
        % Vd_R211
        % ref_r(1);
        % ref_Vy(1);
        Vel*3.6;
        Vset_pid;                   % -> CarSim 速度 PID 的 Set_Vx (滤波后; tpid=0 时等于 Vset_zg)
        PrjP.yr;sqrt(Vel^2+Vy^2);LTR;(rad2deg(alpha_l2)+rad2deg(alpha_r2))/2;rad2deg(VehStateMeasured.Yaw);VehStateMeasured.beta;
        % Xo;
        % Yo;
% eta1;eta2
        VehiclePara.CafTan; 
        VehiclePara.CarTan;   % <-- 在线切线刚度(负值)，原为常数 CafHat/CarHat
        ParaHAT.Roll;    
        ParaHAT.Rollrate;
        ParaHAT.Fx_l1;ParaHAT.Fx_l2;ParaHAT.Fx_r1;ParaHAT.Fx_r2;
        ParaHAT.Fy_l1;ParaHAT.Fy_l2;ParaHAT.Fy_r1;ParaHAT.Fy_r2;
        ParaHAT.Fz_l1;ParaHAT.Fz_l2;ParaHAT.Fz_r1;ParaHAT.Fz_r2;
        ];



   



%% ------------------------------------------------------------------

%% ---- 回写状态 ----
St.InitialParams = InitialParams;
St.WarmStart = WarmStart;
St.rho = rho;
St.VehiclePara = VehiclePara;
St.Constraints = Constraints;
end


function a_lim = local_alim(T, Fz, Ft)
%LOCAL_ALIM  |Fy| 首次达到 Ft 的侧偏角 (>0); Ft 超过饱和力时取饱和角。
%  Fiala 可闭式反解: F = muFz*[1-(1-u)^3], u = C*tan(a)/(3*muFz)
%    => u = 1 - (1 - min(max(Ft/muFz, 0), 1))^(1/3),  a = atan(3*muFz*u/Cst)
z    = min(max(Fz, T.fzlo), T.fzhi);
Cst  = max(Fz, 0) * (T.cq(1)*z*z + T.cq(2)*z + T.cq(3));
muFz = max(Fz, 0) * (T.mq(1)*z*z + T.mq(2)*z + T.mq(3));
Cst  = Cst / (1 + T.kcs*Cst);
Cst  = max(Cst, 1.0);   muFz = max(muFz, 1.0);
u    = 1 - (1 - min(max(Ft/muFz, 0), 1))^(1/3);
a_lim = atan(3*muFz*u/Cst);
end


function xi = local_xi(Fx_l, Fz_l, Fx_r, Fz_r, mu)
%LOCAL_XI  轴级摩擦椭圆降额系数: 每轮 sqrt(1-(Fx/(mu*Fz))^2) 的 Fz 加权平均
%  抬轮(Fz->0)时该轮权重自然趋零; 纵向已饱和的轮 xi_w = 0。
Fz_l = max(Fz_l, 1);   Fz_r = max(Fz_r, 1);
xl = sqrt(max(1 - (Fx_l/(mu*Fz_l))^2, 0));
xr = sqrt(max(1 - (Fx_r/(mu*Fz_r))^2, 0));
xi = (Fz_l*xl + Fz_r*xr)/(Fz_l + Fz_r);
end

function idx = local_db_gamma_idx(Constraints, Nr)
idx = 2;
if isfield(Constraints,'GammaDBIndex') && ~isempty(Constraints.GammaDBIndex)
    idx = round(Constraints.GammaDBIndex);
end
idx = min(max(1, idx), max(1, Nr));
end
