function [Q,R,S,Wshl,Wact,dFyfmax,dMFxmax,dMdmax,Fyfmax,MFxmax,Mdmax,Mdmin,Tb_u,Fdu,Fdl,Mdnom] = func_CostWeightingRegulation_QuadSlacks(MPCParameters,CostWeights,Constraints,r_ssmax,ParaHAT,VehiclePara,VehStateMeasured)

%% -------- 参数/符号 --------
    Np  = MPCParameters.Np;
    Ny  = MPCParameters.Ny;
    Nu  = MPCParameters.Nu;
    Nc  = MPCParameters.Nc;
    Ts  = MPCParameters.Ts;

    Lf  = VehiclePara.lf;
    Lr  = VehiclePara.lr;
    tf  = VehiclePara.tf;
    tr  = VehiclePara.tr;
    rt  = VehiclePara.rt;
    mu  = VehiclePara.mu;
    isw = VehiclePara.isw;
    d_f = VehiclePara.ldf; %左右减振器间距
    d_r = VehiclePara.ldr;
    k1  = VehiclePara.k1;   % 上界段1斜率
    k2  = VehiclePara.k2;   % 上界段2斜率
    b   = VehiclePara.b;    % 上界段2偏置
    k3  = VehiclePara.k3;    % 下界斜率
    v0  = VehiclePara.v0;     % 速度分段点
    CafHat = VehiclePara.CafHat;
    Fzf = ParaHAT.Fzf;

    Vym     = Constraints.Vymax;
    pm      = Constraints.phimax;
    dpm     = Constraints.dphimax;
    em      = Constraints.emax;
    epm     = Constraints.epsimax;
    ep_r   = max(Constraints.epsilon_alpha, 1e-8);
    ep_ar  = max(Constraints.epsilon_alpha,1e-8);
    ep_ltr = max(Constraints.epsilon_LTR, 1e-8);
    ep_e   = max(Constraints.epsilon_e,   1e-8);

%% -------- 0) 归一化基准常量 (只依赖车辆与执行器规格, 运行中不变) --------
%  原实现把 R2/S2/S3/Q2 的分母取成"瞬时可达值", 后果有三:
%    (a) 抬轮时 MFxmax=0 => R2=Inf => H 非有限 => QP 失败(曾 799 次崩 137 次);
%    (b) 同一个权重数字在不同时刻/不同工况代表不同物理含义, 权重无法迁移;
%    (c) diag(H) 跨 12.6 个数量级 => cond(H)=1e13 => 解只剩 3 位有效数字(缺陷 A4)。
%  正确划分: 约束(bounds)负责"此刻能做到什么", 代价归一化负责"我们在意的尺度"。
%  下面全部改为从车辆模型算出的常量。见 README_架构缺陷与救治方案 任务 #6。
    g_  = VehiclePara.g;
    m_  = VehiclePara.m;
    L_  = Lf + Lr;

    % 前轴最大侧向力: 静态前轴载荷 x 附着系数
    Fzf_st  = m_*g_*Lr/L_;
    Fyf_ref = mu * Fzf_st;

    % 差动制动最大横摆力矩: 取"单侧轮胎摩擦上限"与"制动器力矩上限"的较小者
    %  摩擦: mu*m*g/2 是单侧总垂向载荷可用的纵向力
    %  制动: 2*Tb_max/rt 是同侧两轮的制动力
    %  实测校核(载170): 该式给 7055 N*m, 瞬时 MFxmax 峰值 7052 N*m —— 吻合。
    MFx_ref = min( mu*m_*g_/2, 2*Constraints.Tb_max/rt ) * (tf/2);

    % MR 可调制的侧倾力矩幅度: 在参考减振器速度下"最硬-最软"的力差 x 力臂
    if isfield(Constraints,'Vd_ref') && ~isempty(Constraints.Vd_ref)
        Vd_ref = Constraints.Vd_ref;
    else
        Vd_ref = 0.5;                      % m/s, 剧烈工况下的典型减振器速度
    end
    dF_ref  = (k2*Vd_ref + b) - k3*Vd_ref; % Vd_ref > v0, 上界走段2
    Md_ref  = 0.5*(d_f + d_r) * dF_ref;

    % 最大横摆角速度: mu*g/V。用固定基准车速, 否则 Q2 会 ~1/V^2 地随工况漂移
    if isfield(Constraints,'Vel_ref') && ~isempty(Constraints.Vel_ref)
        Vel_ref = Constraints.Vel_ref;
    else
        Vel_ref = 25;                      % m/s (90 km/h), 归一化基准, 不是工况车速
    end
    r_ref   = mu*g_/Vel_ref;

    % 轮胎侧向/法向
    Fy_L1 = ParaHAT.Fy_l1;  Fy_R1 = ParaHAT.Fy_r1;
    Fy_L2 = ParaHAT.Fy_l2;  Fy_R2 = ParaHAT.Fy_r2;
    Fz_L1 = ParaHAT.Fz_l1;  Fz_R1 = ParaHAT.Fz_r1;
    Fz_L2 = ParaHAT.Fz_l2;  Fz_R2 = ParaHAT.Fz_r2;

    % 后桥驱动/阻滞力矩（若有）
    Td_L1 = ParaHAT.Td_L1;  Td_R1 = ParaHAT.Td_R1;
    Td_L2 = ParaHAT.Td_L2;  Td_R2 = ParaHAT.Td_R2;

    % —— 减振器活塞速度（注意大小写）
    V_fl = VehStateMeasured.V_l1;  % Front-Left
    V_fr = VehStateMeasured.V_r1;  % Front-Right
    V_rl = VehStateMeasured.V_l2;  % Rear-Left
    V_rr = VehStateMeasured.V_r2;  % Rear-Right

%% -------- 1) 单轮纵向力/制动矩上限：摩擦椭圆 + 执行器极限 + 驱动矩抵消 --------
    clip = @(x) max(0, x);
    
    % ① 按摩擦椭圆得到每个轮在当前 Fy 下剩余的 Fxf 可用上限 (≥0)
    FxfL_u = sqrt( clip((mu*Fz_L1)^2 - Fy_L1^2) );
    FxfR_u = sqrt( clip((mu*Fz_R1)^2 - Fy_R1^2) );
    FxrL_u = sqrt( clip((mu*Fz_L2)^2 - Fy_L2^2) );
    FxrR_u = sqrt( clip((mu*Fz_R2)^2 - Fy_R2^2) );
    
    % ② 由纵向力上限换算成“轮端制动矩”上限（摩擦受限）
    Tb_fl_u_fric = FxfL_u * rt;
    Tb_fr_u_fric = FxfR_u * rt;
    Tb_rl_u_fric = FxrL_u * rt;
    Tb_rr_u_fric = FxrR_u * rt;
    
    % ③ 执行器物理上限（例如：3000 N·m），注意应≥0
    Tb_act_cap = Constraints.Tb_max;   % ← 按硬件标定填写（也可做成输入/参数）
    Tb_fl_u_act = Tb_act_cap;
    Tb_fr_u_act = Tb_act_cap;
    Tb_rl_u_act = Tb_act_cap;
    Tb_rr_u_act = Tb_act_cap;
    
    % ④ 取两者更小者作为“未经驱动抵扣”的制动能力上限
    Tb_fl_u_raw = min(Tb_fl_u_fric, Tb_fl_u_act);
    Tb_fr_u_raw = min(Tb_fr_u_fric, Tb_fr_u_act);
    Tb_rl_u_raw = min(Tb_rl_u_fric, Tb_rl_u_act);
    Tb_rr_u_raw = min(Tb_rr_u_fric, Tb_rr_u_act);
    
    % ⑤ 若后轮存在驱动矩 Td_*，会占用纵向附着圈，需“抵扣”
    %    注意：抵扣后不能为负（负数意味着此刻无法产生制动矩）
    Tb_fl_max = max(0, Tb_fl_u_raw - max(0, Td_L1));        % 前轮一般无驱动，Td=0
    Tb_fr_max = max(0, Tb_fr_u_raw - max(0, Td_R1));
    Tb_rl_max = max(0, Tb_rl_u_raw - max(0, Td_L2));   % 仅抵扣“正向驱动”
    Tb_rr_max = max(0, Tb_rr_u_raw - max(0, Td_R2));
    
    % ⑥ 暴露给下层分配的“每轮可用制动矩上限”
    Tb_u.Tb_L1 = Tb_fl_max;
    Tb_u.Tb_R1 = Tb_fr_max;
    Tb_u.Tb_L2 = Tb_rl_max;
    Tb_u.Tb_R2 = Tb_rr_max;

    % 由四轮最大制动矩推最大可用附加横摆矩（保守：取正负中的较小绝对值）
    % 为产生最大正横摆力矩：左侧车轮不制动，右侧车轮全力制动
    % 即：Tb_fl = 0, Tb_fr = Tb_fr_max, Tb_rl = 0, Tb_rr = Tb_rr_max
    delta  = VehStateMeasured.delta_f;
    if abs(delta) < 0.15
        MFxmax_pos = (tf/(2*rt)) * ( Tb_fl_max + Tb_rl_max );          % 小角近似
        MFxmax_neg = (tf/(2*rt)) * ( -Tb_fr_max - Tb_rr_max );
    else
        MFxmax_pos = (tf/(2*rt)) * ( Tb_fl_max*cos(delta) + Tb_rl_max ) ...
                   - (Lf*sin(delta)/rt) * (Tb_fl_max + 0);
        MFxmax_neg = (tf/(2*rt)) * ( -Tb_fr_max*cos(delta) - Tb_rr_max ) ...
                   - (Lf*sin(delta)/rt) * (0 + Tb_fr_max);
    end
    MFxmax = min(abs(MFxmax_pos), abs(MFxmax_neg));
    %  速率限是执行器属性, 不该正比于瞬时可达值 —— 原式在抬轮(MFxmax=0)时
    %  把 DB 的速率限也压成 0, 等于把控制量冻住。改用常量基准, 语义不变:
    %  "0.2 s 走完整个可达范围"(ESC 建压时间量级)。
    %  2026-09-25: 按执行周期 Ts_exec 缩放, 与 AFS(dFyfmax)、CDC(dMdmax) 同源。
    %  原来用预测步 Ts=0.05, 而 DeltaU 每 Ts_exec=0.01 施加一次 -> DB 实际
    %  速率是声明值("0.2 s 走完量程")的 5 倍, 与论文 |Du| <= udot_max*T_c 不符。
    if isfield(MPCParameters,'Ts_exec') && ~isempty(MPCParameters.Ts_exec)
        Ts_db = MPCParameters.Ts_exec;
    else
        Ts_db = Ts;
    end
    dMFxmax = MFx_ref * Ts_db / 0.2;

%% -------- 2) Ddelta 的幅值/速率上限 -------
    %  控制量为前轮转角修正 Ddelta。
    %  幅值：由主函数按「仿射模型预测力不超摩擦圆」给出非对称界(Lim.dlt_ub/lb)，
    %        这里只给一个对称兜底值（机械极限）。
    %  速率：直接在角度空间给出，按执行周期缩放 —— 这是力空间做不到的
    %        （原实现实测转角速率 1463 deg/s，声明 72 deg/s）。
    Fyfmax  = Constraints.SW_max / isw;                       % rad, 兜底对称界
    if isfield(Constraints,'dlt_rate_max') && ~isempty(Constraints.dlt_rate_max)
        dlt_rate = Constraints.dlt_rate_max;                  % rad/s @前轮
    else
        dlt_rate = deg2rad(300);                              % Ataei 量级
    end
    if isfield(MPCParameters,'Ts_exec') && ~isempty(MPCParameters.Ts_exec)
        Ts_r = MPCParameters.Ts_exec;
    else
        Ts_r = Ts;
    end
    %  Ts_r = Ts_exec = 0.01 (触发周期), 而预测步 Ts = 0.05, 两者差 5 倍。
    %  曾怀疑「按 Ts_exec 限 DeltaU 会让 MPC 低估自身权限 5 倍」, 试过两种改法:
    %    (x) DeltaU(1) 按 Ts_exec、k>=2 按 Ts  -> 计划出现前 50ms 冻结的折线, 更差
    %    (b) 全按 Ts, 施加时只走 DeltaU(1)/5   -> 计划过激进, 最差
    %  实测三者的方向盘速率峰值都精确等于 720 deg/s(物理上限): 滚动时域每个预测步
    %  内重解 5 次, 保守的时域界并不妨碍实际轨迹达到满滑移率, 反而起阻尼作用。
    %  结论: 保持全部按 Ts_exec。详见 README_架构缺陷与救治方案 2026-09-01。
    dFyfmax = dlt_rate * Ts_r;                                % rad/步
    
%% -------- 3) Md 的幅值/速率上限 -------
    % ---  上/下界力-速模型（分段）---
    Gu = @(v) sign(v) .* ( (abs(v) <= v0) .* (k1.*abs(v)) + ...
                           (abs(v) >  v0) .* (k2.*abs(v) + b) );
    Gl = @(v) k3 .* v;
    Fdu=zeros(4,1);Fdl=zeros(4,1);
    % ---  四支减振器在当前速度下的可达上/下界力 ---
    Fdu(1) = Gu(V_fl);  Fdl(1) = Gl(V_fl);
    Fdu(2) = Gu(V_fr);  Fdl(2) = Gl(V_fr);
    Fdu(3) = Gu(V_rl);  Fdl(3) = Gl(V_rl);
    Fdu(4) = Gu(V_rr);  Fdl(4) = Gl(V_rr);
    
    % ---  由上/下界力构造“最大/最小”可用抗侧倾力矩 ---
    %  M_d = Bd_v'*Fd，Bd_v 各分量有正有负；线性函数在箱约束上的极值必须
    %  按分量符号分别取 ub/lb（正系数取 ub、负系数取 lb），不能四个都取 Gu。
    %  另：Gu/Gl 均为奇函数，v<0 时 Gu 反而是下界，故先按数值排序。
    %  旧写法仅在四轮速度严格反对称（纯侧倾）时才碰巧正确；一旦混入
    %  垂向跳动/俯仰就会系统性低估可用力矩（纯 heave 时给出 0）。
    Bd_v  = 0.5*[-d_f; d_f; -d_r; d_r];      % 与 func_QPA_CDC 的 Bd 同序同号
    Fd_lo = min(Fdl, Fdu);
    Fd_hi = max(Fdl, Fdu);
    Mdmax = sum( max(Bd_v.*Fd_lo, Bd_v.*Fd_hi) );
    Mdmin = sum( min(Bd_v.*Fd_lo, Bd_v.*Fd_hi) );

    % ---  sigma_MR 的中心: 标称电流下的被动侧倾力矩  ---
    %  MR 是半主动件, 断电仍出力, 所以它的"不作用"不是 Md=0, 而是
    %  保持标称电流 I_nom (= baseline 用的电流) 的被动阻尼。
    %  func_BuildQPConstraints 用它把 gamma_CDC 的含义改成"调制幅度",
    %  gamma_CDC=0 即退回被动。见 README_架构缺陷与救治方案 任务 #2。
    if isfield(Constraints,'I_nom') && ~isempty(Constraints.I_nom)
        I_nom = Constraints.I_nom;
    else
        I_nom = 2.0;                       % A, 与 InitialParams.BaselineCurrent 同值
    end
    Fd_nom = [func_MRDamper(V_fl, I_nom); func_MRDamper(V_fr, I_nom); ...
              func_MRDamper(V_rl, I_nom); func_MRDamper(V_rr, I_nom)];
    Mdnom  = Bd_v.' * Fd_nom;
    Mdnom  = min(max(Mdnom, min(Mdmin,Mdmax)), max(Mdmin,Mdmax));   % 夹进可达区间
 
    % ---  变化率上限  ---
    
    %  MR 的力矩变化率 = 可调制幅度 / 响应时间。原写死 (500/0.03)*Ts, 其中
    %  500 N*m 没有来源; 改用第 0 节的 Md_ref (由阻尼器上下界模型算出)。
    %  另: 原来 MR 用 Ts=0.05 而 AFS 用 Ts_exec=0.01 缩放, 同一种「每预测步
    %  增量上限」差 5 倍且无依据, 这里统一用 Ts_r(与 dFyfmax 同源)。
    %  两处修正几乎抵消: 2328/0.03*0.01 = 776 vs 原 (500/0.03)*0.05 = 833。
    dMdmax = (Md_ref / Constraints.tau_MR) * Ts_r;

%% 归一化权重
%  分母一律是第 0 节算出的常量基准, 不再出现瞬时可达值。
%  于是 CostWeights.* 的数字有了统一含义: "该项用满其参考量时的代价"。
%  地板不再需要(分母恒为正常量), 全部删除。
    Q1 = CostWeights.Q1/(Vym^2);
    Q2 = CostWeights.Q2/(r_ref^2);        % 原为 r_ssmax^2, 随 1/V^2 漂移
    Q3 = CostWeights.Q3/(pm^2);
    Q4 = CostWeights.Q4/(dpm^2);
    Q5 = CostWeights.Q5/(em^2);
    Q6 = CostWeights.Q6/(epm^2);
    R1 = CostWeights.R1/(dFyfmax^2);
    R2 = CostWeights.R2/(dMFxmax^2);      % dMFxmax 现已是常量, 见上
    R3 = CostWeights.R3/(dMdmax^2);
    S1 = CostWeights.S1/(Fyf_ref^2);      % 原为瞬时 Fyfmax^2
    S2 = CostWeights.S2/(MFx_ref^2);      % 原为瞬时 MFxmax^2
    S3 = CostWeights.S3/(Md_ref^2);       % 原为瞬时 |Mdmax-Mdmin|^2
    %  松弛项原来除的是 ep 的一次方, 而 Q/R 除的都是平方, 于是 W*eps^2/ep
    %  不是无量纲量, W 的数字与 Q/R 的数字不可比。改为除平方, 与 Q/R 一致:
    %  eps = ep 时该项恰为 Np*CostWeights.W。L1 罚(func_BuildQPCost 里
    %  w_L1 = lambda*Np*diag(W).*eps_scale)随之仍与二次项保持 lambda 的比例。
    W1 = CostWeights.W1/(ep_r^2);
    W2 = CostWeights.W2/(ep_ar^2);
    W3 = CostWeights.W3/(ep_ltr^2);
    W4 = CostWeights.W4/(ep_e^2);
    V1 = CostWeights.V1/1;
    V2 = CostWeights.V2/1;
    V3 = CostWeights.V3/1;

% 3. 状态跟踪Q，控制权重R，松弛因子Wshl，优先级变量Wact 
    %  终端代价: 最后一个预测节点的 Q 乘 Qf_scale。
    %  动机: 预测时域 Np*Ts = 0.6 s 比闭环自己激起的振荡周期(1.3~1.8 Hz, 即
    %  0.55~0.77 s)还短, MPC 看不到自己动作一个周期之后的后果 -> 转角在 1~3 Hz
    %  上有 52% 的能量(2026-09-05 实测)。加长 Np 能压到 19% 但摧毁跟踪精度
    %  (ey_RMS 0.27->0.49, 制动占空 38%->90%), 因为方案二把线性化冻结在整个时域。
    %  终端代价是标准替代: 用末节点的加权近似"时域之外的剩余代价", QP 规模不变。
    %  Qf_scale 的量级参考: 若闭环以时间常数 tau 衰减, 剩余代价 ~ Q*tau/Ts,
    %  tau≈0.5 s、Ts=0.05 => 量级 10。1 = 关闭。
    %  这一项同时是缺陷 B2(无终端要素)的部分补救。
    if isfield(CostWeights,'Qf_scale') && ~isempty(CostWeights.Qf_scale)
        qf = CostWeights.Qf_scale;
    else
        qf = 1;
    end
    %  codegen(R2018a): Q/R/S 原来是 cell(N,N) 逐块赋值再 cell2mat。
    %  R2018a 要求 "cell2mat 之前每个元素都必须赋过值", 循环赋值它证明不了
    %  (RefU_cell 就是这么报错的)。三个都是块对角阵, 直接写进预分配矩阵,
    %  非对角块本来就是零 —— 与 cell2mat 逐位等价。
    Q = zeros(Np*Ny, Np*Ny);
    for i = 1:Np
        r = (i-1)*Ny + (1:Ny);
        if i == Np
            Q(r,r) = qf * diag([Q1, Q2, Q3, Q4, Q5, Q6, zeros(1,Ny-6)]);
        else
            Q(r,r) = diag([Q1, Q2, Q3, Q4, Q5, Q6, zeros(1,Ny-6)]);
        end
    end
    R = zeros(Nc*Nu, Nc*Nu);
    for i = 1:Nc
        r = (i-1)*Nu + (1:Nu);
        R(r,r) = diag([R1, R2, R3]);
    end
    S = zeros(Nc*Nu, Nc*Nu);
    for i = 1:Nc
        r = (i-1)*Nu + (1:Nu);
        S(r,r) = diag([S1, S2, S3]);
    end

    Wshl =diag([W1, W1, W2, W2, W3, W3, W4, W4]);
    Wact  = diag([V1 V2 V3]);
end % end of func_CostWeightingRegulation