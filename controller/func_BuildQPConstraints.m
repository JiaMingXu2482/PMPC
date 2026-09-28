function [A_cons, b_cons, lb, ub, umax, umin] = func_BuildQPConstraints( ...
        MPCParameters, Constraints, Envelope, Pred, AI, Ut, zeta, Lim)
% func_BuildQPConstraints  组装 MPC 的不等式约束与变量上下界
% -------------------------------------------------------------------------
% 决策向量 x = [DeltaU; Epsilon; Gamma]   (Nr=0 时无 Gamma 段)
%   A_cons * x <= b_cons ,  lb <= x <= ub
%
% 约束顺序: (1) 控制量幅值  (2) 稳定性 sh  (3) 侧倾 LTR  (4) 道路 env
%
% 输入:
%   Envelope : func_Envelope 输出 (Hsh/Gsh, Henv/Genv, Hr/Or/Gr)
%   Pred     : struct，含 PSI, THETA, PHI, GAMMA
%   Lim      : struct，含 Fyfmax, MFxmax, Mdmax, Mdmin, dFyfmax, dMFxmax, dMdmax
% -------------------------------------------------------------------------

Nu = MPCParameters.Nu;  Nc = MPCParameters.Nc;
Np = MPCParameters.Np;  Ne = MPCParameters.Ne;  Nr = MPCParameters.Nr;
Ny = MPCParameters.Ny;
prio_on = isfield(Constraints,'PrioModeRT') && (Constraints.PrioModeRT == 1);
ctrl_mode = 2;
if isfield(Constraints,'ControllerMode') && ~isempty(Constraints.ControllerMode)
    ctrl_mode = Constraints.ControllerMode;
end
gdb_idx = 2;
if isfield(Constraints,'GammaDBIndex') && ~isempty(Constraints.GammaDBIndex)
    gdb_idx = round(Constraints.GammaDBIndex);
end
gdb_idx = min(max(1, gdb_idx), max(1, Nr));

% 约束时域：三类状态约束各自的施加步数（字段缺省时 = Np，即全时域）
%  codegen(R2018a): 直接读, 不走 isfield/isempty 链。setup_pmpc 一定会设这三个,
%  而 isfield 链在老 Coder 上会削弱常量传播, 使 N_* 退化成运行时值 ->
%  A_cons 行数变可变 -> 一路传到 qpkwik.p 报
%    "Dimension 1 is fixed on the left-hand side but varies on the right"
%  当前配置: N_sh=6, N_r=6, N_env=Np=16 (2026-09-26 前 Np=12)。
N_sh  = min(max(1, round(MPCParameters.Ncons_sh )), Np);
N_r   = min(max(1, round(MPCParameters.Ncons_r  )), Np);
N_env = min(max(1, round(MPCParameters.Ncons_env)), Np);

PSI = Pred.PSI; THETA = Pred.THETA; PHI = Pred.PHI; GAMMA = Pred.GAMMA;
xi0 = PSI*zeta + PHI*GAMMA;          % 零输入自由响应，三处状态约束共用

%% ---- 1) 控制量幅值上下界 ----
% 由速度依赖的 Mdmax/Mdmin 生成不对称上下界，并向内收缩留裕度
Md_up    = max(Lim.Mdmax, Lim.Mdmin);
Md_low   = min(Lim.Mdmax, Lim.Mdmin);
Md_up_s  = min( (Md_up >=0)*0.9*Md_up  + (Md_up <0)*1.1*Md_up , Md_up  );
Md_low_s = max( (Md_low<=0)*0.9*Md_low + (Md_low>0)*1.1*Md_low, Md_low );

% AFS: 若给了非对称转角界则用之（控制量为前轮转角时必须非对称）
if isfield(Lim,'dlt_ub') && ~isempty(Lim.dlt_ub)
    afs_ub = Lim.dlt_ub;  afs_lb = Lim.dlt_lb;
else
    afs_ub = 0.95*Lim.Fyfmax;  afs_lb = -0.95*Lim.Fyfmax;
end
umax = [ afs_ub;  0.95*Lim.MFxmax;  Md_up_s  ];
umin = [ afs_lb; -0.95*Lim.MFxmax;  Md_low_s ];

% ---- 保证箱体始终包含当前控制量 ----
%  Fyfmax/MFxmax/Mdmax 都是随垂向载荷与摩擦圆实时变化的量，箱体可以
%  收缩得比 |Delta u| <= dumax 追得上的速度更快。此时 u(k-1) 落在新箱
%  体之外，而速率限制一步拉不回来 => 硬性不可行（与松弛变量无关，
%  无界 eps 也救不了）。把箱体撑开到至少包含 u(k-1)，即可保证
%  「Delta u = 0, gamma = 1, eps 充分大」始终是一个可行点，从而
%  与无界松弛一起给出 QP 恒可行。
%  但不能简单地 umax = max(umax,u_prev)：那样一旦 u 跑到界外，箱体就
%  跟着它走，没有任何力量把它拉回来（实测 51% 的时间 Md 在界外，且
%  越界样本中 100% 是上一拍也在界外 —— 自持陷阱）。
%  正确做法：允许在界外，但每拍必须朝界内至少移动一个速率限幅 dumax，
%  这样既保证可行域非空，又保证有限步内回到物理界内。
u_prev  = zeta(end-2:end);      % zeta = [x(Nx); u(k-1)], Nx = 7 起不再是 7:9
du_v    = [Lim.dFyfmax; Lim.dMFxmax; Lim.dMdmax];
umax = max(umax, u_prev - du_v);
umin = min(umin, u_prev + du_v);

Umin = kron(ones(Nc,1), umin);
Umax = kron(ones(Nc,1), umax);

%% ---- 2) 状态软约束矩阵 ----
Hsh = Envelope.Hsh;  Gsh  = Envelope.Gsh;
Henv= Envelope.Henv; Genv = Envelope.Genv;
Hr  = Envelope.Hr;   Gr   = Envelope.Gr;   Or = Envelope.Or;

% Sel(n): 从 Np 个输出块中取前 n 块
sel   = @(n) [eye(n*Ny), zeros(n*Ny, (Np-n)*Ny)];
A_sh  = kron(eye(N_sh ),Hsh)  * sel(N_sh );   b_sh  = kron(ones(N_sh ,1),Gsh);
A_env = kron(eye(N_env),Henv) * sel(N_env);   b_env = kron(ones(N_env,1),Genv);
b_env = b_env + kron(Lim.kap(1:N_env), Envelope.gkap);   % 四角点弯道修正 -kappa*a^2/2, 见 func_Envelope
A_r   = kron(eye(N_r  ),Hr)   * sel(N_r  );   b_r   = kron(ones(N_r  ,1),Gr);

% E_r: 把 Nc 段的控制量贡献映射到前 N_r 步
if N_r <= Nc
    E_r = [eye(2*N_r), zeros(2*N_r, 2*(Nc-N_r))];
else
    % 超出 Nc 的步保持最后一步的输入（而非置零）
    E_r = [ eye(2*Nc);
            repmat([zeros(2, 2*(Nc-1)), eye(2)], N_r-Nc, 1) ];
end
O_r = kron(eye(Nc),Or);

if prio_on
    % ---- 论文 III-D (2026-09-26): 行按界归一化, 每层一个违反量 ----
    %  x = [DeltaU; s1; s2; gamma_DB]。第一层(LTR + 车道四角点) -> s1,
    %  第二层(横摆 + 后轴侧偏) -> s2。归一化后行值 0.1 即超出界 10%,
    %  s_l = max(0, 该层最大行值), 正是论文式 (measures) 的 s_l。
    dsh  = kron(ones(N_sh ,1), 1./Envelope.sc_sh);
    denv = kron(ones(N_env,1), 1./Envelope.sc_env);
    dr   = kron(ones(N_r  ,1), 1./Envelope.sc_r);
    E_sh = zeros(size(A_sh,1), Ne);    E_sh(:,2) = -1;
    E_env = zeros(size(A_env,1), Ne);  E_env(:,1) = -1;
    E_r_eps = zeros(size(A_r,1), Ne);  E_r_eps(:,1) = -1;
    A_cons_sh  = [bsxfun(@times, A_sh*THETA, dsh), E_sh, zeros(size(A_sh,1),Nr)];
    b_cons_sh  = dsh .* (b_sh - A_sh*xi0);
    A_cons_env = [bsxfun(@times, A_env*THETA, denv), E_env, zeros(size(A_env,1),Nr)];
    b_cons_env = denv .* (b_env - A_env*xi0);
    A_cons_r   = [bsxfun(@times, A_r*THETA + E_r*O_r*AI, dr), E_r_eps, zeros(size(A_r,1),Nr)];
    b_cons_r   = dr .* (b_r - A_r*xi0 - E_r*O_r*Ut);
else
% (1) 稳定性约束 (yaw rate / alpha_r)
A_cons_sh = [A_sh*THETA, -1*kron(ones(N_sh,1),[eye(4),zeros(4,4)]), zeros(length(Gsh)*N_sh,Nr)];
b_cons_sh = b_sh - A_sh*xi0;

% (2) 道路边界约束
A_cons_env= [A_env*THETA, -1*kron(ones(N_env,1),Envelope.Eenv), zeros(length(Genv)*N_env,Nr)];   % 四角点 4 行, 见 func_Envelope
b_cons_env= b_env - A_env*xi0;

% (3) 侧倾 (LTR) 约束 —— 混合状态-输入约束
A_cons_r  = [A_r*THETA + E_r*O_r*AI, -1*kron(ones(N_r,1),[zeros(2,4),eye(2),zeros(2,2)]), zeros(length(Gr)*N_r,Nr)];
b_cons_r  = b_r - A_r*xi0 - E_r*O_r*Ut;
end

%% ---- 3) 控制量幅值约束 (区分 PrioMode / Nr>0 / Nr=0) ----
if prio_on
    % ---- 论文式 (const_box): AFS、CDC 普通箱约束; 只有 DB 由 gamma 缩放 ----
    %     gamma*umin_b <= M_b(k+i) <= gamma*umax_b,  0 <= gamma <= 1
    %  执行器不排序: AFS/CDC 只受物理界; DB 的幅值由 gamma 缩放, gamma 在 L3 里按 W_b 定价
    %  (制动介入代价), DB 对 L1/L2 始终可用。
    %  行布局与原来相同: 前 3*Nc 行上界、后 3*Nc 行下界, 第 i 拍第 k 个输入在 3*(i-1)+k。
    A_high = zeros(3*Nc, Nu*Nc + Ne + Nr);
    A_low  = zeros(3*Nc, Nu*Nc + Ne + Nr);
    b_high = zeros(3*Nc, 1);
    b_low  = zeros(3*Nc, 1);
    for i = 1:Nc
        AI_i = AI((i-1)*Nu+1:i*Nu, :);
        Ut_i = Ut((i-1)*Nu+1:i*Nu);
        for k = 1:3
            row = 3*(i-1) + k;
            A_high(row, 1:Nu*Nc) =  AI_i(k, :);
            A_low(row,  1:Nu*Nc) = -AI_i(k, :);
            if k == 2                                  % DB: gamma 缩放
                A_high(row, Nu*Nc + Ne + gdb_idx) = -umax(2);
                b_high(row)                 = -Ut_i(2);
                A_low(row,  Nu*Nc + Ne + gdb_idx) =  umin(2);
                b_low(row)                  =  Ut_i(2);
            else                                       % AFS / CDC: 普通箱
                b_high(row) = umax(k) - Ut_i(k);
                b_low(row)  = Ut_i(k) - umin(k);
            end
        end
    end
    A_input = [A_high; A_low];
    b_input = [b_high; b_low];
elseif Nr > 0
    % 优先级策略, 以各执行器的"零作用点" uc 为中心:
    %     uc + gamma*(umin-uc)  <=  u(k)  <=  uc + gamma*(umax-uc)
    %   gamma=0 => u 退到 uc (该执行器不作用); gamma=1 => 用满 [umin,umax]。
    %
    % AFS / 差动制动是主动件, 不作用即 u=0, 故 uc=0, 退化为原来的
    % umin*gamma <= u <= umax*gamma。
    %
    % MR 是半主动件, 退不出——线圈断电它仍在出力。它的"不作用"是保持
    % 标称电流 I_nom (与 baseline 同值) 的被动阻尼, 对应力矩 Mdnom。
    % 原先用 rho_min=[0;0;1] 把 gamma_CDC 钉在 1 来表达"MR 常开",
    % 副作用是 Nc*V3*gamma^2 退化为常数, V3 从 10 扫到 8e4 毫无响应
    % (见 README_归一化审查 第四节)。改成以 Mdnom 为中心后:
    %   gamma_CDC = 0 -> 保持被动阻尼, 物理上正确且始终可行;
    %   gamma_CDC 度量的是"调制幅度", V3 因而真正为 MR 的调制定价。
    % baseline (固定 I_nom) 就是 gamma_CDC == 0 的特例, 对照关系变干净。
    if isfield(Lim,'Mdnom') && ~isempty(Lim.Mdnom)
        uc = [0; 0; Lim.Mdnom];
    else
        uc = zeros(3,1);
    end
    uc(3) = min(max(uc(3), min(umin(3),umax(3))), max(umin(3),umax(3)));  % 必须在箱内

    A_high = zeros(3*Nc, Nu*Nc + Ne + Nr);
    A_low  = zeros(3*Nc, Nu*Nc + Ne + Nr);
    b_high = zeros(3*Nc, 1);
    b_low  = zeros(3*Nc, 1);

    for i = 1:Nc
        AI_i = AI((i-1)*Nu+1:i*Nu, :);
        Ut_i = Ut((i-1)*Nu+1:i*Nu);
        for k = 1:3                                  % 3 个控制量
            row = 3*(i-1) + k;
            % High:  DeltaU - gamma*(umax-uc) <= uc - Ut
            A_high(row, 1:Nu*Nc)        =  AI_i(k, :);
            A_high(row, Nu*Nc + Ne + k) = -(umax(k) - uc(k));
            b_high(row)                 =  uc(k) - Ut_i(k);
            % Low:  -DeltaU + gamma*(umin-uc) <= Ut - uc
            A_low(row, 1:Nu*Nc)         = -AI_i(k, :);
            A_low(row, Nu*Nc + Ne + k)  =  umin(k) - uc(k);
            b_low(row)                  =  Ut_i(k) - uc(k);
        end
    end
    A_input = [A_high; A_low];
    b_input = [b_high; b_low];

    %% ---- 3c) 前轴摩擦耦合 (方案 B) ----
    %  |u_DB| + kappa*|u_AFS - u_op| <= 0.95*MFxmax
    %  即 AFS 每多用一点前轴侧向力, 前轮能提供的纵向力就少一点, 差动制动的
    %  可用横摆力矩随之减少。这是 MFxmax 用实测 Fy 计算时漏掉的耦合。
    %  用绝对值形式(菱形)是保守内近似, 不会高估可用能力; 展开为 4 行/步。
    %  u_AFS = u_op 时退化为原来的 |u_DB| <= 0.95*MFxmax, 故 DeltaU=0 仍可行。
    %  ⚠️ 必须测 MPCParameters.fx_couple, **不能**测 Lim.fx_couple。
    %  Lim 是 pmpc_step 里用 struct(...) 运行时拼出来的, 字段大多是运行时值;
    %  常量一旦存进运行时结构体, R2018a 就当运行时值处理, 这个 if 折不掉,
    %  于是下面的 [A_input; A_fx] 让行数变可变 -> A_cons/Am/bm/iA0 全变尺寸
    %  -> qpkwik.p 报 [59 x 1] ~= [:? x 1]。(R2024b 能看穿结构体, R2018a 不能。)
    %  MPCParameters 属于 Pm(块里是 coder.Constant), 直接测才折得掉。
    if MPCParameters.fx_couple ~= 0
        kap  = Lim.kappa_fx;   uop = Lim.u_op;
        A_fx = zeros(4*Nc, Nu*Nc + Ne + Nr);
        b_fx = zeros(4*Nc, 1);
        for i = 1:Nc
            AI_i = AI((i-1)*Nu+1:i*Nu, :);
            Ut_i = Ut((i-1)*Nu+1:i*Nu);
            aA = AI_i(1,:);   aD = AI_i(2,:);      % 对 DeltaU 的累加行
            for sD = [1 -1]
                for sA = [1 -1]
                    r = 4*(i-1) + (sD<0)*2 + (sA<0) + 1;
                    A_fx(r, 1:Nu*Nc) = sD*aD + kap*sA*aA;
                    b_fx(r) = 0.95*Lim.MFxmax - sD*Ut_i(2) - kap*sA*(Ut_i(1) - uop);
                end
            end
        end
        A_input = [A_input; A_fx];
        b_input = [b_input; b_fx];
    end

    %% ---- 3b) sigma 耦合预算 (方案 A) ----
    %  sigma_AFS + sigma_DB <= B。目的: 让 gamma 不再各自独立地被 |u|/umax 顶上去,
    %  而是必须互相让位 —— 这才是"优先级分配"。
    %  B 的取值依据是摩擦椭圆 a^2+d^2<=1 的切线线性化 B=sqrt(2), 但注意:
    %  鱼钩下 sigma_AFS 缩放的是作动器幅值限(AFS_max=5.73deg, Wang 的 delta_fm)
    %  而非摩擦限, 所以这个物理依据在该工况并不严格成立 —— B 带有工程参数性质,
    %  论文中需如实说明。详见 README_架构缺陷与救治方案 2026-09-01。
    if MPCParameters.sigma_budget > 0        % 同上: 必须测 MPCParameters 不能测 Lim
        Bb = MPCParameters.sigma_budget;
        %  可行性: 上一拍的 sigma 可能已超预算(尤其刚打开约束时)。沿用本文件
        %  第 55~63 行同样的做法 —— 允许在界外, 但每拍必须向界内移动 dB,
        %  从而 "DeltaU=0" 始终可行, 且有限步内回到预算内。
        s_prev = 2;
        if isfield(Lim,'gamma_prev') && ~isempty(Lim.gamma_prev)
            s_prev = sum(Lim.gamma_prev(1:2));
        end
        dB   = 0.1;
        Beff = max(Bb, s_prev - dB);
        A_input = [A_input; zeros(1, Nu*Nc + Ne), 1, 1, 0];
        b_input = [b_input; Beff];
    end
else
    A_input = [ AI, zeros(Nu*Nc, Ne);
               -AI, zeros(Nu*Nc, Ne) ];
    b_input = [ Umax - Ut;
                Ut - Umin ];
end

%% ---- 4) 合并 ----
%  codegen: 预分配固定尺寸再按固定下标填, 不用纵向拼接。
%  当前配置各块行数(全部由 Pm 里的常量决定, 编译期可定; Np = 16, 2026-09-26 前为 12):
%     A_input   6*Nc              = 36
%     A_cons_sh numel(Gsh)*N_sh   = 4*6  = 24
%     A_cons_r  numel(Gr)*N_r     = 2*6  = 12
%     A_cons_env numel(Genv)*N_env= 4*16 = 64   (四角点, 2026-09-25 前为 2*12)
%     -------------------------------------------- 合计 136
%     变量数: MPC/ZENG 29 (Nu*Nc + 8 松弛), PMPC 21 (Nu*Nc + s1 s2 + gamma_DB)
nvars_ = Nu*Nc + Ne + Nr;
n_in_  = size(A_input,1);
n_sh_  = size(A_cons_sh,1);
n_r_   = size(A_cons_r,1);
n_env_ = size(A_cons_env,1);
A_cons = zeros(n_in_ + n_sh_ + n_r_ + n_env_, nvars_);
b_cons = zeros(n_in_ + n_sh_ + n_r_ + n_env_, 1);
k_ = 0;
A_cons(k_+1:k_+n_in_,  :) = A_input;    b_cons(k_+1:k_+n_in_)  = b_input;    k_ = k_ + n_in_;
A_cons(k_+1:k_+n_sh_,  :) = A_cons_sh;  b_cons(k_+1:k_+n_sh_)  = b_cons_sh;  k_ = k_ + n_sh_;
A_cons(k_+1:k_+n_r_,   :) = A_cons_r;   b_cons(k_+1:k_+n_r_)   = b_cons_r;   k_ = k_ + n_r_;
A_cons(k_+1:k_+n_env_, :) = A_cons_env; b_cons(k_+1:k_+n_env_) = b_cons_env;

%% ---- 5) 变量上下界 ----
dumax = [Lim.dFyfmax; Lim.dMFxmax; Lim.dMdmax];
dUmin = kron(ones(Nc,1), -dumax);
dUmax = kron(ones(Nc,1),  dumax);

eps_min = zeros(Ne,1);
% ---- 无界松弛 ----
%  原来 eps <= eps_max 是硬上界：一旦预测漂移所需的松弛量超过 eps_max，
%  QP 直接不可行（Np 加大时尤其明显）。改为无界后配合 func_BuildQPCost 的 L1 精确罚：
%    (i)  可行性：对任意 zeta，取 eps 足够大总能满足软约束 => QP 恒可行；
%    (ii) 精确性：lambda_L1 足够大时，软约束解 = 硬约束解（只要后者可行）。
%  2026-09-05: 改为**有限但充裕**的上界, 以便论文能写「约束违反有上界」。
%  取各松弛自身的参考尺度 epsilon_* 的 eps_ub_scale 倍。inf = 退回无界。
%  只要该界从不咬合, 上面 (i)(ii) 两条性质就原样保留, 而我们多了一个
%  可陈述的、被实测验证过的违反上限。实测是否咬合见 README 任务 #7。
if isfield(Lim,'eps_ub') && ~isempty(Lim.eps_ub)
    eps_ub = Lim.eps_ub(:);
else
    eps_ub = inf(Ne,1);
end

if prio_on
    eps_lb = eps_min;
    eps_ub2 = eps_ub;
    if Ne > 2
        eps_ub2(3:Ne) = 0;           % PMPC 新结构未使用的松弛固定为 0
    end
    gam_lb = ones(Nr,1);
    gam_ub = ones(Nr,1);
    gam_lb(gdb_idx) = 0;             % 仅 gamma_DB 可调
    lb = [dUmin; eps_lb; gam_lb];
    ub = [dUmax; eps_ub2; gam_ub];
elseif Nr > 0
    if ctrl_mode == 1
        rho_min = zeros(Nr,1);       % PMPC(旧结构)保留 3 gamma
    else
        rho_min = ones(Nr,1);        % baseline/ZENG: 屏蔽 gamma 变量
    end
    rho_max = ones(Nr,1);
    lb = [dUmin; eps_min; rho_min];
    ub = [dUmax; eps_ub;  rho_max];
else
    lb = [dUmin; eps_min];
    ub = [dUmax; eps_ub];
end

end
