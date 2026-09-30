function [x_opt, exitflag, delta_U_first, rho_val, epsilon_val, WarmStart, cert] = ...
        func_SolveMPCQP(H, f, A_cons, b_cons, lb, ub, WarmStart, MPCParameters, Constraints)
%  cert (可选, 2026-09-25): 论文 III-E 的在线证书, 11x1, 只记录不参与控制
%    cert(1:Ne)      松弛证书  sum(乘子 * 该松弛在约束行里的系数) / 该松弛的线性罚系数
%                    eps_j = 0 时 <= 1 (< 1 即精确罚条件成立), eps_j > 0 时 > 1
%    cert(Ne+1:end)  执行器证书 rho_gamma_j = 1'mu_j / (Nc*W_gamma_j)
%                    gamma_j = 0 时 <= 1 (< 1 即命题 4 条件成立), gamma_j > 0 时 > 1
%    PrioMode (Ne=8, Nr=3 的统一编译布局): cert(1:8) 为通用松弛证书，
%                    cert(10) 为 DB 净乘子和 / W_b，cert(9)、cert(11) 为 NaN。
%    求解失败或非 KWIK 路径时为 NaN。
%  KWIK 的活动集热启动在 func_QPKwik 内部用 persistent 维护。
%  MPCParameters.QPSolver: 0 = quadprog(默认,与原版逐位等价) / 1 = KWIK
% func_SolveMPCQP  求解 MPC 的 QP 并提取各段结果
% -------------------------------------------------------------------------
% 决策向量 x = [DeltaU; Epsilon; Gamma]
% 求解失败时返回零控制增量，并保持 WarmStart 不变。
%
% 输出:
%   x_opt        : 完整解向量 (nvars x 1; 失败时全零)
%   exitflag     : quadprog 退出标志 (1 = 成功)
%   delta_U_first: 本拍施加的控制增量 (Nu x 1)
%   rho_val      : 优先级因子 (3 x 1)
%   epsilon_val  : 松弛变量 (Ne x 1)
%   WarmStart    : 更新后的热启动向量（失败时原样返回）
% -------------------------------------------------------------------------

if isfield(MPCParameters,'QPSolver'), qsel = MPCParameters.QPSolver; else, qsel = 0; end
verbose = isfield(MPCParameters,'Verbose') && MPCParameters.Verbose;
Nu = MPCParameters.Nu;  Nc = MPCParameters.Nc;
Ne = MPCParameters.Ne;  Nr = MPCParameters.Nr;
nvars = Nu*Nc + Ne + Nr;
cert  = nan(Ne + 3, 1);     % codegen: 提前 return 的路径上也必须有定义
prio_on = false;
gdb_idx = 2;
if nargin >= 9 && isstruct(Constraints)
    prio_on = isfield(Constraints,'PrioModeRT') && (Constraints.PrioModeRT == 1);
    if isfield(Constraints,'GammaDBIndex') && ~isempty(Constraints.GammaDBIndex)
        gdb_idx = round(Constraints.GammaDBIndex);
    end
end
gdb_idx = min(max(1, gdb_idx), max(1, Nr));

% 热启动仅用于加速，不能让损坏的上一拍解终止本拍求解。
if numel(WarmStart) ~= nvars || ~all(isfinite(WarmStart(:)))
    if verbose
        %  codegen(R2018a): warning 不支持 —— MATLAB Function 块里会报
        %    "Function 'warning' is not supported for code generation."
        %  改用 fprintf(2,..) 写 stderr: 信息一样, codegen 支持。
        %  生成代码时可用 PMPC_VERBOSE=0 折掉整个打印分支。
        fprintf(2, 'func_SolveMPCQP: WarmStart size invalid or non-finite; reset to zero.\n');
    end
    WarmStart = zeros(nvars,1);
else
    WarmStart = WarmStart(:);
end

% ---- 有限性检查 ----
%  active-set 算法要求 H/f/A/b/lb/ub 全部有限，出现 Inf/NaN 会直接抛错
%  并终止整个仿真。这里改为：定位到具体是哪一项坏掉、打印一次，然后
%  按「求解失败」处理（保持上一控制量），让仿真继续跑完便于诊断。
bad = '';
if ~all(isfinite(H(:))),      bad = [bad 'H ']; end
if ~all(isfinite(f(:))),      bad = [bad 'f ']; end
if ~all(isfinite(A_cons(:))), bad = [bad 'A_cons ']; end
if ~all(isfinite(b_cons(:))), bad = [bad 'b_cons ']; end
if ~all(isfinite(lb(:))),     bad = [bad 'lb ']; end
if any(isnan(ub(:))),         bad = [bad 'ub(NaN) ']; end   % ub 允许 +Inf
if ~isempty(bad)
    if verbose
        fprintf(2, 'func_SolveMPCQP: QP matrices contain Inf/NaN: %s-> holding previous control\n', bad);
    end
    x_opt         = zeros(nvars,1);   % codegen: 尺寸必须固定(原为 [])
    exitflag      = -3;
    delta_U_first = zeros(Nu,1);
    rho_val       = [0; 0; 0];
    epsilon_val   = zeros(Ne,1);
    WarmStart     = zeros(nvars,1);
    return;
end

% active-set 还要求初始点处的目标值和约束值有限。无界软约束可能让
% 上一拍的有限解非常大，矩阵乘法随后溢出为 Inf；此时零初值仍可用。
Hx0 = H*WarmStart;
Ax0 = A_cons*WarmStart;
if ~all(isfinite(Hx0)) || ~isfinite(WarmStart'*Hx0) || ...
        ~isfinite(f'*WarmStart) || ~all(isfinite(Ax0))
    if verbose
        fprintf(2, 'func_SolveMPCQP: WarmStart produced Inf/NaN in QP; reset to zero.\n');
    end
    WarmStart = zeros(nvars,1);
end

% ---- Jacobi 预缩放 ----
%  H 的对角量级跨 12.6 个数量级 (2026-08-31 实测):
%    DeltaU-MR  3.97e-05 | DeltaU-DB 2.9e3 | DeltaU-AFS 4.2e5~3.7e6
%    Eps        1.4e6~1.5e8 | Gamma 12~9.6e5
%  => cond(H) = 1.04e13。双精度 eps=2.2e-16, 病态方向只剩约 3 位有效数字,
%  OptimalityTolerance=1e-6 形同虚设; active-set 落到哪个顶点取决于舍入,
%  于是「数学上不影响 argmin 的改动」也能让闭环结果变化两三倍 (缺陷 A4)。
%
%  令 x = S*xs, S = diag(1./sqrt(diag(H))), 则 Hs = S*H*S 对角为 1。
%  这是同一个问题的等价改写(S 正定对角), 只改求解器看到的数值尺度。
%  根治仍需重做归一化层(缺陷 B1)与那些没有物理来源的权重(C4)。
dH = max(diag(H), eps*max(diag(H)));
sc = 1 ./ sqrt(dH);
Hs = (sc*sc.') .* H;                 % = S*H*S
Hs = (Hs+Hs.')/2;
fs = sc .* f;
%  codegen(R2018a): 不能用隐式扩展(implicit expansion)。
%  A_cons 是 96x29, sc.' 是 1x29, R2016b+ 的 MATLAB 会自动广播, 但
%  R2018a 的 Coder 不支持, 直接报 "Size mismatch (size [96 x 29] ~= size [1 x 29])"。
%  bsxfun 在 R2018a codegen 里是支持的, 数值完全等价。
As = bsxfun(@times, A_cons, sc.');   % = A_cons*S
lbs = lb ./ sc;   ubs = ub ./ sc;    % x = S*xs => xs = x./sc
x0s = WarmStart ./ sc;
x0s = min(max(x0s, lbs), ubs);   % 裁进界内, 省一次 active-set 的 Phase-1

lamQ = zeros(size(A_cons,1) + 2*nvars, 1);   % codegen: 各分支都要有定义
if qsel == 1
    % ---- KWIK (mpcqpsolver): R2018a 上唯一可 codegen 的路径 ----
    [xs, st, ~, lamQ] = func_QPKwik(Hs, fs, As, b_cons, lbs, ubs, 2500, true);
    %  统一成 quadprog 的 exitflag 语义: 1=成功, 其余=失败
    %  KWIK 的 status>0 才是最优(值=迭代次数); status=0 达到上限,
    %  官方文档明说此时解**可能次优也可能不可行**, 按失败处理
    %  (与 MPC Toolbox 自己的默认行为一致: 保持上一拍控制量)。
    if st > 0, exitflag = 1; else, exitflag = st - 10; end
elseif coder.target('MATLAB')
    %  quadprog 分支只在**解释执行**时可用: R2018a 的 quadprog 不支持 codegen
    %  (R2020a 才加)。coder.target('MATLAB') 是编译期常量, 生成代码时整支剔除,
    %  于是 quadprog 不会进 C 代码; 仿真里仍可用 PMPC_QPSOLVER=0 切回去对照。
    %  optimoptions 原来在函数顶部无条件调用, R2018a 直接报
    %    "Function 'optimoptions' not supported for code generation."
    %  (R2024b 靠死代码消除侥幸过了。)搬进这个分支里, 跟 quadprog 一起被剔除。
    options = optimoptions('quadprog', ...
        'MaxIterations',        2500, ...
        'Algorithm',            'active-set', ...
        'OptimalityTolerance',  1e-6, ...
        'ConstraintTolerance',  1e-6);
    [xs, ~, exitflag] = quadprog(Hs, fs, As, b_cons, [], [], lbs, ubs, x0s, options);
else
    xs = zeros(nvars,1);  exitflag = -9;   % 生成代码中不应走到这里
end
%  codegen: x_opt 尺寸必须固定。失败时返回全零,
%  调用方已经靠 exitflag==1 把关, 不依赖 isempty(x_opt)。
x_opt = zeros(nvars,1);
if numel(xs) == nvars, x_opt = sc .* xs; end

if exitflag == 1
    WarmStart     = func_WarmStart_shiftHorizon(x_opt, MPCParameters);
    delta_U_opt   = x_opt(1:Nu*Nc);
    delta_U_first = delta_U_opt(1:Nu);

    %  codegen: rho_val 必须是固定 3x1。Nr = 3*(ContrlMode==1), 只可能是 0 或 3,
    %  所以把变尺寸切片 x_opt(end-Nr+1:end) 写成定长的后 3 个。
    rho_val = [1; 1; 1];            % 无优先级变量时默认全激活
    if Nr == 3 && ~prio_on
        rho_val = x_opt(nvars-2:nvars);
    elseif Nr == 3 && prio_on
        rho_val = [1; x_opt(Nu*Nc + Ne + gdb_idx); 1];
    elseif Nr == 1                  % 兼容旧 PMPC 结构
        rho_val = [1; x_opt(nvars); 1];
    end

    epsilon_val = x_opt(Nu*Nc+1 : Nu*Nc+Ne);
else
    delta_U_first = zeros(Nu,1);
    rho_val       = [0; 0; 0];
    epsilon_val   = zeros(Ne,1);
end

%% ---- 在线证书 (论文 III-E 式 certificate / certificate_act), 只记录 ----
%  乘子与预缩放无关: 缩放的是变量(列), 约束行没动, 所以 lamQ 就是原问题 A_cons 的乘子。
%  对变量 v 的平稳性条件 (A*x <= b 形式):  (H*x)_v + f_v + A(:,v)'*lam - nu_lb + nu_ub = 0
%  v 为松弛或 gamma 时, 它在约束行里的系数全是非正的(-1 或 -(umax-uc) 等), 于是
%     s_v = sum_i lam_i * (-A_iv) = f_v + (H*x)_v - nu_lb + nu_ub
%  取值为 0 时 (H*x)_v = 0(这两类变量的 Hessian 块是对角的), s_v = f_v - nu_lb <= f_v,
%  取正值时 s_v = f_v + (H*x)_v > f_v。故 cert_v = s_v / f_v 以 1 为分界。
%  对 gamma, 系数 -(umax-uc) 正好是论文按 r_j^+ 归一化的行, s_v 即 1'mu_j。
if qsel == 1 && exitflag == 1
    lamA = lamQ(1:size(A_cons,1));
    for j = 1:Ne
        v = Nu*Nc + j;
        if f(v) > 0
            cert(j) = sum(lamA .* (-A_cons(:,v))) / f(v);
        end
    end
    %  执行器证书: gamma_j = 0 时执行器被钉在中性点, 同一拍的上/下两条优先级行
    %  同时起作用, 乘子不唯一(退化), 求解器给出的拆分会让比值恰好 = 1(实测 DB、
    %  CDC 全是 1.000)。唯一确定的是两行对 u_j(i) 的**合力** d = lam_h - lam_l,
    %  取合力相同、和最小的那组乘子: 只留 d 所指一侧, mu = |d|*r。它对"执行器
    %  固定在中性点"的问题 P_A,j 同样是合法乘子, 故命题 4 的判据照用。
    %  行布局: A_cons 前 6*Nc 行是优先级约束, 先 3*Nc 行上界再 3*Nc 行下界,
    %  第 i 拍第 j 个执行器在 3*(i-1)+j。只在 sigma_budget / fx_couple 关闭时成立
    %  (两者会往 gamma 列或输入块里加行, 命题 4 也不覆盖), 否则置 NaN。
    if Nr == 3 && ~prio_on && MPCParameters.sigma_budget == 0 && MPCParameters.fx_couple == 0
        for j = 1:3
            v = Nu*Nc + Ne + j;
            if f(v) > 0
                s = 0;
                for i = 1:Nc
                    rh = 3*(i-1) + j;          % 上界行
                    rl = 3*Nc + rh;            % 下界行
                    d  = lamA(rh) - lamA(rl);
                    s  = s + max(d,0)*(-A_cons(rh,v)) + max(-d,0)*(-A_cons(rl,v));
                end
                cert(Ne + j) = s / f(v);
            end
        end
    elseif prio_on && Nr > 0
        %  PrioMode 只有 gamma_DB(列 nvars), 行布局不变: DB 在每拍第 2 行。
        %  cert(Ne+2) = 净乘子和 / W_b, 与定理 1(c) 的先验界 L_b / W_b 对照。
        v = Nu*Nc + Ne + gdb_idx;
        if f(v) > 0
            s = 0;
            for i = 1:Nc
                rh = 3*(i-1) + 2;
                rl = 3*Nc + rh;
                d  = lamA(rh) - lamA(rl);
                s  = s + max(d,0)*(-A_cons(rh,v)) + max(-d,0)*(-A_cons(rl,v));
            end
            cert(Ne + 2) = s / f(v);
        end
    end
end

end
