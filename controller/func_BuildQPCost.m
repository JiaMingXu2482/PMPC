function [H, f] = func_BuildQPCost(MPCParameters, Constraints, Pred, Wts, ...
                                   zeta, AI, Ut, ref_Vy, ref_r, lambda_L1, ...
                                   gamma_prev, W_dgamma)
% func_BuildQPCost  组装 MPC 的二次规划代价 (H, f)
% -------------------------------------------------------------------------
% 决策向量 x = [DeltaU; Epsilon; Gamma]   (Nr=0 时无 Gamma 段)
%   J = 0.5*x'*H*x + f'*x
%
% 输入:
%   MPCParameters : 含 Nu,Nc,Np,Ne,Nr
%   Constraints   : 含 epsilon_r / epsilon_alpha / epsilon_LTR / epsilon_e
%   Pred          : struct，含 PSI, THETA, PHI, GAMMA (func_SystemFurture 输出)
%   Wts           : struct，含 Q,R,S (跟踪/增量/控制量) 与 W (松弛), V (优先级)
%   zeta          : 增广初始状态 [xi; u_prev]
%   AI, Ut        : 控制量累加矩阵与上一步控制量的堆叠
%   ref_Vy,ref_r  : 参考横摆/侧向速度序列 (长度 >= Np)
%   lambda_L1     : L1 精确罚强度 (0 = 退回纯二次罚)
%
% 输出:
%   H, f
% -------------------------------------------------------------------------

Nu = MPCParameters.Nu;  Nc = MPCParameters.Nc;
Np = MPCParameters.Np;  Ne = MPCParameters.Ne;  Nr = MPCParameters.Nr;
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

PSI = Pred.PSI; THETA = Pred.THETA; PHI = Pred.PHI; GAMMA = Pred.GAMMA;
Q = Wts.Q; R = Wts.R; S = Wts.S; W = Wts.W; V = Wts.V;

%% --- H 矩阵 (Cell 结构方便处理 Nr=0 的情况) ---
%  codegen(R2018a): 原来用 cell(3,3)+cell2mat 拼(为了方便处理 Nr=0)。
%  R2018a 对 cell 有 "cell2mat 之前每个元素都必须赋过值" 的检查, 换成
%  按块写进预分配矩阵 —— 非对角块全是零, 与 cell2mat 逐位等价。
n1 = Nc*Nu;  n2 = Ne;  n3 = Nr;
H = zeros(n1+n2+n3, n1+n2+n3);
H(1:n1, 1:n1)           = 2*(THETA'*Q*THETA + R + AI'*S*AI);
if prio_on
    %  论文式 (MPC_problem): rho*(s1^2 + s2^2 + gamma^2), rho = eta*W_b(s、gamma 共用一个 rho)。
    %  线性权重 w1, w2 由候选裕度在线给出(func_PriorityCert), 在 pmpc_step 里写进 f。
    %  rho 只影响不可达时的折中, 不影响精确性(引理 1 对任意 rho >= 0 成立)。
    eta_s = 1e-3;
    if isfield(Constraints,'gamma_reg') && ~isempty(Constraints.gamma_reg)
        eta_s = Constraints.gamma_reg;
    end
    Hs_eps = zeros(n2,1);
    Hs_eps(1:min(2,n2)) = 2*eta_s*Nc*V(2,2);
    H(n1+(1:n2), n1+(1:n2)) = diag(Hs_eps);
else
    H(n1+(1:n2), n1+(1:n2)) = 2*Np*W;
end
% ---- Delta-gamma 惩罚：给优先级因子加惯性 ----
%  gamma 只出现在 (38c) 与代价里，且代价对 gamma 单调递增，故最优解恒有
%  gamma* = max_k |u(k)|/u_max —— 它是被 u 唯一决定的从属变量，每拍从零
%  重新决策，在「用/不用某执行器」的切换面附近会逐拍翻转（实测 MFx 有
%  74% 时间≈0、12% 顶上限，抖动全部集中在 gamma_DB 过渡区）。
%  加入 ||gamma - gamma_prev||^2_{W_dgamma} 后 gamma 具有记忆，
%  切换带有迟滞；同时 gamma 不再可被 |u|/u_max 消去，成为真正的决策状态。
if nargin < 12 || isempty(W_dgamma), W_dgamma = zeros(Nr); end
if nargin < 11 || isempty(gamma_prev), gamma_prev = zeros(Nr,1); end
%  --- 优先级变量 gamma 的罚: **线性**, 与原文一致 ---
%  R. Hajiloo et al., "A prioritisation model predictive control for
%  multi-actuated vehicle stability with experimental verification,"
%  Vehicle System Dynamics, 2023. 代价 Eq.(27):
%      J = ... + W_r s_r + W_beta s_beta + **W_rho * rho(k)**
%  即优先级变量是**线性**罚。原来写成二次罚 Nc*V*gamma^2 是实现偏差:
%    线性罚梯度恒为 W_rho -> "用就用足, 不用就归零"的干净开关;
%    二次罚梯度 2*V*gamma -> gamma 越大边际代价越高, 总停在折中小值,
%    实测使差动制动只用到 baseline 的 12%(RMS), 跟踪反而变差。
%  W_dgamma (Delta-gamma 迟滞) 是本项目为抑制 gamma 逐拍翻转加的, 仍为二次, 保留。
%  纯线性罚会使 H 的 gamma 块奇异(在 gamma 方向上退化为 LP), active-set
%  quadprog 会因此返回伪不可行(实测 28/799 拍 exitflag=-2)。故加一个
%  **很小的**二次正则项 eta*Nc*V 保持正定; eta=1e-3 时在 gamma=1 处
%  二次项仅为线性项的 0.1%, 不改变"用就用足"的开关性质。
if prio_on
    %  只有 gamma_DB: W_b*gamma + rho_g*gamma^2, W_b = Nc*V2, rho_g = eta*Nc*V2
    %  (与原来三 gamma 结构里 DB 那一项的数值相同, 便于对比)。
    eta_g = 1e-3;
    if isfield(Constraints,'gamma_reg') && ~isempty(Constraints.gamma_reg)
        eta_g = Constraints.gamma_reg;
    end
    H(n1+n2+gdb_idx, n1+n2+gdb_idx) = 2*eta_g*Nc*V(2,2);
elseif Nr > 0 && ctrl_mode == 1
    eta_g = 1e-3;
    if isfield(Constraints,'gamma_reg') && ~isempty(Constraints.gamma_reg)
        eta_g = Constraints.gamma_reg;
    end
    H(n1+n2+(1:n3), n1+n2+(1:n3)) = 2*(W_dgamma + eta_g*Nc*V);
end
H = (H+H')/2 + 1e-8*eye(size(H));      % 正定化

%% --- f 向量 ---
% 参考轨迹向量 Yr：每步 6 个输出，只有前两项 (Vy, r) 有参考
Yr_blk = [ref_Vy(:)'; ref_r(:)'; zeros(MPCParameters.Ny-2, Np)];
if MPCParameters.Ny == 8
    Yr_blk(8,:) = Wts.Vx_ref(:)';
end
Yr     = Yr_blk(:);                                               % (Np*Ny) x 1

a    = PSI*zeta + PHI*GAMMA;
f_DU = 2*(THETA'*(Q*(a - Yr)) + AI'*(S*Ut));

% ---- 松弛变量的归一化尺度（W 已按它归一）----
%  eps_scale 只是尺度，不作为 eps 的硬上界（上界见 func_BuildQPConstraints）
eps_scale = [Constraints.epsilon_r*ones(2,1);   Constraints.epsilon_alpha*ones(2,1);
             Constraints.epsilon_LTR*ones(2,1); Constraints.epsilon_e*ones(2,1)];

% ---- L1 精确罚 ----
%  因 eps>=0，||eps||_1 = 1'*eps，故 L1 罚就是 f 中的一个线性项，无需辅助变量。
%  二次罚梯度 2*W*eps 在 eps->0 时趋于 0，故最优 eps* 恒 >0（无论 W 多大）；
%  L1 罚梯度恒为 w，只要 w > |lambda*|（该约束的最优乘子），eps* 精确为 0。
%  默认尺度：w_L1(i) = lambda_L1 * Np * W(i,i) * eps_scale(i)
%    => 在 eps = eps_scale 处 L1 项与二次项等量；eps 更小时 L1 主导。
if prio_on
    w_L1    = zeros(Ne,1);             % 占位: w1, w2 由 func_PriorityCert 给出后写入
    f_gamma = zeros(Nr,1);
    f_gamma(gdb_idx) = Nc*V(2,2) * double(MPCParameters.abl ~= 1);   % W_b: 仅 DB 通道
elseif Nr > 0 && ctrl_mode == 1
    w_L1    = lambda_L1 * Np * diag(W) .* eps_scale;
    f_gamma = Nc*diag(V) - 2 * W_dgamma * gamma_prev(:);   % 线性优先级罚 + 迟滞项
else
    w_L1    = lambda_L1 * Np * diag(W) .* eps_scale;
    f_gamma = zeros(Nr,1);
end

f = [ f_DU; w_L1; f_gamma ];

end
