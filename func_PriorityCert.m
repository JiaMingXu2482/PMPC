function [w, pc, dUc, gc] = func_PriorityCert(MPCParameters, Constraints, Pred, Wts, ...
        zeta, Ut, ref_Vy, ref_r, H, f, A_cons, b_cons, WarmStart, umin, umax, Lim) %#codegen
%FUNC_PRIORITYCERT  候选裕度认证与安全权重(论文 III-E 定理 1、式 margin / weight_rule / bound_Lb)
% -------------------------------------------------------------------------
% 只用于 PrioMode = 1 (PMPC)。在解 QP **之前**调用, 不用 QP 的解:
%   1) 候选 z^ : 上一拍解后移一步(热启动向量, 末拍增量为零), 逐拍裁进本拍的
%      幅值箱与速率限 => z^ 属于 Z (论文 "projected onto Z", 同命题 1 的构造)。
%   2) 裕度 delta_l = -max_i gbar_{l,i}(z^)   (A_cons 的软约束行已按界归一化)。
%      delta_2: 横摆/后轴侧偏(相平面)裕度;  delta_1: LTR + 车身角点裕度。
%   3) F(z^) = J4(z^) + W_b*gamma^ + rho*gamma^2,  J4 含 QP 代价里省掉的常数项。
%   4) 权重规则: w2 = vartheta*F/delta2,  w1 = vartheta*(F + w2[-d2]+ + rho[-d2]+^2)/delta1,
%      截断到 [wmin, wmax];  delta_l <= 0 时取上限。
%   5) 认证: (a) d1,d2 > 0 且 w_l > F/d_l;   (b) d1 > 0 >= d2 且 w1 > (F+w2|d2|+rho d2^2)/d1;
%      (c) M_b(k-1)=0, 无 DB 候选 z^0 满足 (a), 且 W_b > L_b  =>  gamma = 0 (L3 内不为跟踪而制动)。
%   certA: 两层零松弛 = 字典序最优;  certB: 只证第一层不被牺牲, 不证 s2 = c2*。
%
% 输出:
%   w   : [w1; w2], 写进 f 的 s1, s2 两个分量
%   pc  : 14x1 [d1 d2 F w1 w2 certA certB Lb certC d1_0 d2_0 F0 gamma^ maxInViol]
%   dUc : 候选的 DeltaU (求解器超迭代上限时的兜底输入, 论文注 rem:compute)
%   gc  : 候选的 gamma_DB
% -------------------------------------------------------------------------
Nu = MPCParameters.Nu;  Nc = MPCParameters.Nc;  Np = MPCParameters.Np;
Ne = MPCParameters.Ne;  Nr = MPCParameters.Nr;  Ny = MPCParameters.Ny;
N_sh  = min(max(1, round(MPCParameters.Ncons_sh )), Np);
N_r   = min(max(1, round(MPCParameters.Ncons_r  )), Np);
N_env = min(max(1, round(MPCParameters.Ncons_env)), Np);
n_in  = 6*Nc;  n_sh = 4*N_sh;  n_r = 2*N_r;  n_env = 4*N_env;   % 行布局同 func_BuildQPConstraints
idx2  = n_in + (1:n_sh);                                         % 第二层: 横摆 + 后轴侧偏
idx1  = n_in + n_sh + (1:(n_r + n_env));                         % 第一层: LTR + 车道四角点
n1    = Nu*Nc;

du     = [Lim.dFyfmax; Lim.dMFxmax; Lim.dMdmax];
u_prev = zeta(end-2:end);

%% ---- 候选 z^ 与无 DB 候选 z^0 ----
dUws = WarmStart(1:n1);
dUws(~isfinite(dUws)) = 0;
[dUc, gc] = local_project(dUws, u_prev, umin, umax, du, Nu, Nc, false);
[dU0, ~ ] = local_project(dUws, u_prev, umin, umax, du, Nu, Nc, true);

%% ---- 裕度 (式 margin) ----
zc = [dUc; zeros(Ne,1); gc*ones(Nr,1)];
z0 = [dU0; zeros(Ne,1); zeros(Nr,1)];
gC = A_cons*zc - b_cons;
g0 = A_cons*z0 - b_cons;
d1  = -max(gC(idx1));   d2  = -max(gC(idx2));
d10 = -max(g0(idx1));   d20 = -max(g0(idx2));
maxInViol = max(gC(1:n_in));                  % 自检: 候选满足输入约束时 <= 0(舍入量级)

%% ---- F(z^) ----
%  rho: 论文式 (MPC_problem) 的 rho, s1、s2、gamma 共用, 与 func_BuildQPCost 同源
eta_g = 1e-3;
if isfield(Constraints,'gamma_reg') && ~isempty(Constraints.gamma_reg)
    eta_g = Constraints.gamma_reg;
end
rho = eta_g*(Nc*Wts.V(2,2));
Wb  = Nc*Wts.V(2,2) * double(MPCParameters.abl ~= 1);   % 消融 noWb 时 W_b = 0 (rho 不变), 与 func_BuildQPCost 一致
H4  = H(1:n1, 1:n1);   f4 = f(1:n1);
Yr  = [ref_Vy(:)'; ref_r(:)'; zeros(Ny-2, Np)];
e0  = Pred.PSI*zeta + Pred.PHI*Pred.GAMMA - Yr(:);
c0  = e0'*(Wts.Q*e0) + Ut'*(Wts.S*Ut);        % J4 的常数项(QP 代价里省掉了)
F   = max(0.5*(dUc'*(H4*dUc)) + f4'*dUc + c0, 0) + Wb*gc + rho*gc^2;
F0  = max(0.5*(dU0'*(H4*dU0)) + f4'*dU0 + c0, 0);

%% ---- 权重规则 (式 weight_rule) ----
vth  = Constraints.prio_vartheta;
wmin = Constraints.prio_wmin;    wmax = Constraints.prio_wmax;
if d2 > 0, w2 = vth*F/d2; else, w2 = wmax(2); end
w2 = min(max(w2, wmin), wmax(2));
n2 = max(-d2, 0);
if d1 > 0, w1 = vth*(F + w2*n2 + rho*n2^2)/d1; else, w1 = wmax(1); end
w1 = min(max(w1, wmin), wmax(1));
w  = [w1; w2];

%% ---- 认证 (a)(b): 截断后的权重是否仍满足定理条件 ----
certA = double(d1 > 0 && d2 > 0 && w1 > F/d1 && w2 > F/d2);
certB = double(d1 > 0 && d2 <= 0 && w1 > (F + w2*n2 + rho*n2^2)/d1);

%% ---- 认证 (c): L3 制动介入代价足够时 DB 保持为零 (式 bound_Lb) ----
certC = 0;  Lb = NaN;
ubar  = max(max(umax(2), -umin(2)), 1e-9);
if abs(u_prev(2)) <= 1e-6*ubar && d10 > 0 && d20 > 0 && w1 > F0/d10 && w2 > F0/d20
    %  对 M_b(k+i) 求导 = 对 DeltaM_b(k+i) 求导 - 对 DeltaM_b(k+i+1) 求导
    Dm = zeros(Nc, n1);
    for i = 1:Nc
        Dm(i, (i-1)*Nu + 2) = 1;
        if i < Nc
            Dm(i, i*Nu + 2) = -1;
        end
    end
    duv = kron(ones(Nc,1), du);                        % Z 落在速率限的箱内
    G   = abs(Dm*f4) + abs(Dm*H4)*duv;                 % G_i: |dJ4/dM_b(k+i)| 在 Z 上的区间上界
    c1  = max(abs(A_cons(idx1, 1:n1)*Dm'), [], 1)';    % c_{1,i}: max_rows |d gbar_1 / d M_b(k+i)|
    c2  = max(abs(A_cons(idx2, 1:n1)*Dm'), [], 1)';
    Lb  = ubar * sum(G + c1*(F0/d10) + c2*(F0/d20));
    certC = double(Wb > Lb);
end

pc = [d1; d2; F; w1; w2; certA; certB; Lb; certC; d10; d20; F0; gc; maxInViol];
end

% =========================================================================
function [dU, g] = local_project(dUws, u_prev, umin, umax, du, Nu, Nc, dbZero)
%LOCAL_PROJECT  上一拍计划的输入轨迹 u_prev + cumsum(dUws), 逐拍裁进
%  [umin, umax] 与 |Delta u| <= du;  dbZero 时 DB 的目标取 0。
%  umin/umax 已按式 box_relax 放宽, 第一拍交集非空; 此后 u 已在箱内, 各拍交集都非空。
dU = zeros(Nu*Nc, 1);
u  = u_prev;          % 候选的输入
ua = u_prev;          % 上一拍计划的输入(目标)
g  = 0;
for i = 1:Nc
    k   = (i-1)*Nu + (1:Nu);
    ua  = ua + dUws(k);
    tgt = ua;
    if dbZero, tgt(2) = 0; end
    lo  = max(umin, u - du);
    hi  = min(umax, u + du);
    un  = min(max(tgt, lo), hi);
    dU(k) = un - u;
    u   = un;
    if u(2) > 0
        g = max(g, u(2)/max(umax(2), 1e-9));
    elseif u(2) < 0
        g = max(g, u(2)/min(umin(2), -1e-9));
    end
end
g = min(max(g, 0), 1);
end
