function [SW, Icmd, Tb, stateNext, Xlocal, Ylocal, logVec] = ...
    pmpc_base_step(meas, Reftraj, params, state, delta_d, prediction)
%PMPC_BASE_STEP 基线 MPC 的 100 Hz 控制入口（6 状态 / 2 控制量）。
%#codegen
%
% 状态  x = [Vy; r; phi; p; ey; epsi]  （含侧倾与侧倾角速度，不含 Md）
% 控制  u = [Delta delta; Mz]           （Md 不参与）
% 减振器输出恒定电流 params.M.cdc_fixed_I，不是决策量。
%
% 与 pmpc_core/pmpc_step 的区别：
%   - 没有 Flo/Fhi 输入（不需要力界，因为不控减振器）
%   - 控制量 2 维不是 3 维，没有 gamma_DB 缩放、没有 Md 硬约束、没有四角力分配 QP
%   - 减振器时滞 tau_d 不建模
%   - 松弛权重是固定值，不做自适应
%   - logVec 为 25x1（保持与协调控制器同格式，空位填 NaN）
%
% logVec =
%  [ey,epsi,phi,p,LTR,Md_actual,Md_cmd,Md_min,Md_max,
%   Fd_L1..R2, Fdes_L1..R2, Mz_cmd, SW, Vx, r, dMz, dMz_max, QP_OK, candidate_used]
% 基线不控 Md，第 6–9 行与第 14–17 行（Fdes）填 NaN。

if nargin < 5, delta_d = 0; end
if nargin < 6, prediction = struct('valid', false); end

stateNext = state;
logVec = nan(25,1);
logVec(24:25) = -1;

Np = coder.const((numel(state.active_main) - 4*numel(state.plan) - 8)/10);
Xlocal = nan(Np+1,1);
Ylocal = nan(Np+1,1);

% 保持值
SW = state.sw_prev;
Tb = state.brake_prev;
Icmd = params.M.cdc_fixed_I*ones(4,1);

% 方向盘：steer_ext 时外部机器人给总转角，这里只输出 AFS 增量
if params.M.steer_ext
    SW_raw = rad2deg(min(max(params.vehicle.isw*state.u_prev(1), ...
        -deg2rad(720)), deg2rad(720)));
elseif params.M.afs_on
    Fyf_ff = sum(meas.Fy([1,3]));
    SW_raw = rad2deg(min(max(params.vehicle.isw*(delta_d + state.u_prev(1) ...
        - params.vehicle.cs_fy*Fyf_ff), -deg2rad(720)), deg2rad(720)));
else
    SW_raw = rad2deg(min(max(params.vehicle.isw*state.u_prev(1), ...
        -deg2rad(720)), deg2rad(720)));
end
max_step = rad2deg(params.vehicle.delta_sw_rate_max*params.M.Ts_exec);
SW = state.sw_prev + min(max(SW_raw - state.sw_prev, -max_step), max_step);

% 诊断字段（不依赖 prepare 的量）
logVec(3) = meas.phi;
logVec(4) = meas.p;
logVec(10:13) = meas.Fd(:);
logVec(18) = state.u_prev(2);
logVec(19) = SW;
logVec(20) = meas.Vx;
logVec(21) = meas.r;

% 无效测量直接返回（保持命令）
if meas.Vx <= 0 || any(~isfinite([meas.X; meas.Y; meas.Vx; meas.Vy; ...
        meas.yaw; meas.r; meas.phi; meas.p; meas.delta_front(:); ...
        meas.steering_wheel; meas.Fx(:); meas.Fy(:); meas.Fz(:); ...
        meas.Fd(:); meas.Td(:)]))
    return
end

% 1. 局部参考 + 轮胎线性化 + 执行器边界
[c, index, valid] = pmpc_base_prepare(meas, Reftraj, params, state, delta_d);
if ~valid, return; end
Xlocal = c.Xlocal;
Ylocal = c.Ylocal;
stateNext.path_index = index;
% 方向盘速率限制是跨拍累积的，必须回写；漏这一句 SW 会被锁在 ±max_step 内
stateNext.sw_prev = SW;

logVec(1) = c.zeta(5);   % ey
logVec(2) = c.zeta(6);   % epsi
logVec(3) = c.zeta(3);   % phi
logVec(4) = c.zeta(4);   % p
logVec(23) = c.duMax(2); % dMz 上限
% 实测侧倾力矩分配：正值表示左侧垂向载荷大于右侧
FzLog = max(meas.Fz(:), 0);
FzTotal = sum(FzLog);
if FzTotal > 1e-6
    logVec(5) = (FzLog(1) + FzLog(2) - FzLog(3) - FzLog(4))/FzTotal;
end
% Md / Fdes 槽位保持 NaN：基线没有 Md 状态与控制量，也不给四角力
% （协调控制器的 6–9 行是 Md_actual/Md_cmd/Md_min/Md_max，14–17 行是 Fdes）。
% 下游画图按 NaN 判断“这条曲线该不该画”，不要往这两段写调试量。

% 2. 预测矩阵
p = pmpc_base_predict(c, params, state.plan);

% 3. QP
[q, candidate, useCandidate] = build_qp(c, p, params, state);

z = zeros(size(q.f));
ok = false;
if ~useCandidate
    [z, ok, stateNext.active_main] = solve_qp(q, state.active_main);
end
if ~ok && useCandidate
    z = candidate;
    stateNext.active_main(:) = false;
end

% 4. 只执行第一项增量
du = zeros(2,1);
nd = coder.const(numel(state.plan));
plan = zeros(nd,1);
candidateUsed = false;
if ok || useCandidate
    if ok
        plan = z(1:nd);
    else
        plan = candidate;
        candidateUsed = true;
    end
    du = plan(1:2);
end
logVec(22) = du(2);
logVec(24) = double(ok);
logVec(25) = double(candidateUsed);

stateNext.u_prev = c.zeta(7:8) + du;

% 候选计划按执行周期对齐（同协调控制器口径）
plan(1:2) = 0;
a = params.M.Ts_exec/params.M.Ts;
stateNext.plan = (1-a)*plan + a*[plan(3:end); zeros(2,1)];

% 5. 分配：Mz -> 四轮制动力矩 QP；电流固定
[Tb, stateNext] = alloc_brake(stateNext.u_prev(2), c, params, stateNext);
Icmd = params.M.cdc_fixed_I*ones(4,1);

logVec(18) = stateNext.u_prev(2);
logVec(19) = SW;
end

%% ------------------------------------------------------------------
function [Tb, s] = alloc_brake(Mz_cmd, c, P, s)
%ALLOC_BRAKE 横摆力矩 -> 四轮制动力矩（与 pmpc_core 同结构、同权重）。
%#codegen
V = P.vehicle;
delta = c.delta;
% Mz = B*T，四列 L1/L2/R1/R2；T 为非负轮端制动力矩
B = [ V.tf/2*cos(delta) - V.lf*sin(delta), V.tr/2, ...
     -V.tf/2*cos(delta) - V.lf*sin(delta), -V.tr/2]/V.rt;
% 轮胎附着利用率惩罚 + 相对上一拍的变化惩罚
G  = diag((1./(V.rt*V.mu*max(c.Fz,1))).^2);
previous = min(max(s.brake_prev, 0), c.brakeMax);
kd = 0.003*(max(V.mu*sum(c.Fz)*V.tf/4, 1)/max(V.rt*V.mu*sum(c.Fz)/4, 1))^2;

if abs(Mz_cmd) <= 1
    % 需求接近零时逐步释放制动
    s.brake_prev = max(previous - 2e4*P.M.Ts_exec, 0);
    s.active_brake(:) = false;
else
    q = struct('H', 2*(B'*B + .1*G + kd*eye(4)), ...
               'f', -2*(B'*Mz_cmd + kd*previous), ...
               'A', zeros(0,4), 'b', zeros(0,1), ...
               'lb', zeros(4,1), 'ub', c.brakeMax);
    [T, ok, s.active_brake] = solve_qp(q, s.active_brake);
    if ok, s.brake_prev = T; else, s.brake_prev = zeros(4,1); end
end
Tb = s.brake_prev;
end

%% ------------------------------------------------------------------
function [q, candidate, useCandidate] = build_qp(c, p, P, s)
%BUILD_QP z = [DeltaU; s_r; s_alpha; s_env]，A*z<=b。
%#codegen
N = coder.const(size(p.Y,1)/6);
K = coder.const(size(p.Y,2)/2);
nd = 2*K;
n  = nd + 4;

% 固定归一化尺度（与协调控制器一致）
VY_REF = 1.0;  R_REF = 0.2; PHI_REF = deg2rad(2); P_REF = deg2rad(10);
EY_REF = 0.3;  EPSI_REF = deg2rad(1);
DFYF_REF = deg2rad(1);  MFX_REF = 500;
EPS_R_REF = deg2rad(1);  EPS_A_REF = deg2rad(1);
EPS_LTR_REF = 0.1;  EPS_E_REF = 0.1;

% 状态顺序 Vy/r/phi/p/ey/epsi
X_REF = [VY_REF; R_REF; PHI_REF; P_REF; EY_REF; EPSI_REF];
Q = diag(P.M.Q_norm./X_REF.^2);
Q = kron(eye(N), Q);

R = kron(eye(K), diag(P.M.R_norm./[DFYF_REF; MFX_REF].^2));

% 增量界：首步按 Ts_exec，后续按 Ts
duLimit = repmat(c.duMax*(P.M.Ts/P.M.Ts_exec), K, 1);
duLimit(1:2) = c.duMax;

H = zeros(n);  f = zeros(n,1);
H(1:nd,1:nd) = 2*(p.Y'*Q*p.Y + R);
H = .5*(H + H') + 1e-8*eye(n);
e = p.y - c.yr;
f(1:nd) = 2*p.Y'*Q*e;

% 松弛权重（基线是固定值，不做自适应），顺序 [r; alpha; LTR; env]
wr = P.M.slack_w(1);
wa = P.M.slack_w(2);
wl = P.M.slack_w(3);
we = P.M.slack_w(4);
f(nd+1) = wr;
f(nd+2) = wa;
f(nd+3) = wl;
f(nd+4) = we;

% 行数：2 控制 × 2（上下）× K = 4K；
% 每节点 2(yaw)+2(alpha)+2(LTR)+4(road) = 10N
A = zeros(4*K + 10*N, n);
b = zeros(4*K + 10*N, 1);

% --- 控制量盒约束（累计增量） ---
for j = 1:K
    for k = 1:2
        row = (j-1)*2 + k;
        A(row, 1:nd)      =  p.AI(row,:);
        A(2*K+row, 1:nd)  = -p.AI(row,:);
        b(row)            =  c.umax(k) - p.Ut(row);
        b(2*K+row)        =  p.Ut(row) - c.umin(k);
    end
end

% --- 状态约束 ---
for j = 1:N
    rows = (j-1)*6 + (1:6);

    % 横摆（2 行）-> s_r
    r = 4*K + (j-1)*4 + (1:2);
    A(r,1:nd) = c.Hsh(1:2,:)*p.Y(rows,:)/EPS_R_REF;
    A(r,nd+1) = -1;
    b(r)      = (c.Gsh(1:2) - c.Hsh(1:2,:)*p.y(rows))/EPS_R_REF;

    % 后轴侧偏（2 行）-> s_alpha
    r = 4*K + (j-1)*4 + 2 + (1:2);
    A(r,1:nd) = c.Hsh(3:4,:)*p.Y(rows,:)/EPS_A_REF;
    A(r,nd+2) = -1;
    b(r)      = (c.Gsh(3:4) - c.Hsh(3:4,:)*p.y(rows))/EPS_A_REF;

    % LTR（2 行）-> s_LTR；含转向输入与轮胎偏置项
    r = 4*K + 4*N + (j-1)*2 + (1:2);
    ur = (min(j,K)-1)*2 + (1:2);
    A(r,1:nd) = (c.Hr*p.Y(rows,:) + c.Or*p.AI(ur,:))/EPS_LTR_REF;
    A(r,nd+3) = -1;
    % LTR = kL*(kg*ay*x + Kt*phi + Md)，冻结 Md 进仿射项（与 Proposed 同号）
    b(r)      = (c.Gr - c.MdLTR - c.Hr*p.y(rows) - c.Or*p.Ut(ur))/EPS_LTR_REF;

    % 道路包络（4 行）-> s_env
    r = 4*K + 6*N + (j-1)*4 + (1:4);
    A(r,1:nd) = c.Henv*p.Y(rows,:)/EPS_E_REF;
    A(r,nd+4) = -1;
    b(r)      = (c.Genv + c.kapNode(j)*c.gkap - c.Henv*p.y(rows))/EPS_E_REF;
end

lb = -[duLimit; 0; 0; 0; 0];
ub = [ duLimit; Inf; Inf; Inf; Inf];

q = struct('H', H, 'f', f, 'A', A, 'b', b, 'lb', lb, 'ub', ub);

% --- 候选计划：投影上一拍计划到本拍可行域，供 QP 失败时回退 ---
candidate = zeros(nd,1);
u = c.zeta(7:8);
for j = 1:K
    k = (j-1)*2 + (1:2);
    next = s.plan(k) + u;
    next = min(max(next, max(c.umin, u - duLimit(k))), ...
                      min(c.umax, u + duLimit(k)));
    candidate(k) = next - u;
    u = next;
end
useCandidate = false;
end

%% ------------------------------------------------------------------
function [x, ok, activeNext] = solve_qp(q, activePrev)
%SOLVE_QP 官方 active-set 求解器（与 pmpc_core 同实现）。
%#codegen
coder.inline('always');
n = coder.const(size(q.H,1));
m = coder.const(size(q.A,1));
x = zeros(n,1);  ok = false;
activeNext = false(m + 2*n, 1);

if any(~isfinite(q.H(:))) || any(~isfinite(q.f)) || ...
        any(~isfinite(q.A(:))) || any(~isfinite(q.b)) || ...
        any(isnan(q.lb)) || any(isnan(q.ub)) || any(q.lb > q.ub)
    return
end

fixed = isfinite(q.lb) & isfinite(q.ub) & (q.lb == q.ub);
base  = zeros(n,1);
base(fixed) = q.lb(fixed);
H = .5*(q.H + q.H');

% 缩放 x = base + map.*y
scale = 1./sqrt(max(diag(H), eps*max(1, max(diag(H)))));
map   = scale.*double(~fixed);
f     = map.*(q.f + H*base);
H     = (map*map').*H + diag(double(fixed));
H     = .5*(H + H');

A = zeros(m + 2*n, n);
b = ones(m + 2*n, 1);
A(1:m,:) = bsxfun(@times, q.A, map');
b(1:m)   = q.b - q.A*base;
for j = 1:n
    if ~fixed(j) && isfinite(q.lb(j))
        A(m+j,j)   = -1;  b(m+j)   = (base(j)-q.lb(j))/scale(j);
    end
    if ~fixed(j) && isfinite(q.ub(j))
        A(m+n+j,j) =  1;  b(m+n+j) = (q.ub(j)-base(j))/scale(j);
    end
end

norms = sqrt(sum(A.^2, 2));
rowscale = max(norms, 1e-12);
rowscale(norms == 0) = 1;
A = bsxfun(@rdivide, A, rowscale);
b = b./rowscale;

if any(~isfinite(H(:))) || any(~isfinite(f)) || ...
        any(~isfinite(A(:))) || any(~isfinite(b)), return; end
[~, notPD] = chol(H);
if notPD ~= 0, return; end

active = logical(activePrev(:));
active(norms == 0) = false;
if nnz(active) > n || (any(active) && rank(A(active,:)) < nnz(active))
    active(:) = false;
end

options = mpcActiveSetOptions('DataType','double');
options.MaxIterations = 200;
options.ConstraintTolerance = 1e-6;
options.UseHessianAsInput = true;
options.IntegrityChecks = false;

[y, status, act] = mpcActiveSetSolver(H, f, A, b, ...
    zeros(0,n), zeros(0,1), active, options);
ok = status > 0 && all(isfinite(y)) && max([0; A*y - b]) <= 1e-5;

if ~ok && any(active)
    [y, status, act] = mpcActiveSetSolver(H, f, A, b, ...
        zeros(0,n), zeros(0,1), false(m+2*n,1), options);
    ok = status > 0 && all(isfinite(y)) && max([0; A*y - b]) <= 1e-5;
end

if ok
    x = base + map.*y;
    ok = all(isfinite(x));
    if ok, activeNext = act; end
end
end
