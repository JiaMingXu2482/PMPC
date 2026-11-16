function [c, index, valid] = pmpc_base_prepare(meas, Reftraj, P, state, delta_d)
%PMPC_BASE_PREPARE 基线 MPC 的局部参考、轮胎线性化与约束。
%#codegen
%
% 与 pmpc_core/prepare_step 的区别：
%   zeta 为 8 维 [Vy; r; phi; p; ey; epsi; u_prev(1:2)]
%   侧倾自由度与 LTR 约束保留（与协调控制器同一个被控对象模型）
%   控制量只有 [Delta delta; Mz]，没有减振器力 Md
%   减振器恒为固定电流，因此无力界 lo/hi、无 tau_d 时滞
%
% 输入：
%   meas      测量结构（pmpc_base_init 的 meas0 同结构）
%   Reftraj   参考路径 [index, X, Y, heading, curvature, 0, station]
%   P         参数结构（含 vehicle / tireF / tireR / road / M）
%   state     跨拍状态（u_prev / plan / path_index / sw_prev / brake_prev / active_*）
%   delta_d   驾驶员前轮转角 [rad]，u(1)=Delta delta 为 AFS 附加角
%
% 输出：
%   c     本拍预测与约束数据
%   index 路径进度（下一拍起点）
%   valid false 表示测量/参考无效，本拍应保持命令

if nargin < 5, delta_d = 0; end

V  = P.vehicle;
vx = max(meas.Vx, 0.5);
delta = delta_d + state.u_prev(1);     % 总前轮转角
Fx = meas.Fx;  Fy = meas.Fy;  Fz = meas.Fz;

% 预测时域结构尺寸从 active_main 反推：4K(控制) + 10N(状态) + 2n(松弛)
Np = coder.const((numel(state.active_main) - 4*numel(state.plan) - 8)/10);

%% 轮胎工作点
alpha = [(meas.Vy + V.lf*meas.r)/vx - delta; ...
         (meas.Vy - V.lr*meas.r)/vx];
[Ff, Cf] = tire(P.tireF, alpha(1), sum(Fz([1,3])));
[Fr, Cr] = tire(P.tireR, alpha(2), sum(Fz([2,4])));

% 摩擦圆：纵向力占用后的剩余能力
Fz = max(Fz, 1);
xi = sqrt(max(1 - (Fx./(V.mu*Fz)).^2, 0));
xf = sum(Fz([1,3]).*xi([1,3]))/sum(Fz([1,3]));
xr = sum(Fz([2,4]).*xi([2,4]))/sum(Fz([2,4]));
Ff = xf*Ff;  Fr = xr*Fr;
Cf = -max(abs(xf*Cf), 0.032*abs(V.CafHat));
Cr = -max(abs(xr*Cr), 0.032*abs(V.CarHat));

% 轮胎切线偏置：Fy = C*alpha + offset
% offsetPred 是预测模型里实际使用的偏置（扣掉驾驶员转角项）；
% offset 保留实测口径，供分配环节使用。
offset = [Ff - Cf*alpha(1); Fr - Cr*alpha(2)];
offsetPred = offset;
offsetPred(1) = offset(1) - Cf*delta_d;

%% 局部参考投影与前视
[ey, epsi, kap, kapNode, yr, index, valid, Xlocal, Ylocal] = ...
    local_reference(meas, Reftraj, P, state.path_index, Np);

% zeta = [Vy; r; phi; p; ey; epsi; 上一拍 Delta delta; 上一拍 Mz]
zeta = [meas.Vy; meas.r; meas.phi; meas.p; ey; epsi; state.u_prev];

%% 控制量绝对界
if P.M.afs_on
    deltaMax =  V.delta_sw_max/V.isw - delta_d;
    deltaMin = -V.delta_sw_max/V.isw - delta_d;
    if deltaMin > deltaMax, deltaMax = 0; deltaMin = 0; end
else
    deltaMax = 0;  deltaMin = 0;      % 转向由外部机器人给，AFS 增量固定 0
end

% Mz 由四轮制动力矩的附着极限换算
brakeMax = max(V.rt*sqrt(max((V.mu*Fz).^2 - Fy.^2, 0)) - max(meas.Td(:),0), 0);
if abs(delta) < 0.15
    Mzpos = V.tf/(2*V.rt)*(brakeMax(1) + brakeMax(2));
    Mzneg = -V.tf/(2*V.rt)*(brakeMax(3) + brakeMax(4));
else
    Mzpos = (V.tf/2*(brakeMax(1)*cos(delta)+brakeMax(2)) ...
             - V.lf*sin(delta)*brakeMax(1))/V.rt;
    Mzneg = (-V.tf/2*(brakeMax(3)*cos(delta)+brakeMax(4)) ...
             - V.lf*sin(delta)*brakeMax(3))/V.rt;
end
Mzpos = max(Mzpos, 0);
Mzneg = min(Mzneg, 0);
if ~P.M.mz_on, Mzpos = 0; Mzneg = 0; end

umax = [deltaMax; Mzpos];
umin = [deltaMin; Mzneg];

%% 控制增量界
duDelta = V.delta_sw_rate_max/V.isw * P.M.Ts_exec;
mzref   = V.mu*V.m*V.g/2*(V.tf+V.tr)/4;
duMz    = mzref * P.M.Ts_exec / P.M.tau_mz;
duMax   = [duDelta; duMz];

% 绝对界与增量可达区间求交，避免上一拍命令卡死
umax = max(umax, zeta(7:8) - duMax);
umin = min(umin, zeta(7:8) + duMax);

%% 状态约束
% Zeng 鞍结稳定域：|r| <= mu*g/vx，|alpha_r| <= min(a_sat, a_sl)
Fzr_st = V.m*V.g*V.lf/V.L;
Cr_cur = max(abs(V.CbarR), 1e3);
a_sat  = atan(V.mu*Fzr_st/Cr_cur);
a_sl   = atan(3*V.mu*Fzr_st/abs(V.CarHat));
alpha_r_sat = min(a_sat, a_sl);
rmax = V.mu*V.g/vx;

% 6 个状态 [Vy; r; phi; p; ey; epsi] -> 只涉及 r、Vy、ey、epsi
Hsh = [0,  1, 0, 0, 0, 0;
       0, -1, 0, 0, 0, 0;
       1/vx, -V.lr/vx, 0, 0, 0, 0;
      -1/vx,  V.lr/vx, 0, 0, 0, 0];
Gsh = [rmax; rmax; alpha_r_sat; alpha_r_sat];

% 道路包络：前轴参考点在 [Y-a_off, Y+rear] 内
af   = .860 + V.lf;
rear = 3.972 - V.lf;
Henv = [0, 0, 0, 0,  1,  af;
        0, 0, 0, 0,  1, -rear;
        0, 0, 0, 0, -1, -af;
        0, 0, 0, 0, -1,  rear];
Genv = (P.road.width/2 - 1.96/2 - .2)*ones(4,1);

% LTR 约束（|LTR| <= 0.8），含状态、转向输入及轮胎偏置
ay = [(Cf*cos(delta) + Cr)/(V.m*vx), ...
      (Cf*V.lf*cos(delta) - Cr*V.lr)/(V.m*vx), zeros(1,4)];
kg = V.m*V.h_TL - V.ms*V.h_S2R;

% 固定电流下的被动阻尼力矩：与 Proposed 完全相同的力臂/轮序定义。
% baseline 不把 Md 作为可优化输入，只把它当作预测域内冻结的已知外扰。
Bd = .5*[-V.ldf; -V.ldr; V.ldf; V.ldr];
MdFrozen = Bd' * meas.Fd(:);
% 诊断开关：md_frozen_on=false 时退回修复前行为（扰动恒 0），用于回归对照。
if isfield(P.M,'md_frozen_on') && ~P.M.md_frozen_on
    MdFrozen = 0;
end
kL = 2/(V.m*V.g*V.tf);
Hr = kL*(kg*ay + [0 0 V.Kt 0 0 0]);
Or = kL*kg*[-Cf*cos(delta)/V.m, 0];
Gr = [.8; .8] - kL*kg*(offsetPred(1)*cos(delta) + offsetPred(2))/V.m*[1; -1];
MdLTR = kL*MdFrozen;   % LTR 中由冻结 Md 贡献的标量项（与 Proposed 的 +k*Md 同号）

%% 打包
c = struct('zeta', zeta, 'ey', ey, 'Cf', Cf, 'Cr', Cr, 'offset', offsetPred, ...
    'v', vx, 'delta', delta, 'delta_d', delta_d, ...
    'Fyf', sum(Fy([1,3])), ...
    'kap', kap, 'kapNode', kapNode, 'yr', yr, 'Fz', Fz, ...
    'brakeMax', brakeMax, 'umin', umin, 'umax', umax, 'duMax', duMax, ...
    'Hsh', Hsh, 'Gsh', Gsh, ...
    'Hr', [Hr; -Hr], 'Gr', Gr, 'Or', [Or; -Or], ...
    'MdFrozen', MdFrozen, 'MdLTR', MdLTR, ...
    'Henv', Henv, 'Genv', Genv, ...
    'gkap', [af^2; rear^2; -af^2; -rear^2]/2, ...
    'Xlocal', Xlocal, 'Ylocal', Ylocal);

end

%% ------------------------------------------------------------------
function [F, C] = tire(T, alpha, Fz)
%Tire 载荷相关 Fiala 轮胎力与切线刚度（与 pmpc_core/prepare_step 同式）。
%#codegen
z    = min(max(Fz, T.fzlo), T.fzhi);
C    = max(Fz,0)*polyval(T.cq, z);
peak = max(max(Fz,0)*polyval(T.mq, z), 1);
C    = max(C/(1 + T.kcs*C), 1);
t    = tan(min(max(alpha, -1.4), 1.4));
if abs(t) >= 3*peak/C
    F = -peak*sign(t);  C = 0;
else
    F = -C*t + C^2/(3*peak)*abs(t)*t - C^3/(27*peak^2)*t^3;
    C = (-C + 2*C^2/(3*peak)*abs(t) - C^3/(9*peak^2)*t^2)*(1 + t^2);
end
end

%% ------------------------------------------------------------------
function [ey, epsi, kap, kn, yr, index, valid, Xlocal, Ylocal] = ...
    local_reference(m, W, P, previous, N)
%LOCAL_REFERENCE 局部样条投影 + 恒速前视，与 pmpc_core 同算法。
%#codegen
V = P.vehicle;
N = coder.const(N);
ey = 0; epsi = 0;
kap = zeros(N,1);  kn = zeros(N,1);  yr = zeros(6*N,1);
Xlocal = zeros(N+1,1);  Ylocal = zeros(N+1,1);
index = previous;  valid = false;
if size(W,1) < 6 || size(W,2) < 7, return; end

% 前轴参考点 -> 质心
x = m.X - V.lf*cos(m.yaw);
y = m.Y - V.lf*sin(m.yaw);

start = max(1, min(previous, size(W,1)));
[~, j] = min((W(start:end,2)-x).^2 + (W(start:end,3)-y).^2);
index = start + j - 1;
if index >= size(W,1), return; end
[~, j] = min((W(:,2)-x).^2 + (W(:,3)-y).^2);

% 邻近 16 点三次样条，求横向误差与航向误差
a = max(1, j-7);  b = min(size(W,1), a+15);
if b - a < 5, a = max(1, b-5); end
t   = linspace(1, b-a+1, 1500);
xx  = spline(1:b-a+1, W(a:b,2), t);
yy  = spline(1:b-a+1, W(a:b,3), t);
psi = spline(1:b-a+1, W(a:b,4), t);
[~, j] = min((xx-x).^2 + (yy-y).^2);
ey     = (xx(j)-x)*sin(m.yaw) - (yy(j)-y)*cos(m.yaw);
epsi   = -(mod(psi(j) - m.yaw + pi, 2*pi) - pi);

% 投影到相邻折线段，得到弧长 station
station = W(index,7);  distance = Inf;
for s = max(index-1,1):min(index, size(W,1)-1)
    dx = W(s+1,2)-W(s,2);  dy = W(s+1,3)-W(s,3);
    len = dx^2 + dy^2;
    if len > 0
        u = ((x-W(s,2))*dx + (y-W(s,3))*dy)/len;
        if s > 1, u = max(u,0); end
        if s < size(W,1)-1, u = min(u,1); end
        d = (x-W(s,2)-u*dx)^2 + (y-W(s,3)-u*dy)^2;
        if d < distance
            distance = d;
            station  = W(s,7) + u*(W(s+1,7)-W(s,7));
        end
    end
end

sCur = min(max(station, W(1,7)), W(end,7));
Xlocal(1) = interp1(W(:,7), W(:,2), sCur, 'linear');
Ylocal(1) = interp1(W(:,7), W(:,3), sCur, 'linear');

% 恒速前视：稳态转向的 (vy_d, r_d) 作为参考状态前两维
K   = -V.m/(2*V.L^2)*(V.lf/V.CarHat - V.lr/V.CafHat);
for j = 1:N
    next  = station + m.Vx*P.M.Ts;
    sCur  = min(max(station, W(1,7)), W(end,7));
    sQ    = min(max(next,    W(1,7)), W(end,7));
    Xlocal(j+1) = interp1(W(:,7), W(:,2), sQ, 'linear');
    Ylocal(j+1) = interp1(W(:,7), W(:,3), sQ, 'linear');
    ps0 = interp1(W(:,7), W(:,4), sCur, 'linear');
    ps1 = interp1(W(:,7), W(:,4), sQ,   'linear');
    kap(j) = (ps1 - ps0)/max(next - station, 1e-6);
    kn(j)  = interp1(W(:,7), W(:,5), sQ, 'linear');
    station = next;

    dlt = V.L*kn(j)*(1 + K*m.Vx^2);
    den = max(V.L*(1 + K*m.Vx^2), 1e-6);
    vy  = (1 - V.m*V.lf*m.Vx^2/(2*V.L*V.CafHat*V.lr))*V.lr*m.Vx*dlt/den;
    r   = m.Vx*dlt/den;
    yr((j-1)*6+1) = min(abs(vy), m.Vx*atan(.02*V.mu*V.g))*sign(dlt);
    yr((j-1)*6+2) = min(abs(r),  V.mu*V.g/max(m.Vx,1e-3))*sign(dlt);
end
valid = true;
end
