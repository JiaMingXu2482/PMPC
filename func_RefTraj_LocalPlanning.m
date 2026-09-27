function [WPIndex,RefU,Kap_dyn,PrjP,Kap_node] = func_RefTraj_LocalPlanning( ...
    MPCParameters, VehiclePara, WayPoints_Index, WayPoints_Collect, ...
    VehStateMeasured, ParaHAT)
% func_RefTraj_LocalPlanning
% 输出：
%   WPIndex   : 最近路点索引（>0有效；0/负值表示异常/末端）
%   RefU      : 参考输入 [delta_des; Vyr; Psidotr] 按 Np 堆叠 (3*Np x 1)
%   Kap_dyn   : 第 p 个预测步(节点 p-1 -> p)的路径平均曲率, 进预测模型 (Np x 1)
%   Kap_node  : 节点 p 处的路径曲率, 进车道四角点约束的 -kappa*a^2/2 (Np x 1)
%   PrjP      : 投影点与误差结构体
% -------------------------------------------------------------------------

%% ===================== Parameters Initialization ===================== %%
L   = VehiclePara.L;     % 轴距
m   = VehiclePara.m;
Lf  = VehiclePara.lf;
Lr  = VehiclePara.lr;
mu  = VehiclePara.mu;    % 路面附着系数（可实时估计）
g   = VehiclePara.g;     % 重力加速度

CafHat = VehiclePara.CafHat;
CarHat = VehiclePara.CarHat;

K = -1*(m/(2*L^2)) * (Lf/CarHat - Lr/CafHat);

Np = MPCParameters.Np;   % 预测步数
Ts = MPCParameters.Ts;   % 单一采样时间

% ---- 单一采样时间：预测时间栅格 Tk(k)=Ts ----
Tk = Ts * ones(Np,1);
%  2026-09-26 对照试验: first_Tc = 1 时第一步用执行周期 Ts_exec (非均匀预测网格),
%  与 func_DynamicalModel 第一步的离散步长一致。
if MPCParameters.first_Tc
    Tk(1) = MPCParameters.Ts_exec;
end

Cf = CafHat;
Cr = CarHat;

% -------- measured/estimated vehicle state --------
Vel    = VehStateMeasured.x_dot;
%  2026-09-26: 参考点改为质心。CarSim 的 Xo/Yo 是车辆原点 = 前轴中心(用 Xcg_TM 实测,
%  质心在其后 lf), 而预测模型 de_y/dt = V_y + V*e_psi 与车道四角点尺寸都按质心 ——
%  在前轴处量 e_y 漏掉 lf*r, 实际输入下 0.6 s 处的预测误差 RMS 0.42 m (质心处 0.12 m)。
%  论文 II-B 的 e_y 本来就定义在质心。见 改进.md 18q。
PosPsi = VehStateMeasured.Yaw;
PosX   = VehStateMeasured.X - Lf*cos(PosPsi);
PosY   = VehStateMeasured.Y - Lf*sin(PosPsi);

%% ===================== Default Outputs (robust) ====================== %%
WPIndex  = -1;
Kap_dyn  = zeros(Np,1);
Kap_node = zeros(Np,1);
RefU     = repmat([0;0;0], Np, 1);

PrjP = struct('ey',0,'epsi',0,'Velr',Vel,'xr',PosX,'yr',PosY,'psir',PosPsi);

%% ================= WaypointData2VehicleCoords ======================== %%
WPNum = length(WayPoints_Collect(:,1));

% -------- 寻找参考路径上距车辆最近的点 --------
Dist_MIN  = 1000;
index_min = 0;

WayPoints_Index = max(1, min(WayPoints_Index, WPNum)); % 防护
for i = WayPoints_Index:1:WPNum
    deltax0 = WayPoints_Collect(i,2) - PosX;
    deltay0 = WayPoints_Collect(i,3) - PosY;
    Dist = sqrt(deltax0.^2 + deltay0.^2);
    if Dist < Dist_MIN
        Dist_MIN  = Dist;
        index_min = i;
    end
end

if (index_min < 1)
    WPIndex = -1;
    return;
else
    if (index_min >= WPNum)
        WPIndex = -2;
        return;
    else
        WPIndex = index_min;
    end
end

%% ====================== If found nearest point ======================= %%
if (WPIndex > 0)

    % --- 计算投影点与误差（使用你文件里的 spline-window 方法） ---
    [PPx, PPy, Psi0, ey, epsi] = func_error(WayPoints_Collect, PosX, PosY, PosPsi);
    PrjP.ey   = -ey;
    PrjP.epsi = -epsi;
    PrjP.Velr = Vel;
    PrjP.xr   = PPx;
    PrjP.yr   = PPy;
    PrjP.psir = Psi0;

    % --- 预测节点的弧长与路径曲率 (2026-09-26, 取代单段三次 Bezier 拟合) ---
    %  原做法: 在前方路径点上拟合一段三次 Bezier, 按 Bezier 参数(不是弧长)均匀采样求曲率;
    %  三次曲线的曲率只能近似线性变化, 实测比车辆位置超前约 2.5 个预测步 (改进.md 18q)。
    %  路径文件本身带精确航向(第 4 列)与曲率(第 5 列 = dpsi/ds), 直接按弧长取:
    %    节点 p 的弧长 s_p = s_0 + Vel*sum(Tk(1:p)),  s_0 = 质心投影点的弧长;
    %    Kap_dyn(p)  = (psi(s_p) - psi(s_{p-1}))/(s_p - s_{p-1}): 该步的平均曲率,
    %                  ZOH 下 e_psi 的积分项 -int(kappa ds) 因此是精确的;
    %    Kap_node(p) = kappa(s_p): 车道约束在节点 p 处的弯道修正 -kappa*a^2/2。
    %  路径两端之外航向、曲率取端值, 即按直线延伸。
    StepLength_S = 0;
    for i = 1:Np
        StepLength_S = StepLength_S + Vel * Tk(i);
    end
    s0 = local_arcpos(WayPoints_Collect, index_min, PosX, PosY);
    s_prev = s0;
    for i = 1:Np
        s_i = s_prev + Vel * Tk(i);
        dpsi = local_interp(WayPoints_Collect(:,7), WayPoints_Collect(:,4), s_i) ...
             - local_interp(WayPoints_Collect(:,7), WayPoints_Collect(:,4), s_prev);
        Kap_dyn(i)  = dpsi / max(s_i - s_prev, 1e-6);
        Kap_node(i) = local_interp(WayPoints_Collect(:,7), WayPoints_Collect(:,5), s_i);
        s_prev = s_i;
    end
    % 剩余路径不足一个预测时域: 置 WPIndex = 0 (原语义, 上层据此不更新路点索引)
    if WayPoints_Collect(WPNum,7) - s0 < StepLength_S
        WPIndex = 0;
    end

    %% ---- 生成 RefU：delta_des, Vyr, Psidotr ----
    %  codegen(R2018a): 原来是 RefU_cell = cell(Np,1) + 逐个赋值 + cell2mat。
    %  循环确实覆盖了 1:Np, 但 R2018a 证明不了, 报
    %    "Codegen requires that every cell-array element contained in 'RefU_cell'
    %     be assigned a value before being passed into 'cell2mat'."
    %  改成预分配数值数组直接写 —— 与 cell2mat 的竖向拼接逐位等价。
    %  (阶段 C 处理 func_SystemFurture / func_DynamicalModel 用的是同一手法。)
    RefU = zeros(3*Np, 1);
    for i = 1:Np
        delta_des = L * Kap_node(i) * (1 + K * Vel^2);

        numerator_vy   = (1 - (m*Lf*Vel^2)/(2*L*Cf*Lr)) * Lr * Vel * delta_des;
        denominator_vy = L * (1 + K*Vel^2);
        vy_ref_unconstrained = numerator_vy / max(1e-6, denominator_vy);

        vy_friction_limit = Vel * atan(0.02*mu*g);
        Vyr = min(abs(vy_ref_unconstrained), vy_friction_limit) * sign(delta_des);

        numerator_psi   = Vel * delta_des;
        denominator_psi = L * (1 + K*Vel^2);
        psi_dot_ref_unconstrained = numerator_psi / max(1e-6, denominator_psi);

        psi_dot_friction_limit = min(abs(psi_dot_ref_unconstrained), mu*g / max(1e-3, Vel));
        Psidotr = psi_dot_friction_limit * sign(delta_des);

        RefU(3*i-2 : 3*i) = [delta_des; Vyr; Psidotr];
    end

end % if WPIndex > 0

end % ===== end of main function =====


%==========================================================================
% sub functions
%==========================================================================

function K = GetPathHeading(Xb,Yb,Xn,Yn)
% Heading angle in [-pi, pi]
AngleY = Yn - Yb;
AngleX = Xn - Xb;
K = atan2(AngleY, AngleX);
end

function [PPx,PPy,de] = func_GetProjectPoint(Xb,Yb,Xn,Yn,Xc,Yc)
% de 与点到直线距离符号相反：左正右负（此处保持原逻辑）
if Xn==Xb
    x=Xn; y=Yc; de=Xc-Xn;
else
    if Yb==Yn
        x=Xc; y=Yn; de=Yn-Yc;
    else
        DifX=Xn-Xb; DifY=Yn-Yb;
        Kindex=DifY/DifX;
        bindex=Yn-Kindex*Xn;
        K=(-1)/Kindex;
        b=Yc-K*Xc;
        x=(bindex-b)/(K-Kindex);
        y=K*x+b;
        de=(Kindex*Xc+bindex-Yc)/sqrt(1+Kindex*Kindex);
    end
end
PPx=x; PPy=y;
end

function K = func_CalPathCurve(XA,YA,XB,YB,XC,YC)
% 三点求圆半径再求曲率（保留原实现）
if XB==XA
    mr=inf;
else
    mr=(YB-YA)/(XB-XA);
end
if XC==XB
    mt=inf;
else
    mt=(YC-YB)/(XC-XB);
end

if mr==mt
    Rff=inf;
else
    if mt==0
        if mr==inf
            Rff=sqrt((XA-XC)^2+(YA-YC)^2)/2;
        else
            mrsubmt=1/(2*(mr-mt));
            Xff=(mr*mt*(YC-YA)+mr*(XB+XC)-mt*mrsubmt*(XA+XB));
            Yff=(YB+YA)/2-(Xff-(XB+XA)/2)/mr;
            Rff=sqrt((XA-Xff)^2+(YA-Yff)^2);
        end
    elseif mt==inf
        if mr==0
            Rff=sqrt((XA-XC)^2+(YA-YC)^2)/2;
        else
            Yff=(YB+YC)/2;
            Xff=(XA+XB)/2-mr*(YC-YA)/2;
            Rff=sqrt((XA-Xff)^2+(YA-Yff)^2);
        end
    else
        mtdao=1/mt;
        if mr==0
            mrsubmt=1/(2*(mr-mt));
            Xff=(mr*mt*(YC-YA)+mr*(XB+XC)-mt*mrsubmt*(XA+XB));
            Yff=(YB+YC)/2-mtdao*(Xff-(XB+XC)/2);
            Rff=sqrt((XA-Xff)^2+(YA-Yff)^2);
        elseif mr==inf
            Yff=(YA+YB)/2;
            Xff=(XB+XC)/2+mt*(YA-YC)/2;
            Rff=sqrt((XA-Xff)^2+(YA-Yff)^2);
        else
            mrsubmt=1/(2*(mr-mt));
            Xff=(mr*mt*(YC-YA)+mr*(XB+XC)-mt*mrsubmt*(XA+XB));
            Yff=(YB+YC)/2-mtdao*(Xff-(XB+XC)/2);
            Rff=sqrt((XA-Xff)^2+(YA-Yff)^2);
        end
    end
end

K1=GetPathHeading(XA,YA,XB,YB);
K2=GetPathHeading(XB,YB,XC,YC);
if K2 > K1
    Rff=Rff;
else
    Rff=-1*Rff;
end
K=1/Rff;
end

function curvature = func_CalPathCurve_YU(X1,Y1,X2,Y2,X3,Y3)
delta_x = X2 - X1; delta_y = Y2 - Y1;
a = sqrt(delta_x.^2 + delta_y.^2);
delta_x = X3 - X2; delta_y = Y3 - Y2;
b = sqrt(delta_x.^2 + delta_y.^2);
delta_x = X1 - X3; delta_y = Y1 - Y3;
c = sqrt(delta_x.^2 + delta_y.^2);

CLOSE_TO_ZERO = 0.01;
if (a < CLOSE_TO_ZERO || b < CLOSE_TO_ZERO || c < CLOSE_TO_ZERO)
    curvature = 0;
    return;
end

s = (a + b + c)/2.0;
K = sqrt(abs(s*(s-a)*(s-b)*(s-c)));
curvature = 4*K/(a*b*c);

rotate_direction = (X2 - X1)*(Y3 - Y2) - (Y2 - Y1)*(X3 - X2);
if (rotate_direction < 0)
    curvature = -curvature;
end
end

function [PPx,PPy,Psi0,e_y,e_psi] = func_error(WayPoints_Collect, PosX, PosY, PosPsi)
% spline-window 投影点与误差计算（保持原逻辑，修正形参命名更清晰）
xo = PosX; yo = PosY; psi = PosPsi;

interp_points = 1500;
window_size   = 15;

X_ref   = WayPoints_Collect(:,2);
Y_ref   = WayPoints_Collect(:,3);
psi_ref = WayPoints_Collect(:,4);

dists = sqrt((X_ref - xo).^2 + (Y_ref - yo).^2);
%  codegen(R2018a): 用 (:) 把输入强制成列向量, 消除"working dimension"歧义。
%  变长向量上 min/max/sum 若无法在编译期确定沿哪一维, 运行到某一拍会报
%    "The working dimension was selected automatically, is variable-length,
%     and has length 1 at run time."
%  对向量而言 v(:) 只是 reshape, 数值与线性下标都不变。
[~, idx_nearest_] = min(dists(:));
idx_nearest = idx_nearest_(1);       % 同上: 钉成标量


start_idx = max(1, idx_nearest - floor(window_size/2));
end_idx   = min(length(X_ref), start_idx + window_size);

if end_idx - start_idx < 5
    start_idx = max(1, end_idx - 5);
end

local_X   = X_ref(start_idx:end_idx);
local_Y   = Y_ref(start_idx:end_idx);
local_psi = psi_ref(start_idx:end_idx);

interp_idx = linspace(1, length(local_X), interp_points);

X_interp   = spline(1:length(local_X), local_X, interp_idx);
Y_interp   = spline(1:length(local_Y), local_Y, interp_idx);
psi_interp = spline(1:length(local_psi), local_psi, interp_idx);

interp_dists = sqrt((X_interp - xo).^2 + (Y_interp - yo).^2);
%  codegen(R2018a): 必须把索引强制成标量。
%  local_X = X_ref(start_idx:end_idx) 的长度是运行时决定的, 于是 X_interp 变尺寸;
%  min() 对可能为空的变尺寸向量, 其索引输出被推成 [1 x :?] 而不是 [1 x 1],
%  X_interp(idx_interp) 随之变尺寸, 一路传到 PrjP.ey/epsi, 最后在
%  func_ReportStatus 里撞上固定 1x1 的 emax.y, 报
%    "Dimension 2 is fixed on the left-hand side but varies on the right"
%  取 (1) 只是把类型钉成标量; 运行时 min 必然返回单个下标(1500 个插值点非空),
%  所以数值完全不变。
[~, idx_interp_] = min(interp_dists(:));   % 同上: 强制列向量
idx_interp = idx_interp_(1);

PPx = X_interp(idx_interp);
PPy = Y_interp(idx_interp);

dx = X_interp(idx_interp) - xo;
dy = Y_interp(idx_interp) - yo;
Psi0 = atan2(dy, dx);

e_y = -dx*sin(psi) + dy*cos(psi);

% wrapToPi 兼容：若没有 Mapping Toolbox，可用本地实现
% codegen: exist(...,'file') 不支持; local_wrapToPi 与 wrapToPi 数学等价,
% 只在恰好等于 ±pi 时端点归属不同(实际不会发生)。固定用本地实现。
e_psi = local_wrapToPi(psi_interp(idx_interp) - psi);
end

function ang = local_wrapToPi(ang)
%  必须与 MATLAB 的 wrapToPi 语义一致: 它对已在 [-pi,pi] 内的角度**原样返回**。
%  无条件做 mod 会多一次浮点 round-trip, 改掉末位 —— 实测会让闭环结果变化。
%
%  codegen(R2018a): 原来直接写 `if ang < -pi || ang > pi`, R2018a 推不出 ang 是标量,
%  报 "Expected a scalar. Non-scalars are not supported with logical operators."
%  改成逐元素循环: 每次比较的都是标量, 且语义与原来**完全一致**(标量输入下逐位相同)。
for k = 1:numel(ang)
    a = ang(k);
    if a < -pi || a > pi
        ang(k) = mod(a + pi, 2*pi) - pi;
    end
end
end

function s = local_arcpos(W, idx, x, y)
%LOCAL_ARCPOS  点 (x,y) 在参考折线上的投影点弧长 (第 7 列 = 弧长)
%  只看最近路点两侧的两段; 首段允许负值(车在路径起点之前), 末段允许超出终点。
n = size(W,1);
s = W(idx,7);
dmin = inf;
for j = max(idx-1,1):min(idx,n-1)
    tx = W(j+1,2) - W(j,2);  ty = W(j+1,3) - W(j,3);
    L2 = tx*tx + ty*ty;
    if L2 > 0
        u = ((x - W(j,2))*tx + (y - W(j,3))*ty) / L2;
        if j > 1,   u = max(u, 0); end
        if j < n-1, u = min(u, 1); end
        px = W(j,2) + u*tx;  py = W(j,3) + u*ty;
        d  = (x - px)*(x - px) + (y - py)*(y - py);
        if d < dmin
            dmin = d;
            s = W(j,7) + u*(W(j+1,7) - W(j,7));
        end
    end
end
end

function v = local_interp(S, V, s)
%LOCAL_INTERP  S 单调递增时的分段线性插值; 两端之外取端值
n = numel(S);
if s <= S(1)
    v = V(1);
elseif s >= S(n)
    v = V(n);
else
    lo = 1;  hi = n;
    while hi - lo > 1
        mid = floor((lo + hi)/2);
        if S(mid) <= s, lo = mid; else, hi = mid; end
    end
    v = V(lo) + (V(hi) - V(lo)) * (s - S(lo)) / (S(hi) - S(lo));
end
end
