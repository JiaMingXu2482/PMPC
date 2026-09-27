function M = func_Metrics(tag, opt)
%FUNC_METRICS  从一份 ERD 算闭环评价指标
%   M = func_Metrics('<dir>\ZENG')          % 不带扩展名
%   M = func_Metrics(D)                     % 也可直接传 func_ReadERD 的结果
%
%   阶段 S(换求解器 quadprog -> mpcqpsolver) 时 MD5 逐字节验收必然失效,
%   验收改用这些指标。见 PLAN_HIL迁移方案.md §4。
%
%   opt 字段(可选):
%     .mu      路面附着 (默认 0.5)   —— 只影响 r_over 这一项的判据
%     .Yl      车道半宽 (默认 1.75 m = Roadwidth/2)
%     .es      安全余量 (默认 0.2 m = env_es)
%     .xf/.xr  车头/车尾到 CarSim 车辆原点(前轴中心)的距离 (默认 0.860 / 3.972 m)
%     .Wv      车身宽 (默认 1.96 m)
%     .lf      质心到前轴 (默认 1.2466 m; 仅 ERD 缺 Xcg_TM/Ycg_TM 时用)
%     .tail    尾段起点 (默认 6.8 s)
%     .wp      参考路径 .mat (默认 'WayPoints_Type1.mat')
%
%   车道口径 (2026-09-26 起为车身角点, 与控制器的四角点约束一致):
%     corner_pk  四个角点到参考路径的最大法向距离 (m)
%     mgn_pct    任一角点进入安全余量 (|d| > Yl - es) 的时间占比 (%)
%     out_pct    任一角点出车道 (|d| > Yl) 的时间占比 (%)
%   角点位置以 CarSim 的 Xo/Yo 为基准 —— 它是**前轴中心**(实测质心在其后 1.245 m = lf),
%   不是质心; 尺寸取自 E_Class_SUV 外形 body_trim.obj 的包围盒(原点同在前轴)。
%   距离用到折线的有符号法向距离(左正), 不用 Y 向差 —— 换道段航向约 8 deg, 后者偏大约 1%。
%   ey_rms / ey_pk: 质心(CarSim Xcg_TM/Ycg_TM)到参考路径的法向距离 —— 论文 e_y 的定义,
%   2026-09-26 起控制器也在质心处量 e_y。旧口径(车辆原点 = 前轴中心的 Y 向差)保留为
%   eyo_rms / eyo_pk, 只供与旧记录对照。旧的"质心 |ey| > 0.5325 m 算越界"已废弃。
if nargin < 2, opt = struct(); end
if ischar(tag) || isstring(tag), D = func_ReadERD(char(tag)); else, D = tag; end
%  路径与 mu 默认按 ERD 记录的数据集名解析(DLC80_mu0.5_* -> DLC 路径, mu = 0.5, 与旧默认相同)
dsn = '';  if isfield(D,'Dataset'), dsn = D.Dataset; end
MV  = func_ManeuverFromName(dsn);
if ~isfield(opt,'mu'),   opt.mu   = MV.mu;    end
if ~isfield(opt,'Yl'),   opt.Yl   = 1.75;     end
if ~isfield(opt,'es'),   opt.es   = 0.2;      end
if ~isfield(opt,'xf'),   opt.xf   = 0.860;    end
if ~isfield(opt,'xr'),   opt.xr   = 3.972;    end
if ~isfield(opt,'Wv'),   opt.Wv   = 1.96;     end
if ~isfield(opt,'lf'),   opt.lf   = 1.2466;   end
if ~isfield(opt,'tail'), opt.tail = 6.8;      end
if ~isfield(opt,'wp'),   opt.wp   = MV.wp;    end
G = 9.80665;

W  = load(opt.wp); W = W.WayPoints_Collect;
if all(diff(W(:,2)) > 0)                                             % 旧口径只对 X 单调的路径(DLC)有定义
    eyo = D.Yo - interp1(W(:,2), W(:,3), D.Xo, 'linear', 'extrap');  % 前轴中心 Y 向差
else
    eyo = nan(size(D.Yo));                                           % J-turn 转过 90 度后 X 不单调
end
ps = deg2rad(D.Yaw);
if isfield(D,'Xcg_TM') && isfield(D,'Ycg_TM')
    Xg = D.Xcg_TM;  Yg = D.Ycg_TM;
else                                                  % 老 ERD 没有质心通道: 由 lf 换算
    Xg = D.Xo - opt.lf*cos(ps);  Yg = D.Yo - opt.lf*sin(ps);
end
ey = sdist_(W(:,2:3), Xg, Yg);

%  车身四角点 [左前 右前 左后 右后] 到参考路径的法向距离
cx = [opt.xf, opt.xf, -opt.xr, -opt.xr];
cy = opt.Wv/2 * [1, -1, 1, -1];
dc = zeros(numel(D.t), 4);
for k = 1:4
    Xc = D.Xo + cx(k)*cos(ps) - cy(k)*sin(ps);
    Yc = D.Yo + cx(k)*sin(ps) + cy(k)*cos(ps);
    dc(:,k) = sdist_(W(:,2:3), Xc, Yc);
end
dmax = max(abs(dc), [], 2);

r    = deg2rad(D.AVz);                       % rad/s
vx   = D.Vx/3.6;                             % m/s
rb   = opt.mu*G ./ max(vx,1);                % Zeng 界 mu*g/Vx
dt   = D.t(2) - D.t(1);
tl   = D.t > opt.tail;
dif  = (D.My_Bk_L1 + D.My_Bk_L2) - (D.My_Bk_R1 + D.My_Bk_R2);
Tb   = [D.My_Bk_L1 D.My_Bk_R1 D.My_Bk_L2 D.My_Bk_R2];
Fd   = [D.Fd_L1 D.Fd_R1 D.Fd_L2 D.Fd_R2];

M.ey_rms    = rms_(ey);                                  % m   质心, 法向距离
M.ey_pk     = max(abs(ey));                              % m
M.eyo_rms   = rms_(eyo);                                 % m   旧口径(前轴中心 Y 向差)
M.eyo_pk    = max(abs(eyo));                             % m
M.corner_pk = max(dmax);                                 % m   车身角点
M.mgn_pct   = 100*mean(dmax > opt.Yl - opt.es);          % %   进入安全余量
M.out_pct   = 100*mean(dmax > opt.Yl);                   % %   出车道
M.ltr_pk   = max(abs(D.LTR));                            % -
M.roll_pk  = max(abs(D.Roll));                           % deg
M.r_over   = 100*mean(abs(r) > rb);                      % %  |r| > mu*g/Vx
M.ay_pk    = max(abs(D.Ay));                             % g   真实侧向加速度
M.vr_pk    = max(abs(vx.*r))/G;                          % g   稳态近似 Vx*r
M.sw_tail  = tailmax_(D.Steer_SW, tl);                   % deg
M.dsw_rms  = rms_(diff(D.Steer_SW))/dt;                  % deg/s
M.dTb_rms  = mean(rms_(diff(Tb))/dt);                    % N*m/s  四轮均值
M.dFd_rms  = mean(rms_(diff(Fd))/dt);                    % N/s
M.db_rms   = rms_(dif);                                  % N*m    左右制动力矩差
M.thr_sat  = 100*mean(D.Throttle<0.01 | D.Throttle>0.99);% %
M.dthr_rms = rms_(diff(D.Throttle))/dt;                  % 1/s
M.vx_mean  = mean(D.Vx);                                 % km/h
M.N        = D.N;
M.T_end    = D.t(end);
end

function d = sdist_(P, xq, yq)
%SDIST_  点到折线 P (n x 2) 的有符号法向距离, 左正(与控制器 e_y 同号)
T  = diff(P);
L2 = sum(T.^2, 2);
d  = zeros(size(xq));
for i = 1:numel(xq)
    s = ((xq(i) - P(1:end-1,1)).*T(:,1) + (yq(i) - P(1:end-1,2)).*T(:,2)) ./ L2;
    s = min(max(s, 0), 1);
    cxp = P(1:end-1,1) + s.*T(:,1);
    cyp = P(1:end-1,2) + s.*T(:,2);
    [~, j] = min((xq(i) - cxp).^2 + (yq(i) - cyp).^2);
    d(i) = (T(j,1)*(yq(i) - cyp(j)) - T(j,2)*(xq(i) - cxp(j))) / sqrt(L2(j));
end
end

function y = rms_(x)
if isvector(x), y = sqrt(mean(x(:).^2));
else,           y = sqrt(mean(x.^2, 1));
end
end

function y = tailmax_(x, m)
if any(m), y = max(abs(x(m))); else, y = NaN; end
end
