function [A, B] = func_TireTable(mode, varargin)
%FUNC_TIRETABLE  轴级侧偏特性 (alpha, Fz) -> [f_bar, c_bar]，Fiala 解析式
% -------------------------------------------------------------------------
% 仿射线性化（Ataei et al., 2020, Eq.1）：
%       f_y(alpha) ~= f_bar + c_bar * (alpha - alpha_bar)
%                   = c_bar * alpha + F_off ,  F_off = f_bar - c_bar*alpha_bar
%
% 关键：f_bar 与 c_bar 必须来自 **同一个** 轮胎模型。
%   之前把 CarSim 实测力和模型切线刚度混用，偏置项吸收了巨大的模型失配，
%   导致 Delta-delta 方案发散。这里两者都由 Fiala 给出，
%   c_bar 用解析导数（不是差分、也不是在线估计）。
%
% 用法:
%   T            = func_TireTable('carsim', mu_road, nWheel)   % 标定(由 CarSim carpet)
%   T            = func_TireTable('comply', T, k_cs)           % 叠加转向柔性
%   [fb, cb]     = func_TireTable('eval',  T, alpha, Fz)       % 解析求值(codegen)
%
% 约定: alpha [rad]，Fz [N]（轴级），f_y 与 alpha 反号（阻碍侧滑）
% -------------------------------------------------------------------------

switch lower(mode)

    case 'carsim'
        % T = func_TireTable('carsim', mu_road, nWheel)
        %  2026-09-27: 由"二维查表"改为 **按载荷标定的 Fiala 解析式** (改进.md 18z)。
        %  不再存 141x70 的 Fbar/Cbar 两张网格, 只存两条二次多项式的系数;
        %  'eval' 直接算解析式与解析导数, 线性区与饱和区都是闭式。
        %
        %  标定(闭式, 不需优化器): 对 carpet 的每个载荷列, 取
        %      muFz = 实测峰值力,   C = 3*muFz/tan(实测峰值侧偏角)
        %  即让 Fiala 的峰值力与饱和角都等于实测值。对 CarSim Tire Tester 实测曲线
        %  (0-7 deg, 车辆实际 alpha_f<=6.1 alpha_r<=4.6) 的误差: RMSE 14-56 N/轮,
        %  最大 1.6-3.0% 峰值。旧注释"任何 (C0,mu) 的 Fiala 只给 55~64e3"是用**单一**
        %  (C0,mu) 套全部载荷所致; 按载荷标定后 3 deg 轴级切线 74735 vs 实测 77737 (-3.9%)。
        %
        %  mu 的处理(2026-09-27 Tire Tester 实测, 20 个算例):
        %    侧偏刚度与 mu 无关(mu=0.3..1.0 极差 1.0%); 峰值力 = mu*mu_ref(Fz)*Fz,
        %    峰值侧偏角 = mu*alpha_pk,ref —— 二者随 mu 精确线性。Fiala 天然满足这三条:
        %    C 不含 mu, 峰值 = muFz, alpha_sl = atan(3*muFz/C) 随 mu 近似线性。
        %  carpet 出自 CarSim "Tire: Lateral Force / 265-75 R16 / Touring Tires"
        %  (8 载荷 x 51 侧偏角, MU_REF_Y=1.0, 单胎)。峰值 mu_y 随载荷 0.876 -> 0.705,
        %  这正是"有效附着低于路面 mu"的物理来源, 已含在 mu_ref(Fz) 里。
        %  第 3 个参数 = 1: C 改取**实测原点刚度**(而非由峰值角反推)。两种标定都只用可直接
        %  量的量: 前者保证小角刚度正确(形状误差 5.3-6.6%), 后者保证饱和角正确(1.6-3.0%)。
        mu_road = varargin{1};
        if numel(varargin) >= 2, nW = varargin{2}; else, nW = 2; end
        if numel(varargin) >= 3 && varargin{3}, use_c0 = true; else, use_c0 = false; end
        C = load('TireCarpet_265_75R16.mat');
        alc = C.AL(:);  fzc = C.FZ(:);  FYc = C.FY;      % (nAlpha x nFz), deg / N, 单胎
        nz  = numel(fzc);
        cn_w = zeros(nz,1);   mn_w = zeros(nz,1);        % 归一化: C/Fz, muFz/Fz (单胎)
        for j = 1:nz
            [pk, ip] = max(FYc(:,j));
            mn_w(j) = pk / fzc(j);
            if use_c0
                cn_w(j) = FYc(1,j) / deg2rad(alc(1)) / fzc(j);      % 原点割线(表首点 0.5 deg)
            else
                cn_w(j) = 3*pk / tan(deg2rad(alc(ip))) / fzc(j);    % 使饱和角 = 实测峰值角
            end
        end
        %  轴级: Fz_axle = nW*Fz_wheel, 力乘 nW  =>  归一化量(除以各自 Fz)不变。
        %  再按路面 mu 缩放峰值(刚度不缩放 —— 实测结论)。
        fza = nW*fzc;
        T.src   = 'Fiala fit to CarSim FY_TIRE_CARPET 265/75 R16';
        T.mu    = mu_road;   T.nW = nW;
        T.cq    = polyfit(fza, cn_w,                     2);   % C(Fz)    = Fz * polyval(cq, Fz)
        T.mq    = polyfit(fza, mn_w*mu_road/C.MU_REF,    2);   % muFz(Fz) = Fz * polyval(mq, Fz)
        T.fzlo  = fza(1);    T.fzhi = fza(end);                % 多项式自变量钳位范围
        T.kcs   = 0;                                           % 转向柔性(轴级 rad/N), 见 'comply'
        %  切线刚度下限用的名义值: 取工作载荷范围内的最大 |C|
        fzs     = linspace(T.fzlo, T.fzhi, 200).';
        T.C0    = max(fzs .* polyval(T.cq, fzs));
        A = T;  B = [];

    case 'comply'
        % T = func_TireTable('comply', T, k_cs)
        %  转向柔性: 实际车轮转角 = 运动学转角 - k_cs*Fy (Fy 为轴级侧向力, 与 alpha 反号),
        %  故 alpha_kin = alpha - k_cs*Fbar(alpha)。以运动学侧偏角为自变量重采样,
        %  得到前轴有效特性, 小角刚度 C/(1 + k_cs*C) (经典柔性转向结论)。
        %  Fiala 下柔性只改刚度: C_eff = C/(1 + k_cs*C), 峰值力不变(饱和力与转角无关),
        %  故只存 k_cs, 在 'eval' 里作用于 C。
        T = varargin{1};  T.kcs = varargin{2};
        fzs  = linspace(T.fzlo, T.fzhi, 200).';
        Cs   = fzs .* polyval(T.cq, fzs);
        T.C0 = max(Cs ./ (1 + T.kcs*Cs));
        A = T;  B = [];

    case 'eval'
        % [f_bar, c_bar] = func_TireTable('eval', T, alpha, Fz)   —— 标量, 供 codegen
        %  Fiala 解析式 + 解析导数, 无查表。C 与 muFz 由二次多项式给出:
        %    C(Fz) = Fz*polyval(cq, z),  muFz(Fz) = Fz*polyval(mq, z),  z = clamp(Fz)
        %  自变量 z 钳位而乘子用真实 Fz => 表下界以下自动变成"过原点线性"(Fz->0 则力->0),
        %  与原查表版的外推约定一致。
        T = varargin{1};  alpha = varargin{2};  Fz = varargin{3};
        z    = min(max(Fz, T.fzlo), T.fzhi);
        Cst  = max(Fz, 0) * (T.cq(1)*z*z + T.cq(2)*z + T.cq(3));
        muFz = max(Fz, 0) * (T.mq(1)*z*z + T.mq(2)*z + T.mq(3));
        Cst  = Cst / (1 + T.kcs*Cst);                    % 转向柔性(k_cs = 0 时无作用)
        Cst  = max(Cst, 1.0);   muFz = max(muFz, 1.0);   % 防除零
        t    = tan(min(max(alpha, -1.4), 1.4));
        tsl  = 3*muFz/Cst;
        if abs(t) >= tsl
            A = -muFz*sign(t);
            B = 0;
        else
            A = -Cst*t + (Cst*Cst/(3*muFz))*abs(t)*t - (Cst^3/(27*muFz*muFz))*t^3;
            B = (-Cst + (2*Cst*Cst/(3*muFz))*abs(t) - (Cst^3/(9*muFz*muFz))*t*t) * (1 + t*t);
        end
        if ~isfinite(A), A = 0;      end
        if ~isfinite(B), B = -T.C0;  end

    otherwise
        error('func_TireTable: mode must be carsim / comply / eval');
end
end
