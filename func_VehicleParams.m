function [Pa]=func_VehicleParams(cargo)
    Pa = struct();
    %  ============ 载荷工况 ============
    %  空载 = DLC;  载荷 = 鱼钩(顶置货箱)。两套参数不能混用。
    %  数值全部取自 CarSim echo 文件的 "! CALC --" 只读计算量, 不要手推:
    %    M_TL / LX_CG_TL / H_CG_TL / IZZ_TL  整车(total laden)
    %    M_SL / H_CG_SL / IXX_SL             簧载(sprung laden)
    %  在 base workspace 设 PMPC_CARGO = 170 (kg) 切到载荷工况; 缺省空载。
    %  cargo 由调用方传入(cq32019mpc 用 wsget 从 CarSim 数据集读)。
    %  不传时退回 base workspace, 供 chk_* 等脚本单独调用。
    if nargin < 1 || isempty(cargo)
        cargo = 0;
        if evalin('base', 'exist(''PMPC_CARGO'',''var'')')
            cargo = evalin('base', 'PMPC_CARGO');
        end
    end

    if cargo > 0
        % ---- 载 170 kg 顶置货箱 (鱼钩) ----
        Pa.m    = 2030;        % M_TL
        Pa.ms   = 1760;        % M_SL
        Pa.Iz   = 3742.62;     % IZZ_TL
        Pa.Ix   = 1403.80;     % IXX_SL (簧载侧倾惯量)
        Pa.h_TL = 0.826076;    % H_CG_TL
        Pa.h_SL = 0.891932;    % H_CG_SL
        Pa.lf   = 1.351444;    % LX_CG_TL
    else
        % ---- 空载 (DLC) ----
        Pa.m    = 1860;        % M_TL
        Pa.ms   = 1590;        % M_SU
        Pa.Iz   = 3438.54;     % IZZ_TL  (原填 3315.9 是手算值, 偏低 3.6%)
        Pa.Ix   = 894.4;       % IXX_SU
        Pa.h_TL = 0.671694;    % H_CG_TL
        Pa.h_SL = 0.720;       % H_CG_SU
        Pa.lf   = 1.246613;    % LX_CG_TL
    end
    Pa.L    = 2.95;                              % 轴距(m)
    Pa.lr   = Pa.L - Pa.lf;                     % 质心至后轴距离(m)
    Pa.BodyLength = 5;%车长
    Pa.BodyWigdth = 2;%车宽
    Pa.Bodydiagonal=sqrt(Pa.BodyLength^2+Pa.BodyWigdth^2);
    % Pa.hf   = -0.053;                           % Roll Center below Axle1
    % Pa.hr   = 0.195;                            % Roll Center above Axle2
    % Pa.hwf  = 0.51;                             % 前轴轮胎中心高度
    % Pa.hwr  = 0.53;                             % 后轴轮胎中心高度
    % Pa.h1   = Pa.hwf + Pa.hf;
    % Pa.h2   = Pa.hwr + Pa.hr;
    % Pa.h_RC = (Pa.h2 - Pa.h1)*Pa.lf/Pa.L+Pa.h1; %侧倾中心高度0.6
    Pa.h_RC  = 0.15;                 % 侧倾中心离地高(悬架几何, 不随载荷变)
    Pa.h_S2R = Pa.h_SL - Pa.h_RC;    % 簧载质心到侧倾中心 (空载0.570 / 载荷0.742)
    Pa.tf =1.575;                               % L_TRACK(1)
    Pa.tr = 1.575;                              % L_TRACK(2)
    Pa.lsf = 1.2;                                 % L_SPRINGS(1)
    Pa.lsr = 1.2;                                 % L_SPRINGS(2)
    Pa.ldf = 1.1;                                 % L_DAMPERS(1)
    Pa.ldr = 1.1;                                 % L_DAMPERS(2)
    %轮胎参数
    Pa.If = 1.1;            % 前轮转动惯量(kg·m²)
    Pa.Ir = 1.1;            % 后轮转动惯量(kg·m²)双轮
    Pa.rt = 0.393;        % 车轮有效半径(m) 原版车 265/75R16, RRE=393mm (Wang2023 R_e=0.393)
    %转向系
    Pa.isw = 18.57;      % 方向盘/前轮 传动比。出处: CarSim 转向齿条表 RACK_KIN (E-Class SUV)
                         % 中位斜率 x RACK_TRAVEL(46.06 mm/rev); 随转角 17.6~19.4, 取中位 18.57。
                         % 原写 17 无出处, 使被控对象实际少转 8.5%、且高估自身转角速率(改进.md 18z)。
    %悬架刚度阻尼
    Pa.kf = 146e3;       % 前悬架刚度(N/m)
    Pa.kr = 46e3;       % 后悬架刚度(N/m)
    Pa.cf = 0;       % 前悬架阻尼(N·s/m)
    Pa.cr = 0;       % 后悬架阻尼(N·s/m)
    % Pa.cf = 30e3;       % 前悬架阻尼(N·s/m)
    % Pa.cr = 50e3;       % 后悬架阻尼(N·s/m)
%     Pa.omega = 30;      % 圆频率(rad/s)
%     Pa.kef=(Pa.kw*Pa.cf^2*Pa.omega^2+Pa.kf*Pa.kw*(Pa.kw+Pa.kf))/(Pa.cf^2*Pa.omega^2+(Pa.kf+Pa.kw)^2);
%     Pa.ker=(Pa.kw*Pa.cr^2*Pa.omega^2+Pa.kr*Pa.kw*(Pa.kw+Pa.kr))/(Pa.cr^2*Pa.omega^2+(Pa.kr+Pa.kw)^2);
% %侧倾刚度与阻尼计算
%     Pa.Kf=Pa.kef*Pa.lsf^2;
%     Pa.Kr=Pa.ker*Pa.lsr^2;% 等效侧倾角刚度 (N·m/rad)   
% 
%     Pa.Kt  = Pa.Kf + Pa.Kr;       %等效侧倾刚度
    Pa.Ct  = Pa.ldf^2*Pa.cf + Pa.ldr^2*Pa.cr;

    Pa.kw = 470e3;    % CarSim FZ_TIRE_COEFFICIENT = 470 N/mm
    
    % --- 等效侧倾刚度 K_t ---
    %   簧上质量相对地面的侧倾角 phi (= ParaHAT.Roll) 同时包含悬架变形与轮胎压缩，
    %   故乘 phi 的刚度必须是「悬架(弹簧+防倾杆)」与「轮胎垂向刚度」的串联值。
    %   参数来源: CarSim LastRun_echo.par
    %     FS_COMP_COEFFICIENT      前 146 / 后 46  N/mm      -> Pa.kf / Pa.kr
    %     MX_AUX_COEFFICIENT       前 384 / 后 510  N*m/deg  (防倾杆)
    %     FZ_TIRE_COEFFICIENT      470 N/mm                  -> Pa.kw
    %     L_SPRINGS 1200 mm -> Pa.lsf/lsr ;  L_TRACK 1575 mm -> Pa.tf
    Pa.Kaux_f = 384*180/pi;                             % 前防倾杆  22002 N*m/rad
    Pa.Kaux_r = 510*180/pi;                             % 后防倾杆  29221 N*m/rad
    Kf_susp   = 0.5*Pa.kf*Pa.lsf^2 + Pa.Kaux_f;         % 前轴悬架 127122
    Kr_susp   = 0.5*Pa.kr*Pa.lsr^2 + Pa.Kaux_r;         % 后轴悬架  62341
    Kt_axle   = 0.5*Pa.kw*Pa.tf^2;                      % 单轴轮胎 582947
    Pa.Kt     = 1/(1/Kf_susp + 1/Kt_axle) ...           % 串联 -> 160681 N*m/rad
              + 1/(1/Kr_susp + 1/Kt_axle);
    %   仅悬架(不含轮胎)为 189462；实测侧倾梯度 3.47 deg/g 对应 155843，
    %   本次数据独立辨识 160343 —— 三者一致，故取串联值。

    Pa.Ct  = 0;
    Pa.g = 9.80665;
    Pa.mu  = 0.4;
    Pa.kd  = 10000;
    %% 半主动减振器约束相关参数

    %  MR 阻尼器上下界: 按台架实测重标 (D:\OneDrive...\试验\MR减振器\matlab\MR_all_data.mat)
    %    0A=关断最软 -> Gl,  4A=最大电流最硬 -> Gu
    %  原 b=6.2e2 使两段在 v0 处不连续: k1*v0=2800 N 而 k2*v0+b=866 N, 力突降 69%,
    %  且 v>0.2 m/s 段整体低估约一半 (v=0.5 时模型 1235 N, 实测 2571 N)。
    %  保持 k1/k2 不变, 仅由连续性重定 v0 与 b: b=(k1-k2)*v0。
    %  拟合 RMSE 273 N, 与四系数全拟合(272 N)等效。
    Pa.k1 = 14e3;   % 上界段1斜率
    Pa.k2 = 1.23e3;   % 上界段2斜率
    Pa.b  = 1916;     % 上界段2偏置 (原 6.2e2, 会造成 v0 处不连续)
    Pa.k3 = 8.3e2;    % 下界斜率; 实测过原点最小二乘 929, 现值偏低 11%
    Pa.v0 = 0.15;    % 速度分段点 (原 0.2)
%% ---- codegen: 下面 4 个在运行中每拍被改写(在线切线刚度),
%  原来是动态添加的。代码生成不允许, 故在此预声明。
%  首拍走 InitialGapflag 早退分支, 不会在写之前读, 初值取 0 安全。
Pa.CbarF  = 0;
Pa.CbarR  = 0;
Pa.CafTan = 0;
Pa.CarTan = 0;

end
