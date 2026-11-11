%% init_valve_avg.m  --  CDC 电磁阀电流环【平均值模型】参数初始化
% 仅用于平均值模型（无 PWM / MOSFET / 二极管，定步长 = 控制周期）。
% 开关模型请用 init_valve.m，两个文件互不依赖。
% 由模型的 PreLoadFcn / InitFcn 调用，变量进入基础工作区。
% PI 不加前馈、不加抗饱和（PI 块取消勾选"限制输出"）；
% 占空比钳位 [0,1] 由 PI 后面的 Saturation 块完成。
%
% 模型块对照：
%   Discrete State-Space : A = a_pl, B = b_pl, C = 1, D = 0, 初始条件 = Iref_init, 采样时间 = Ts_avg
%   Discrete FIR Filter  : 系数 [0.5 0.5], 初始状态 = Iref_init, 采样时间 = Ts_avg
%   1/z                  : 初始条件 = Iref_init
%   PI                   : 采样时间 = Tc, P = Kp, I = Ki, 积分器初值 = D_init_avg, 不限幅
%   Saturation           : 上限 1, 下限 0
%   求解器               : 定步长 = Ts_avg, 停止时间 = Tstop

%% ========== 1. 用户可调参数 ==========
fs_ctrl    = 1000;    % 控制频率 [Hz]，必须等于模型基础步长的倒数（步长 0.001 s → 1000）
bw_ratio   = 30;      % 电流环带宽 = 控制频率 / bw_ratio（越小越快，裕度越低）
Ti_ratio   = 2.4;     % 积分时间 Ti = Ti_ratio*L/R（越大超调越小、越慢；1 为零极点对消）

Iref_init  = 0;       % 参考电流初值 [A]
Iref_final = 0;       % 参考电流终值 [A]
Iref_tstep = 0.2;     % 参考电流阶跃时刻 [s]
Imax_work  = 2;       % 最大工作电流 [A]（只用于检查占空比余量）

verbose    = false;   % true：每次仿真启动时打印参数摘要

%% ========== 2. 对象参数 ==========
Vdc = 24;             % 电源电压 [V]
R   = 3;              % 线圈电阻 [Ohm]
L   = 2;              % 线圈电感 [mH]（与开关模型保持一致）
L_H = L*1e-3;         % 线圈电感 [H]（仅用于下面的计算）

%% ========== 3. 时序参数 ==========
Tc     = 1/fs_ctrl;   % 控制周期 [s]
Ts_avg = Tc;          % 平均值模型的定步长 [s]

%% ========== 4. 对象离散化 ==========
% 一阶 RL 对象按控制周期 Tc 精确离散化：  i[k+1] = a_pl*i[k] + b_pl*u[k]
tau_e = L_H/R;                    % 电气时间常数 [s]
a_pl  = exp(-Tc/tau_e);
b_pl  = (Vdc/R)*(1 - a_pl);

%% ========== 5. PI 电流环参数 ==========
% 带宽由 bw_ratio 决定，积分时间 Ti = Ti_ratio*L/R：
%   Ki = wc*R/Vdc,  Kp = Ti_ratio*wc*L_H/Vdc   （并联形式，Ki 单位 1/s）
% Ti_ratio = 1（零极点对消）没有计入约 2.5 拍的总滞后，阶跃超调 15% 以上，
% 所以取 Ti_ratio > 1，用较大的 Kp 换取无超调。
fc = fs_ctrl/bw_ratio;            % 目标带宽 [Hz]
wc = 2*pi*fc;                     % [rad/s]
Kp = Ti_ratio*wc*L_H/Vdc;
Ki = wc*R/Vdc;

%% ========== 6. 初始状态（让仿真 0 时刻就处于 Iref_init 的稳态）==========
D_init_avg = R*Iref_init/Vdc;     % Iref_init 对应的稳态占空比（模型不含二极管压降）

%% ========== 7. 估算与检查 ==========
D_max_avg = R*Imax_work/Vdc;                  % 最大工作电流所需占空比
% 相角裕度：按离散环路 PI * 对象 * FIR[0.5 0.5] * 1/z 的频率响应直接计算
w_chk  = logspace(0, log10(pi/Tc*0.999), 4000);
z_chk  = exp(1j*w_chk*Tc);
L_chk  = (Kp + Ki*Tc./(z_chk - 1)) .* (b_pl./(z_chk - a_pl)) ...
         .* (0.5*(1 + 1./z_chk)) ./ z_chk;
[~, i_chk] = min(abs(abs(L_chk) - 1));        % 幅值穿越 1 的位置
PM_est = 180 + angle(L_chk(i_chk))*180/pi;    % 相角裕度估算 [deg]

if D_max_avg > 0.8
    warning('init_valve_avg:Saturation', ...
        '最大工作电流所需占空比约 %.0f%%，PI 输出可能饱和，建议加回抗饱和。', D_max_avg*100);
end
if PM_est < 45
    warning('init_valve_avg:PM', ...
        '估算相角裕度仅 %.0f deg，建议增大 bw_ratio 或提高控制频率。', PM_est);
end

if verbose
    fprintf('--- init_valve_avg ---\n');
    fprintf('控制 %.0f Hz, 步长 %.1e s\n', fs_ctrl, Ts_avg);
    fprintf('tau = %.3f ms, a_pl = %.4f, b_pl = %.4f\n', tau_e*1e3, a_pl, b_pl);
    fprintf('带宽 %.0f Hz, Ti_ratio = %.1f, Kp = %.4f, Ki = %.2f, 相角裕度估算 %.0f deg\n', ...
        fc, Ti_ratio, Kp, Ki, PM_est);
    fprintf('所需占空比：最大工作电流 %.1f%%\n', D_max_avg*100);
end

clear w_chk z_chk L_chk i_chk;
