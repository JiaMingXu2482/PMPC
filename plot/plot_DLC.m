% plot_DLC.m — DLC 工况三幅图：车辆轨迹 / beta-r 相平面 / 纵向车速
% 数据包：COM_plot_package_4fb1637_R70_20261009
% 用法：把 root 改成数据包解压路径后直接运行
%
% 输出：三幅 figure 窗口（不自动存图，需要时手动另存）

clear; close all; clc;

%% ---------- 路径与分段 ----------
% 数据路径：优先读 plot_local.m（本地配置，不进 git）；否则用脚本同目录下的相对路径
if exist('plot_local.m','file') == 2
    root = plot_local();
else
    root = fullfile(fileparts(mfilename('fullpath')), 'COM_plot_package_4fb1637_R70_20261009');
end
ctrls = {'mpc_mil','nodelay_mil','pmpc_mil'};
% 图例显示名：Controller A=baseline MPC, Controller B=PMPC无时延补偿(消融), Proposed=完整PMPC
disp_names = {'Controller A','Controller B','Proposed'};
colors = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.4660 0.6740 0.1880]}; % 蓝/红/绿
DLC_S0 = 28.756; DLC_S1 = 200.645;   % DLC 参考里程区间 (m)
X_OFF = 30;   % 横坐标统一平移：X-30，DLC 起点从 ~0 开始

% 参考路径
R = load(fullfile(root,'COM_reference.mat'));
W = R.W;                       % 981x9, W(:,2)=X, W(:,3)=Y, W(:,7)=参考里程
refS = W(:,7);
refIdx = refS >= DLC_S0 & refS <= DLC_S1;
refX = W(refIdx,2); refY = W(refIdx,3);

%% ---------- 读三组数据 ----------
nC = numel(ctrls);
dat = cell(1,nC);
for i = 1:nC
    t = readtable(fullfile(root, ctrls{i}, 'vehicle_signals.csv'));
    % DLC 段索引
    idx = t.station_m >= DLC_S0 & t.station_m <= DLC_S1;
    t = t(idx,:);
    % 实际轨迹 Xo/Yo（v7.3 mat，MATLAB 原生可读）
    S = load(fullfile(root, ctrls{i}, 'vehicle_signals.mat'), 'D');
    Xo = squeeze(S.D.Xo); Yo = squeeze(S.D.Yo);
    % D 与 CSV 同采样(0.01s,2000点)，取同样 idx；若长度不一致则按时间对齐
    if numel(Xo) == 2000
        full_idx = (1:2000)';
        csv_all = readtable(fullfile(root, ctrls{i}, 'vehicle_signals.csv'));
        idx_full = csv_all.station_m >= DLC_S0 & csv_all.station_m <= DLC_S1;
        Xo = Xo(idx_full); Yo = Yo(idx_full);
    end
    dat{i}.t = t; dat{i}.Xo = Xo; dat{i}.Yo = Yo;
end

%% ---------- 图1：车辆轨迹 X-Y ----------
fh = figure('Color','w','Position',[100 100 720 520]);
hold on;
plot(refX - X_OFF, refY, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
for i = 1:nC
    plot(dat{i}.Xo - X_OFF, dat{i}.Yo, 'Color', colors{i}, 'LineWidth', 1.6, ...
        'DisplayName', disp_names{i});
end
hold off;
xlabel('X (m)'); ylabel('Y (m)');
legend('Location','best'); grid on;
set(gca,'DataAspectRatioMode','auto','PlotBoxAspectRatioMode','auto');  % 拖动可自由改变比例
set(gca,'FontName','Times New Roman','FontSize',14,'FontWeight','bold','GridColor',[0.65,0.65,0.65],'LineWidth',1,'Position',[0.0853 0.2201 0.8542 0.6269]); box on;

%% ---------- 图2：beta-r 相平面 ----------
% 稳定包络（参考 Zeng et al. 2025, IEEE TTE, Eq.27 + Fig.9）：
%   -alpha_r_sat + (lr/vx)*r ≤ β ≤ alpha_r_sat + (lr/vx)*r   (后轴侧偏饱和约束，平行四边形)
%   |r| ≤ μg/vx                                              (横摆角速度极限，Eq.12)
%   alpha_r_sat = atan(μ*m*g*lf/(Cαr*L))，见 func_Envelope.m
MU = 0.5; G = 9.81;                       % DLC 段路面附着
vx_ms = mean(dat{1}.t.vx_kmh)/3.6;        % 平均车速 (m/s)
r_lim = MU*G/vx_ms;                       % 横摆角速度极限 (rad/s)
% 整车参数（DLC 空载，见 func_VehicleParams.m）
MV_m = 1860; MV_lf = 1.2466; MV_lr = 1.7034; MV_L = 2.95;
Fzr_st = MV_m*G*MV_lf/MV_L;               % 后轴静载 (N)
C_ar = 62923;                             % 后轴侧偏刚度 (N/rad)，μ=0.5 估计值（CarHat 按 μ 折算）
a_sat = atan(MU*Fzr_st/C_ar);             % 后轴饱和侧偏角 (rad)
% 平行四边形顶点：(β,r)，β 为 x 轴，r 为 y 轴
rv = [-r_lim, r_lim];                     % r 顶点 (rad/s)
b_lo = -a_sat + (MV_lr/vx_ms)*rv;         % 下斜边 β (rad)
b_up =  a_sat + (MV_lr/vx_ms)*rv;         % 上斜边 β (rad)
R2D = 180/pi;
bx = [b_lo(1), b_lo(2), b_up(2), b_up(1), b_lo(1)]*R2D;
by = [rv(1), rv(2), rv(2), rv(1), rv(1)]*R2D;
fh = figure('Color','w','Position',[100 100 560 480]);
hold on;
for i = 1:nC
    plot(dat{i}.t.beta_deg, dat{i}.t.yaw_rate_deg_s, ...
        'Color', colors{i}, 'LineWidth', 1.4, 'DisplayName', disp_names{i});
end
% 稳定包络平行四边形（黑虚线，对标论文 Fig.9）
plot(bx, by, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Stability envelope');
hold off;
xlabel('\beta (deg)'); ylabel('r (deg/s)');
legend('Location','best'); grid on;
set(gca,'FontName','Times New Roman','FontSize',14,'FontWeight','bold','GridColor',[0.65,0.65,0.65],'LineWidth',1,'Position',[0.0853 0.2201 0.8542 0.6269]); box on;

%% ---------- 图3：纵向车速 ----------
fh = figure('Color','w','Position',[100 100 720 420]);
hold on;
for i = 1:nC
    plot(dat{i}.t.station_m - X_OFF, dat{i}.t.vx_kmh, ...
        'Color', colors{i}, 'LineWidth', 1.6, 'DisplayName', disp_names{i});
end
hold off;
xlabel('Reference station (m)'); ylabel('V_x (km/h)');
xlim([DLC_S0 DLC_S1] - X_OFF);
legend('Location','best'); grid on;
set(gca,'FontName','Times New Roman','FontSize',14,'FontWeight','bold','GridColor',[0.65,0.65,0.65],'LineWidth',1,'Position',[0.0853 0.2201 0.8542 0.6269]); box on;

disp('Done: 3 figures displayed.');
