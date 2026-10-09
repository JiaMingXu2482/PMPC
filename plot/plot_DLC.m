% plot_DLC.m — DLC 工况三幅图：车辆轨迹 / beta-r 相平面 / 纵向车速
% 数据包：COM_plot_package_4fb1637_R70_20261009
% 用法：把 root 改成数据包解压路径后直接运行
%
% 输出：fig_dlc_trajectory.png, fig_dlc_betar.png, fig_dlc_vx.png (300dpi)

clear; close all; clc;

%% ---------- 路径与分段 ----------
root = 'COM_plot_package_4fb1637_R70_20261009';   % <-- 改成你的路径
ctrls = {'MPC','ZENG','PMPC'};
colors = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.4660 0.6740 0.1880]}; % 蓝/红/绿
DLC_S0 = 28.756; DLC_S1 = 200.645;   % DLC 参考里程区间 (m)

% 参考路径
R = load(fullfile(root,'COM_reference.mat'));
W = R.W;                       % 981x9, W(:,2)=X, W(:,3)=Y
refX = W(:,2); refY = W(:,3);

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
plot(refX, refY, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
for i = 1:nC
    plot(dat{i}.Xo, dat{i}.Yo, 'Color', colors{i}, 'LineWidth', 1.6, ...
        'DisplayName', ctrls{i});
end
hold off;
xlabel('X (m)'); ylabel('Y (m)');
legend('Location','best'); grid on; axis equal;
set(gca,'FontName','Times New Roman','FontSize',11);
print(fh, 'fig_dlc_trajectory.png','-dpng','-r300');

%% ---------- 图2：beta-r 相平面 ----------
fh = figure('Color','w','Position',[100 100 560 480]);
hold on;
for i = 1:nC
    plot(dat{i}.t.beta_deg, dat{i}.t.yaw_rate_deg_s, ...
        'Color', colors{i}, 'LineWidth', 1.4, 'DisplayName', ctrls{i});
end
hold off;
xlabel('\beta (deg)'); ylabel('r (deg/s)');
legend('Location','best'); grid on;
set(gca,'FontName','Times New Roman','FontSize',11);
print(fh, 'fig_dlc_betar.png','-dpng','-r300');

%% ---------- 图3：纵向车速 ----------
fh = figure('Color','w','Position',[100 100 720 420]);
hold on;
for i = 1:nC
    plot(dat{i}.t.station_m, dat{i}.t.vx_kmh, ...
        'Color', colors{i}, 'LineWidth', 1.6, 'DisplayName', ctrls{i});
end
hold off;
xlabel('Reference station (m)'); ylabel('V_x (km/h)');
xlim([DLC_S0 DLC_S1]);
legend('Location','best'); grid on;
set(gca,'FontName','Times New Roman','FontSize',11);
print(fh, 'fig_dlc_vx.png','-dpng','-r300');

disp('Done: fig_dlc_trajectory.png, fig_dlc_betar.png, fig_dlc_vx.png');
