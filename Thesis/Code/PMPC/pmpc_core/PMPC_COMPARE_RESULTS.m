%% PMPC_COMPARE_RESULTS.m
% 从 PMPC_Results 中选择多个仿真 MAT 文件，
% 对比不同预测模型步长 Ts 和减振器时间常数 tau_d 下的结果。
%
% 使用方法：
% 1. 在 MATLAB 命令行运行本脚本。
% 2. 按住 Ctrl 多选需要比较的 PMPC_tau_*.mat 文件。
% 3. 点击“打开”，脚本自动绘图、计算指标并导出 CSV。
%
% 注意：本脚本要求 MAT 文件同时保存 Ts 和 tau_d。
% 旧版 PMPC_SAVE_RESULT.m 未保存 Ts，旧 MAT 文件需要重新仿真保存。

clc;

%% 1. 选择多个仿真结果文件
[files, result_dir] = uigetfile( ...
    fullfile(pwd, 'PMPC_Results', 'PMPC_tau_*.mat'), ...
    '选择需要比较的 PMPC 仿真结果（可按住 Ctrl 多选）', ...
    'MultiSelect', 'on');

if isequal(files, 0)
    fprintf('已取消文件选择。\n');
    return;
end

if ischar(files)
    files = {files};
end

if numel(files) < 2
    error('至少选择两个 MAT 文件，才能进行参数对比。');
end

%% 2. 读取并检查所有结果
nRuns = numel(files);
runs = struct( ...
    'file', cell(1, nRuns), ...
    'Ts', cell(1, nRuns), ...
    'Ts_exec', cell(1, nRuns), ...
    'tau_d', cell(1, nRuns), ...
    'time_s', cell(1, nRuns), ...
    'data', cell(1, nRuns));

for k = 1:nRuns
    file_path = fullfile(result_dir, files{k});
    S = load(file_path);

    required_fields = {'Ts', 'tau_d', 'time_s', 'data'};
    for j = 1:numel(required_fields)
        if ~isfield(S, required_fields{j})
            error(['文件 %s 缺少变量 %s。该文件可能由旧版保存脚本生成，' ...
                '没有记录 Ts。请用新版 PMPC_SAVE_RESULT.m 重新保存该次仿真。'], ...
                files{k}, required_fields{j});
        end
    end

    D = squeeze(S.data);
    t = double(S.time_s(:));

    if size(D, 1) == 25
        % 已经是 25 × N
    elseif size(D, 2) == 25
        D = D.';
    else
        error('文件 %s 的数据不是 25 通道，尺寸为 %s。', ...
            files{k}, mat2str(size(D)));
    end

    if size(D, 2) ~= numel(t)
        error('文件 %s 的时间点数和数据点数不一致。', files{k});
    end

    runs(k).file = files{k};
    runs(k).Ts = double(S.Ts);
    runs(k).tau_d = double(S.tau_d);
    runs(k).time_s = t;
    runs(k).data = double(D);

    if isfield(S, 'Ts_exec')
        runs(k).Ts_exec = double(S.Ts_exec);
    else
        runs(k).Ts_exec = NaN;
    end
end

% 按 Ts、tau_d 从小到大排列，图例顺序固定且明确
sort_key = [[runs.Ts].', [runs.tau_d].'];
[~, order] = sortrows(sort_key, [1 2]);
runs = runs(order);

labels = cell(1, nRuns);
for k = 1:nRuns
    labels{k} = sprintf('T_s = %.0f ms, \\tau_d = %.1f ms', ...
        runs(k).Ts * 1000, runs(k).tau_d * 1000);
end

%% 3. 图 1：车辆状态响应
figure('Name', 'PMPC车辆状态对比', 'Color', 'w');
tiledlayout(4, 1, 'TileSpacing', 'compact');

nexttile;
hold on;
for k = 1:nRuns
    plot(runs(k).time_s, runs(k).data(1, :), 'LineWidth', 1.25);
end
grid on;
ylabel('e_y (m)');
title('横向路径误差');
legend(labels, 'Location', 'best');

nexttile;
hold on;
for k = 1:nRuns
    plot(runs(k).time_s, runs(k).data(3, :), 'LineWidth', 1.25);
end
grid on;
ylabel('\phi (rad)');
title('侧倾角');

nexttile;
hold on;
for k = 1:nRuns
    plot(runs(k).time_s, runs(k).data(4, :), 'LineWidth', 1.25);
end
grid on;
ylabel('p (rad/s)');
title('侧倾角速度');

nexttile;
hold on;
for k = 1:nRuns
    plot(runs(k).time_s, runs(k).data(5, :), 'LineWidth', 1.25);
end
grid on;
xlabel('Time (s)');
ylabel('LTR');
title('载荷转移率');

%% 4. 图 2：实际侧倾力矩
figure('Name', 'PMPC实际侧倾力矩对比', 'Color', 'w');
hold on;
for k = 1:nRuns
    plot(runs(k).time_s, runs(k).data(6, :), 'LineWidth', 1.3);
end
grid on;
xlabel('Time (s)');
ylabel('M_d (N m)');
title('实际侧倾力矩对比');
legend(labels, 'Location', 'best');

%% 5. 图 3：四角减振器力跟踪误差
figure('Name', 'CDC减振器力跟踪误差对比', 'Color', 'w');
hold on;
for k = 1:nRuns
    D = runs(k).data;
    force_error = D(10:13, :) - D(14:17, :);
    force_error_rms_time = sqrt(mean(force_error.^2, 1, 'omitnan'));
    plot(runs(k).time_s, force_error_rms_time, 'LineWidth', 1.3);
end
grid on;
xlabel('Time (s)');
ylabel('Four-corner force error RMS (N)');
title('四角减振器实际力与期望力的跟踪误差');
legend(labels, 'Location', 'best');

%% 6. 图 4：四角减振器实际力
corner_names = {'L1', 'L2', 'R1', 'R2'};
figure('Name', 'CDC四角减振器实际力对比', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

for corner = 1:4
    nexttile;
    hold on;
    for k = 1:nRuns
        plot(runs(k).time_s, runs(k).data(9 + corner, :), ...
            'LineWidth', 1.15);
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Fd (N)');
    title(corner_names{corner});
    legend(labels, 'Location', 'best');
end

%% 7. 计算定量评价指标
tau_ms = zeros(nRuns, 1);
Ts_ms = zeros(nRuns, 1);
Ts_exec_ms = nan(nRuns, 1);
ey_rms_m = zeros(nRuns, 1);
ey_peak_m = zeros(nRuns, 1);
phi_rms_rad = zeros(nRuns, 1);
phi_peak_rad = zeros(nRuns, 1);
p_rms_rad_s = zeros(nRuns, 1);
p_peak_rad_s = zeros(nRuns, 1);
LTR_peak = zeros(nRuns, 1);
Md_actual_rms_Nm = zeros(nRuns, 1);
Fd_tracking_rms_N = zeros(nRuns, 1);
QP_success_pct = zeros(nRuns, 1);
candidate_used_pct = zeros(nRuns, 1);

for k = 1:nRuns
    D = runs(k).data;
    ey = D(1, :);
    phi = D(3, :);
    p = D(4, :);
    LTR = D(5, :);
    Md_actual = D(6, :);
    force_error = D(10:13, :) - D(14:17, :);

    Ts_ms(k) = runs(k).Ts * 1000;
    tau_ms(k) = runs(k).tau_d * 1000;
    Ts_exec_ms(k) = runs(k).Ts_exec * 1000;

    ey_rms_m(k) = sqrt(mean(ey.^2, 'omitnan'));
    ey_peak_m(k) = max(abs(ey), [], 'omitnan');
    phi_rms_rad(k) = sqrt(mean(phi.^2, 'omitnan'));
    phi_peak_rad(k) = max(abs(phi), [], 'omitnan');
    p_rms_rad_s(k) = sqrt(mean(p.^2, 'omitnan'));
    p_peak_rad_s(k) = max(abs(p), [], 'omitnan');
    LTR_peak(k) = max(abs(LTR), [], 'omitnan');
    Md_actual_rms_Nm(k) = sqrt(mean(Md_actual.^2, 'omitnan'));
    Fd_tracking_rms_N(k) = sqrt(mean(force_error(:).^2, 'omitnan'));
    QP_success_pct(k) = 100 * mean(D(24, :) > 0.5, 'omitnan');
    candidate_used_pct(k) = 100 * mean(D(25, :) > 0.5, 'omitnan');
end

Result = table( ...
    Ts_ms, tau_ms, Ts_exec_ms, ...
    ey_rms_m, ey_peak_m, ...
    phi_rms_rad, phi_peak_rad, ...
    p_rms_rad_s, p_peak_rad_s, ...
    LTR_peak, Md_actual_rms_Nm, ...
    Fd_tracking_rms_N, ...
    QP_success_pct, candidate_used_pct);

fprintf('\n========== PMPC Ts / tau_d 对比结果 ==========\n');
disp(Result);

%% 8. 导出对比指标 CSV
stamp = datestr(now, 'yyyymmdd_HHMMSS');
csv_name = sprintf('PMPC_Comparison_%s.csv', stamp);
csv_path = fullfile(result_dir, csv_name);
writetable(Result, csv_path);

fprintf('对比指标已导出：\n%s\n', csv_path);
fprintf('参与比较的仿真数量：%d\n', nRuns);
for k = 1:nRuns
    fprintf('  Ts = %.1f ms, tau_d = %.1f ms : %s\n', ...
        Ts_ms(k), tau_ms(k), runs(k).file);
end
