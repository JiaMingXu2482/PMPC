%% PMPC_SAVE_RESULT.m
% 每次 Simulink 仿真结束后，在 MATLAB 基础工作区运行本脚本。
% 保存实际传入 PMPC 控制器的 params.M.Ts 和 params.M.tau_d，
% 避免不同预测步长的实验在比较时无法区分。

clc;

%% 1. 读取控制器实际使用的参数
if ~exist('params', 'var') || ~isstruct(params) || ...
        ~isfield(params, 'M') || ~isstruct(params.M)
    error('没有找到 params.M。请先运行 pmpc_init，并确认参数已传入基础工作区。');
end

if ~isfield(params.M, 'Ts')
    error('params.M.Ts 不存在，无法记录预测模型离散步长。');
end
if ~isfield(params.M, 'tau_d')
    error('params.M.tau_d 不存在，无法记录减振器力响应时间常数。');
end

Ts = double(params.M.Ts);
tau_d = double(params.M.tau_d);

if isfield(params.M, 'Ts_exec')
    Ts_exec = double(params.M.Ts_exec);
else
    Ts_exec = NaN;
end

if ~isscalar(Ts) || ~isfinite(Ts) || Ts <= 0
    error('params.M.Ts 必须是大于 0 的有限标量。');
end
if ~isscalar(tau_d) || ~isfinite(tau_d) || tau_d <= 0
    error('params.M.tau_d 必须是大于 0 的有限标量。');
end

fprintf('本次控制器参数：\n');
fprintf('  M.Ts     = %.6f s (%.1f ms)\n', Ts, Ts * 1000);
fprintf('  M.tau_d  = %.6f s (%.1f ms)\n', tau_d, tau_d * 1000);
if isfinite(Ts_exec)
    fprintf('  M.Ts_exec= %.6f s (%.1f ms)\n', Ts_exec, Ts_exec * 1000);
end

%% 2. 检查仿真输出
if ~exist('out', 'var')
    error('未找到 out。请先运行 Simulink 仿真。');
end

try
    log_saved = out.pmpc_log;
catch
    error('无法读取 out.pmpc_log，请检查 To Workspace 变量名。');
end

if ~isa(log_saved, 'timeseries')
    error('out.pmpc_log 不是 timeseries。请检查 To Workspace 的保存格式。');
end

%% 3. 整理时间和数据
time_s = double(log_saved.Time(:));
data = squeeze(log_saved.Data);

if size(data, 1) == 25
    % 25 × N，保持不变
elseif size(data, 2) == 25
    data = data.';
else
    error('日志不是 25 通道，squeeze 后尺寸为 %s。', mat2str(size(data)));
end

if size(data, 2) ~= numel(time_s)
    error('时间点数 %d 与数据采样点数 %d 不一致。', ...
        numel(time_s), size(data, 2));
end

%% 4. 当前 25 个日志通道定义
channel_names = { ...
    'ey', 'epsi', 'phi', 'p', 'LTR', ...
    'Md_actual', 'Md_cmd', 'Md_min', 'Md_max', ...
    'Fd_L1', 'Fd_L2', 'Fd_R1', 'Fd_R2', ...
    'Fdes_L1', 'Fdes_L2', 'Fdes_R1', 'Fdes_R2', ...
    'Mz_cmd', 'SW', 'Vx', 'r', ...
    'deltaMd', 'deltaMd_max', 'QP_OK', 'candidate_used'};

channel_units = { ...
    'm', 'rad', 'rad', 'rad/s', '-', ...
    'N*m', 'N*m', 'N*m', 'N*m', ...
    'N', 'N', 'N', 'N', ...
    'N', 'N', 'N', 'N', ...
    'N*m', 'deg', 'm/s', 'rad/s', ...
    'N*m', 'N*m', '-', '-'};

%% 5. 创建结果目录和唯一文件名
result_dir = fullfile(pwd, 'PMPC_Results');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

tau_tag = strrep(sprintf('%.4f', tau_d), '.', 'p');
Ts_tag = strrep(sprintf('%.4f', Ts), '.', 'p');
time_tag = datestr(now, 'yyyymmdd_HHMMSS_FFF');
file_name = sprintf('PMPC_tau_%s_Ts_%s_%s.mat', ...
    tau_tag, Ts_tag, time_tag);
file_path = fullfile(result_dir, file_name);

saved_at = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');

if numel(time_s) >= 2
    sample_time_s = median(diff(time_s));
else
    sample_time_s = NaN;
end

try
    model_name = bdroot;
catch
    model_name = '';
end

%% 6. 保存结果和实验参数
save(file_path, ...
    'Ts', 'Ts_exec', 'tau_d', 'log_saved', 'time_s', 'data', ...
    'channel_names', 'channel_units', ...
    'sample_time_s', 'saved_at', 'model_name', '-v7.3');

fprintf('\n============================================\n');
fprintf('PMPC 仿真结果保存成功\n');
fprintf('预测模型步长 Ts：%.3f ms\n', Ts * 1000);
fprintf('减振器时间常数 tau_d：%.3f ms\n', tau_d * 1000);
fprintf('采样点数：%d\n', numel(time_s));
fprintf('通道数：%d\n', size(data, 1));
fprintf('保存文件：%s\n', file_path);
fprintf('============================================\n\n');
