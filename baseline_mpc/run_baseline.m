function out = run_baseline(scenario, speed, opts)
%RUN_BASELINE 跑一个基线 MPC 算例（DLC / Slalom / FishHook）。
%
%   run_baseline('dlc', 80)
%   run_baseline('slalom', 80)
%   run_baseline('fishhook', 80, 'cargo', 190)
%
% opts（name-value）：
%   cargo     车顶载荷 [kg]（仅 FishHook）
%   stopTime  仿真时长 [s]
%   prefix    结果文件前缀
%   outdir    结果目录
%   model     模型名，默认 'pmpc_base'
%   Q_norm / R_norm / slack_w / cdc_fixed_I   覆盖 pmpc_base_init 默认值
%
% 依赖：pmpc_base.slx（先跑 build_pmpc_base）、CarSim 正常 GUI、对应 simfile。

if nargin < 2, speed = 80; end
if nargin < 3, opts = struct(); end
if ischar(opts), opts = struct(); end   % 兼容旧的空参

mdl  = 'pmpc_base';
here = fileparts(mfilename('fullpath'));
root = fileparts(here);

% ---- 工况映射 ----
switch lower(scenario)
    case 'dlc',      man = 1;  simfile = fullfile(root,'simfile_dlc.sim');
    case 'slalom',   man = 2;  simfile = fullfile(root,'simfile_slalom.sim');
    case 'fishhook', man = 4;  simfile = fullfile(root,'simfile.sim');
    otherwise, error('scenario 只能是 dlc / slalom / fishhook');
end
assert(isfile(simfile), '缺少 simfile：%s', simfile);

if ~isfield(opts,'stopTime'), opts.stopTime = 15; end
if ~isfield(opts,'prefix')
    opts.prefix = sprintf('base_%s_%g', lower(scenario), speed);
end
if ~isfield(opts,'outdir')
    opts.outdir = fullfile(root,'PMPC_Results','Baseline_MPC');
end
if ~isfolder(opts.outdir), mkdir(opts.outdir); end

addpath(here, '-begin');
clear functions;
clear pmpc_base_init pmpc_base_bus pmpc_base_prepare pmpc_base_predict pmpc_base_step

% ---- CarSim 展开 par：初速 + 载荷 + 路面 + 起始位置 ----
[resdir, parfile] = scenario_resdir(simfile);
lines = regexp(fileread(parfile), '\r?\n', 'split');
for k = 1:numel(lines)
    s = strtrim(lines{k});
    if strncmp(s, 'SV_VXS ', 7)
        lines{k} = sprintf('SV_VXS %g', speed);
    elseif strncmp(s, 'SV_XO ', 6)
        lines{k} = 'SV_XO 0';
    end
end
if isfield(opts,'cargo') && isfinite(opts.cargo)
    for k = 1:numel(lines)
        if strncmp(strtrim(lines{k}), 'M_PL ', 5)
            lines{k} = sprintf('M_PL %g', opts.cargo);
        end
    end
end
fid = fopen(parfile, 'w'); fprintf(fid, '%s\n', lines{:}); fclose(fid);

% ---- 工作区 ----
horizon = struct('Ts',0.05,'Np',12,'Nc',6);
% InitFcn 只传 here，其余从基础工作区的 base_* 读，先写进去
assignin('base','base_maneuver', man);
assignin('base','base_horizon',  horizon);
assignin('base','base_opts',     opts);
assignin('base','base_vtarget',  speed);
pmpc_base_setup(here, man, horizon, opts);

% ---- 模型 ----
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(fullfile(here, [mdl '.slx']));
set_param([mdl '/CarSim'], 'SIMFILE', simfile);
set_param(mdl, 'StopTime', sprintf('%g', opts.stopTime));

% ---- 仿真 ----
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

% ---- 落盘 ----
L = out.pmpc_log;  D = squeeze(L.Data);
if size(D,1) ~= 25, D = D.'; end
t = L.Time;
save(fullfile(opts.outdir,[opts.prefix '.mat']), 'D', 't');
for ext = {'.vs','.vsb'}
    src = fullfile(resdir, ['LastRun' ext{1}]);
    if isfile(src)
        copyfile(src, fullfile(opts.outdir,[opts.prefix ext{1}]));
    end
end

% ---- 摘要 ----
ey = D(1,:); phi = D(3,:); sw = D(19,:); vx = D(20,:);
mz = D(18,:); rr = D(21,:);
fprintf(['t_end %.2f | Vx %.1f..%.1f | ey %.3f/%.3f | roll %.2f deg | ' ...
         'SW %.1f | Mz %.0f/%.0f | r %.1f | QP %.1f%%\n'], ...
    t(end), min(vx)*3.6, max(vx)*3.6, max(abs(ey)), sqrt(mean(ey.^2)), ...
    max(abs(phi))*180/pi, max(abs(sw)), mean(abs(mz)), max(abs(mz)), ...
    max(abs(rr))*180/pi, 100*mean(D(24,:) > 0.5));
end

%% ------------------------------------------------------------------
function [resdir, parfile] = scenario_resdir(simfile)
% 从 simfile 里解析 Run_xxxx 目录名
txt = fileread(simfile);
% 用共享工具解析（支持 Run_bl_slalom 这类带下划线的名字）；
% 旧正则 Run_[0-9a-f\-]+ 会把 Run_bl_slalom 截成 Run_b。
tok = cs_dataset_name(simfile);
assert(~isempty(tok), 'simfile 里找不到 Run_… 目录');
resdir  = fullfile('D:\Program Files\CarSim2019.0\CarSim2019.0_Data\Results', tok);
parfile = fullfile(resdir, 'Run_all.par');
assert(isfile(parfile), '缺少 Run_all.par：%s', parfile);
end
