function out = run_dlc(ctrl, speed, opts)
%RUN_DLC  DLC 工况统一运行脚本（CarSim 路面 mu=0.4，力界插值逆模型）。
%
%   run_dlc(1, 70)   % MPC，70 km/h
%   run_dlc(2, 70)   % ZENG
%   run_dlc(3, 70)   % Proposed
%
% opts（可选 struct）：
%   .stopTime  仿真时长 [s]，默认 10
%   .outdir    结果目录，默认 PMPC_Results/DLC_mu0.4_interp
%   .prefix    结果文件名前缀，默认 sprintf('dlc_c%d_%g',ctrl,speed)
%   .Q_norm / .R_norm / .slack_w / .rho_I0 / .rho_dI / .Wb /
%   .w1_max / .w2_max   覆盖 pmpc_init 默认值（增益 g 固定为 1）
%
% 依赖：pmpc_mil.slx、simfile_dlc.sim（指向 Run_4d669775-...）、
%       CarSim 2019 GUI/license 已启动。

mdl = 'pmpc_mil';
root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end

if nargin < 3 || isempty(opts), opts = struct(); end
if ~isfield(opts,'stopTime'), opts.stopTime = 15; end
if ~isfield(opts,'outdir')
    opts.outdir = fullfile(root,'PMPC_Results','DLC_mu0.4_interp');
end
if ~isfield(opts,'prefix')
    opts.prefix = sprintf('dlc_c%d_%g', ctrl, speed);
end

simfile = fullfile(root,'simfile_dlc.sim');
assert(isfile(simfile), '缺少 simfile_dlc.sim：%s', simfile);
assert(~isempty(cs_dataset_name(simfile)), ...
    'simfile_dlc.sim 里找不到 Run_<dataset> 名，请检查 simfile。');

% 改 CarSim 展开 par 的初始车速 SV_VXS。
% dataset 目录从 simfile 里推导（每个 worktree 指向自己的副本，
% 避免并发时互相改写 Run_all.par 的 SV_VXS/TSTOP/SSTOP）。
resdir = fullfile('D:\Program Files\CarSim2019.0\CarSim2019.0_Data\Results', cs_dataset_name(simfile));
parfile = fullfile(resdir,'Run_all.par');
assert(isfile(parfile), '缺少 Run_all.par：%s', parfile);
lines = regexp(fileread(parfile),'\r?\n','split');
n = 0;
for k = 1:numel(lines)
    if strncmp(strtrim(lines{k}),'SV_VXS ',7)
        lines{k} = sprintf('SV_VXS %g',speed); n = n + 1;
    end
end
assert(n > 0, 'Run_all.par 中找不到 SV_VXS');
% 初始位置：CarSim 默认 SV_XO=30（车从 X=30 m 起步），改成 0 让 Xo 从原点起步，
% 便于和解析参考路径（X 从 0 开始）直接对齐画图。SV_YO/SV_YAW 保持 0 不变。
for k = 1:numel(lines)
    if startsWith(strtrim(lines{k}),'SV_XO ')
        lines{k} = sprintf('SV_XO 0');
    end
end

% 仿真终止：CarSim 的 TSTOP/SSTOP 决定物理终止，Simulink 的 StopTime 只是上限。
% 为了让三个速度都跑满 200 m（画图对齐），把 TSTOP 放宽、SSTOP 设成目标里程，
% Simulink StopTime 给足（CarSim 到里程会自己停）。
tstop = 20; sstop = 200;
if isfield(opts,'tstop') && isfinite(opts.tstop), tstop = opts.tstop; end
if isfield(opts,'sstop') && isfinite(opts.sstop), sstop = opts.sstop; end
for k = 1:numel(lines)
    if startsWith(strtrim(lines{k}),'TSTOP ')
        lines{k} = sprintf('TSTOP %g', tstop);
    elseif startsWith(strtrim(lines{k}),'SSTOP ')
        lines{k} = sprintf('SSTOP %g', sstop);
    end
end

fid = fopen(parfile,'w'); fprintf(fid,'%s\n',lines{:}); fclose(fid);

addpath(fullfile(root,'pmpc_core'),'-begin');
addpath(fullfile(root,'nlcsnn'),'-begin');

clear functions;
clear pmpc_init pmpc_bus pmpc_step pmpc_preview prepare_step ...
      build_prediction force_bounds_four force_bounds_impl_10ms ...
      inv_current_impl_10ms fwd_mf_impl curr_dyn_impl lon_arbitrate func_ZengRho;

if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(root,[mdl '.slx']));

set_param([mdl '/CarSim'],'SIMFILE',simfile);
allb = find_system(mdl,'FindAll','on','Type','block');
for k = 1:numel(allb)
    try
        if strcmp(get_param(allb(k),'BlockType'),'Constant') && ...
                contains(get_param(allb(k),'Name'),'v_target')
            set_param(allb(k),'Value',sprintf('%g',speed));
        end
    catch
    end
end
set_param(mdl,'StopTime',sprintf('%g',opts.stopTime));

[P,R,s0,m0] = pmpc_init(1,ctrl);
for f = {'Q_norm','R_norm','slack_w','rho_I0','rho_dI','Wb', ...
         'w1_max','w2_max','margin_mode','tau_d_mode','w_fixed','tau_d','tau_lead', ...
         'gamma_on','cdc_mode'}
    if isfield(opts,f{1}), P.M.(f{1}) = opts.(f{1}); end
end

assignin('base','params',P);
assignin('base','Reftraj',R);
assignin('base','state0',s0);
assignin('base','meas0',m0);
[mb,sb,pb] = pmpc_bus(m0,s0);
assignin('base','PMPC_MeasBus',mb);
assignin('base','PMPC_StateBus',sb);
assignin('base','PMPC_PredictionBus',pb);

fprintf('\n===== %s | ctrl=%d | DLC %g km/h | mu=%.2f | stop=%.1f s =====\n', ...
    opts.prefix, ctrl, speed, P.road.mu, opts.stopTime);
fprintf('Q=%s R=%s slack=%s\n', mat2str(P.M.Q_norm'), mat2str(P.M.R_norm'), ...
    mat2str(P.M.slack_w'));

out = sim(mdl,'ReturnWorkspaceOutputs','on');

if ~isfolder(opts.outdir), mkdir(opts.outdir); end
for ext = {'.vs','.vsb'}
    src = fullfile(resdir,['LastRun' ext{1}]);
    if isfile(src)
        copyfile(src, fullfile(opts.outdir,[opts.prefix ext{1}]));
    end
end
L = out.pmpc_log; D = squeeze(L.Data);
if size(D,1) ~= 27, D = D.'; end
t = L.Time;
save(fullfile(opts.outdir,[opts.prefix '.mat']),'D','t');
try
    Icmd = out.pmpc_Icmd;
    save(fullfile(opts.outdir,[opts.prefix '.mat']),'Icmd','-append');
catch
end

ey = D(1,:); epsi = D(2,:); ltr = D(5,:); mdc = D(7,:);
mz = D(18,:); sw = D(19,:); vx = D(20,:); rr = D(21,:);
[~,k] = max(abs(ey));
fprintf(['t_end %.2f | Vx %.1f..%.1f | ey %.3f/%.3f fin %.3f @%.2fs | ' ...
         'epsi %.2f deg | SW %.1f | Mz %.0f/%.0f | Md %.0f/%.0f | ' ...
         'yaw %.1f | LTR %.2f | QP %.1f%%\n'], ...
    t(end), min(vx)*3.6, max(vx)*3.6, max(abs(ey)), sqrt(mean(ey.^2)), ey(end), t(k), ...
    max(abs(rad2deg(epsi))), max(abs(sw)), mean(abs(mz)), max(abs(mz)), ...
    mean(abs(mdc)), max(abs(mdc)), max(abs(rad2deg(rr))), max(abs(ltr)), ...
    100*mean(D(24,:)>0.5));
end
