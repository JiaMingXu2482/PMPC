function out = run_nu_fh(variant, gridMode, ctrl, opts)
%RUN_FISHHOOK  FishHook 80 km/h / mu=0.9 / 车顶 cargo 工况统一复现脚本。
%
%   run_fishhook('nocomp', 1)   % MPC baseline
%   run_fishhook('nocomp', 3)   % Proposed，不补偿
%   run_fishhook('comp25', 3)   % Proposed，25 ms 时滞补偿
%
% 变体口径（只影响控制器参数，不改 CarSim 被控对象）：
%   'nocomp' : tau_d=1e-4,  preview_L=0,     preview_alpha=0
%              预测模型认为阻尼力瞬时建立，力界不预瞄。
%   'comp25' : tau_d=0.025, preview_L=0.025, preview_alpha=1
%              预测模型按 25 ms 建立时间提前给力，力界预瞄 25 ms。
%
% opts（可选 struct）：
%   .stopTime  仿真时长 [s]，默认 8
%   .vtarget   目标/初始车速 [km/h]，默认 80
%   .outdir    结果目录，默认 PMPC_Results/FishHook_80_mu0.9_cargo170_newplant
%   .prefix    结果文件名前缀，默认 sprintf('fh_%s_c%d',variant,ctrl)
%   .cargo     车顶 payload 质量 [kg]，写入 CarSim Run_all.par 的 M_PL，默认 170
%   .steer_prep 转向预备段 [s]：把 STEER_SW_TABLE 整体后移并在前面插入 0° 保持，默认 1.0
%               （0 = 不加预备段，保持 CarSim 数据集原表）
%   .tau_d / .preview_L / .preview_alpha / .tau_lead  覆盖变体默认值
%   .cdc_mode  1=连续逆模型（默认），2=SH 开关 CDC（Mz/AFS 不变，做法 A）
%   .model     模型名，默认 'pmpc_mil'；带 CDC 指令延迟用 'pmpc_fh_delay_new'
%   .tau_cdc   CDC 力指令延迟 [s]，仅 pmpc_fh_delay_new 使用，默认 0.001
%   .rebuild   true 时删除 slprj/slxc 强制重编译，默认 false
%   .verbose   打印仿真日志，默认 true
%
% 依赖：pmpc_mil.slx、simfile.sim（指向 FH80_cargo_MPC 数据集）、
%       CarSim 2019 GUI/license 已启动。

root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end

if nargin < 3 || isempty(opts), opts = struct(); end
if ~isfield(opts,'stopTime'),  opts.stopTime = 8;   end
if ~isfield(opts,'vtarget'),   opts.vtarget  = 80;  end
if ~isfield(opts,'outdir')
    opts.outdir = fullfile(root,'results');
end
if ~isfield(opts,'prefix')
    opts.prefix = sprintf('nu_fh_%s_%s_c%d', lower(gridMode), lower(variant), ctrl);
end
if ~isfield(opts,'cargo'),     opts.cargo = 170;   end
if ~isfield(opts,'steer_prep'), opts.steer_prep = 1.0; end
if ~isfield(opts,'cdc_mode'),   opts.cdc_mode = 1;   end
if ~isfield(opts,'model'),     opts.model = 'pmpc_mil'; end
if ~isfield(opts,'tau_cdc'),   opts.tau_cdc = 0.001; end
if ~isfield(opts,'rebuild'),   opts.rebuild = false; end
mdl = 'pmpc_mil_nu';
if ~isfield(opts,'verbose'),   opts.verbose = true;  end

variant = lower(variant);
switch variant
    case 'nocomp'
        cfg = struct('tau_d',1e-4,'preview_L',0,'preview_alpha',0);
    case 'comp25'
        cfg = struct('tau_d',0.025,'preview_L',0.025,'preview_alpha',1);
    otherwise
        error('variant 只能为 ''nocomp'' 或 ''comp25''');
end

simfile = fullfile(root,'simfile.sim');
assert(isfile(simfile), '缺少 simfile.sim：%s', simfile);
txt = fileread(simfile);
assert(~isempty(cs_dataset_name(simfile)), ...
    'simfile.sim 里找不到 Run_<dataset> 名，请检查 simfile。');

% 车顶 payload 质量写进 CarSim 展开后的 Run_all.par（M_PL）。
% dataset 目录从 simfile 里推导（每个 worktree 指向自己的副本，
% 避免并发时互相改写 Run_all.par 的 SV_VXS/TSTOP/SSTOP）。
resdir = fullfile('D:\Program Files\CarSim2019.0\CarSim2019.0_Data\Results', cs_dataset_name(simfile));
parfile = fullfile(resdir,'Run_all.par');
assert(isfile(parfile), '缺少 Run_all.par：%s', parfile);
lines = regexp(fileread(parfile),'\r?\n','split');
hit = find(startsWith(strtrim(lines),'M_PL '),1);
assert(~isempty(hit), 'Run_all.par 中找不到 M_PL');
lines{hit} = sprintf('M_PL %g',opts.cargo);

% 车顶货箱质心高度 H_CG_PL [mm]（opt.h_pl 以 m 给出）。降低它可减小侧翻倾向。
if isfield(opts,'h_pl') && isfinite(opts.h_pl)
    nh = 0;
    for k = 1:numel(lines)
        if startsWith(strtrim(lines{k}),'H_CG_PL ')
            lines{k} = sprintf('H_CG_PL %g', opts.h_pl*1000); nh = nh + 1;
        end
    end
    fprintf('H_CG_PL %.2f m applied to %d line(s)\n', opts.h_pl, nh);
end

% 路面附着：改 CarSim 展开 par 的 MU_ROAD_CONSTANT（两处：road + friction map）。
% 控制器内部的 Road.mu 在 pmpc_init 之后同步覆盖，保证模型与路面一致。
nmu = 0;
if isfield(opts,'mu') && isfinite(opts.mu)
    for k = 1:numel(lines)
        if strncmp(strtrim(lines{k}),'MU_ROAD_CONSTANT ',17)
            lines{k} = sprintf('MU_ROAD_CONSTANT %g',opts.mu); nmu = nmu + 1;
        end
    end
    fprintf('mu %.2f applied to %d MU_ROAD_CONSTANT lines\n', opts.mu, nmu);
end

% 初始位置：CarSim 默认 SV_XO=30（车从 X=30 m 起步），改成 0 让 Xo 从原点起步，
% 便于和解析参考路径（X 从 0 开始）直接对齐画图。SV_YO/SV_YAW 保持 0 不变。
x0 = 0;
if isfield(opts,'x0') && isfinite(opts.x0), x0 = opts.x0; end
for k = 1:numel(lines)
    if startsWith(strtrim(lines{k}),'SV_XO ')
        lines{k} = sprintf('SV_XO %g', x0);
    end
end

% 转向预备段：把 STEER_SW_TABLE 整体后移 prep 秒，并在最前面插入 0° 保持点。
% 已加过（存在 t>0 且角度为 0 的点）则跳过，保证重复调用幂等。
if opts.steer_prep > 0
    i0 = find(strncmp(strtrim(lines),'STEER_SW_TABLE',14),1);
    if ~isempty(i0)
        i1 = i0 + find(strncmp(strtrim(lines(i0+1:end)),'ENDTABLE',8),1);
        tab = strtrim(lines(i0+1:i1-1));
        vals = nan(numel(tab),2);
        for q = 1:numel(tab)
            v = sscanf(strrep(tab{q},',',' '),'%f %f');
            if numel(v) == 2, vals(q,:) = v(:).'; end
        end
        vals = vals(all(isfinite(vals),2),:);
        already = any(vals(2:end,1) > 0 & abs(vals(2:end,2)) < 1e-9);
        if ~already && ~isempty(vals)
            vals(:,1) = vals(:,1) + opts.steer_prep;
            vals = [0 0; vals];
            newtab = arrayfun(@(q) sprintf('%.4f, %g', vals(q,1), vals(q,2)), ...
                              1:size(vals,1), 'UniformOutput', false);
            lines = [lines(1:i0).'; newtab(:); lines(i1:end).'];
            fprintf('steer prep %.2f s applied, table: %s\n', opts.steer_prep, ...
                    strjoin(strtrim(newtab(:)).', ' | '));
        end
    end
end

% 可选：重写转向速率（deg/s）。保持 1 s 预备段与 0.16 s 保持段，
% 只改上升/反向斜坡的时长。用于扫描工况烈度。
if isfield(opts,'steer_rate') && opts.steer_rate>0
    R=opts.steer_rate;
    t1=1+294/R; t2=t1+0.16; t3=t2+588/R; t4=t3+2.6;
    tab={sprintf('%.4f, 0',0); sprintf('%.4f, 0',1); ...
         sprintf('%.4f, 294',t1); sprintf('%.4f, 294',t2); ...
         sprintf('%.4f, -294',t3); sprintf('%.4f, -294',t4)};
    i0=find(strncmp(strtrim(lines),'STEER_SW_TABLE',14),1);
    if ~isempty(i0)
        i1=i0+find(strncmp(strtrim(lines(i0+1:end)),'ENDTABLE',8),1);
        lines=[lines(1:i0).'; tab; lines(i1:end).'];
        fprintf('steer rate %.0f deg/s applied\n',R);
    end
end

fid = fopen(parfile,'w');
fprintf(fid,'%s\n',lines{:});
fclose(fid);

addpath(fullfile(root,'pmpc_core'),'-begin');
addpath(fullfile(root,'nlcsnn'),'-begin');

% 清函数缓存；必要时清编译缓存，避免 MATLAB Function 使用旧代码。
clear functions;
clear pmpc_init pmpc_bus pmpc_step pmpc_preview prepare_step ...
      build_prediction force_bounds_four force_bounds_impl_10ms ...
      inv_current_impl_10ms fwd_mf_impl curr_dyn_impl lon_arbitrate func_ZengRho;
if opts.rebuild
    slxc = fullfile(root,[mdl '.slxc']);
    if isfile(slxc), delete(slxc); end
    slprj = fullfile(root,'slprj');
    if isfolder(slprj), rmdir(slprj,'s'); end
end

% 注意：不保存模型。set_param 会把 v_target/StopTime/SIMFILE 写脏，
% 若在这里 save_system 会把这些运行参数烘进 slx（2026-11-11 踩过）。
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
load_system(fullfile(root,[mdl '.slx']));

% CarSim 句柄 + 目标车速 + 时长。
set_param([mdl '/CarSim'],'SIMFILE',simfile);
allb = find_system(mdl,'FindAll','on','Type','block');
for k = 1:numel(allb)
    try
        if strcmp(get_param(allb(k),'BlockType'),'Constant') && ...
                contains(get_param(allb(k),'Name'),'v_target')
            set_param(allb(k),'Value',sprintf('%g',opts.vtarget));
        end
    catch
    end
end
set_param(mdl,'StopTime',sprintf('%g',opts.stopTime));

% 控制器参数：FishHook + 指定控制器 + 瞬态时域，再按变体覆盖时滞口径。
% FishHook 的参考时域是 0.6 s（12 x 0.05）。非均匀档位保持同样总长：
% 近段 10 x 5 ms + 3 x 50 ms + 4 x 100 ms = 0.60 s，共 17 节点。
if strcmpi(gridMode,'uniform')
    h = struct('Ts',0.05,'dt_grid',0.05*ones(1,12),'Nc',6);
elseif strcmpi(gridMode,'nu')
    h = struct('Ts',0.05,'dt_grid',[0.005*ones(1,10),0.05*ones(1,3),0.1*ones(1,4)],'Nc',6);
else
    error('gridMode 只能是 uniform 或 nu');
end
muOpt = [];
if isfield(opts,'mu') && isfinite(opts.mu), muOpt = opts.mu; end
[P,R,s0,m0] = pmpc_init(4,ctrl,h,muOpt);
P.M.tau_d         = cfg.tau_d;
P.M.preview_L     = cfg.preview_L;
P.M.preview_alpha = cfg.preview_alpha;
for f = {'tau_d','preview_L','preview_alpha','tau_lead','cdc_mode', ...
         'w1_max','w2_max','margin_mode','tau_d_mode','tau_lead_eff','R_norm','Q_norm','slack_w','w_env','Wb', ...
         'afs_on','mz_on','w_prev_alloc'}
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
assignin('base','tau_cdc',opts.tau_cdc);   % 仅 pmpc_fh_delay_new 使用
if ~isempty(find_system(mdl,'FindAll','on','Name','FdesDelay'))
    % 写成数值，避免 Transport Delay 在编译时缓存旧变量值。
    set_param([mdl '/FdesDelay'],'DelayTime',sprintf('%g',opts.tau_cdc));
end

fprintf('\n===== %s | ctrl=%d | %s | %.0f km/h | cargo %g kg | stop=%.1f s | Ts=%.3f Np=%d Nc=%d =====\n', ...
    opts.prefix, ctrl, variant, opts.vtarget, opts.cargo, opts.stopTime, P.M.Ts, P.M.Np, P.M.Nc);
fprintf('tau_d=%.4g s, preview_L=%.4g s, preview_alpha=%g, mu=%.2f\n', ...
    P.M.tau_d, P.M.preview_L, P.M.preview_alpha, P.road.mu);

out = sim(mdl,'ReturnWorkspaceOutputs','on');

% 结果落盘：CarSim 的 .vs/.vsb 与控制器 logVec。
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
% 若模型带 Icmd_log（pmpc_fishhook_log），一并保存四角电流。
try
    Icmd = out.pmpc_Icmd;
    save(fullfile(opts.outdir,[opts.prefix '.mat']),'Icmd','-append');
catch
end
try
    Fd1k = out.pmpc_Fd1k;
    save(fullfile(opts.outdir,[opts.prefix '.mat']),'Fd1k','-append');
catch
end

% 摘要（logVec 行号与 pmpc_step 头部注释一致）。
ey = D(1,:); phi = D(3,:); ltr = D(5,:); mdc = D(7,:);
mz = D(18,:); sw = D(19,:); vx = D(20,:); rr = D(21,:);
fprintf(['t_end %.2f | Vx %.1f..%.1f | ey %.2f/%.2f | roll %.2f deg | ' ...
         'LTR %.2f | SW %.1f | Mz %.0f/%.0f | Md %.0f/%.0f | r %.1f | QP %.1f%%\n'], ...
    t(end), min(vx)*3.6, max(vx)*3.6, max(abs(ey)), sqrt(mean(ey.^2)), ...
    max(abs(phi))*180/pi, max(abs(ltr)), max(abs(sw)), ...
    mean(abs(mz)), max(abs(mz)), mean(abs(mdc)), max(abs(mdc)), ...
    max(abs(rr))*180/pi, 100*mean(D(24,:)>0.5));

if opts.verbose
    fprintf('结果已保存到 %s\n', opts.outdir);
end
end
