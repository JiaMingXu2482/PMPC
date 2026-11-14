function save_run_params(outdir, prefix, P, ctrl, speed, maneuver)
%SAVE_RUN_PARAMS 把一次算例"实际生效"的参数落到 <prefix>_params.mat。
%
%   save_run_params(opts.outdir, opts.prefix, P, ctrl, speed, maneuver)
%
% 为什么单独落盘：三方对比时要能从磁盘复核每个算例到底用了什么口径，
% 尤其是 tau_d —— pmpc_init 的 Slalom case 给 controller=3 设 0.20（comp 口径），
% noComp 必须在 run_* 的 opts 里显式覆盖成 1e-4，光看代码看不出来跑的是哪个值。
%
% 字段（都是标量或小向量，便于 load 后直接看）：
%   maneuver       工况编号 1=DLC 2=Slalom 3=U-turn 4=FishHook
%   maneuver_name  同上，字符串
%   speed          目标/初始车速 [km/h]
%   horizon        [Ts Np Nc] 预测步长 / 预测步数 / 控制步数
%   Ts, Np, Nc     上面三个的拆开副本
%   tau_d          实际生效的时滞常数 [s]（已含 build_prediction 的 1e-4 下限口径）
%   tau_d_mode     1=常数 M.tau_d；2=按压缩速度时变（comp 口径）
%   controller     1=MPC 2=ZENG 3=Proposed
%   Q_norm / R_norm / slack_w
%   w1_max / w2_max / Wb / rho_I0 / rho_dI / preview_L / preview_alpha（有就存）
%   cdc_mode / margin_mode
%   SH_MODE        从 nlcsnn/inv_current_impl_10ms.m 静态解析（0=力界插值，2=固定电流基线）
%   timestamp      生成时刻（datestr）

maneuver_name = {'DLC','Slalom','U-turn','FishHook'};
if maneuver < 1 || maneuver > numel(maneuver_name)
    maneuver_name_str = sprintf('maneuver%d', maneuver);
else
    maneuver_name_str = maneuver_name{maneuver};
end

horizon = [P.M.Ts, P.M.Np, P.M.Nc];
Ts = P.M.Ts; Np = P.M.Np; Nc = P.M.Nc;
tau_d = P.M.tau_d;
tau_d_mode = P.M.tau_d_mode;
controller = P.M.controller;
Q_norm = P.M.Q_norm;
R_norm = P.M.R_norm;
slack_w = P.M.slack_w;

% 可选字段：存在才存，避免不同工况 struct 不一致时报错
opt = struct();
for f = {'w1_max','w2_max','Wb','rho_I0','rho_dI','preview_L','preview_alpha', ...
         'cdc_mode','margin_mode','w_fixed','tau_lead'}
    if isfield(P.M, f{1}), opt.(f{1}) = P.M.(f{1}); end
end

% SH_MODE 是 inv_current_impl_10ms.m 里的局部常量，base 工作区看不到，静态解析
SH_MODE = NaN;
root = fileparts(mfilename('fullpath'));   % save_run_params.m 就在 worktree 根目录
iv = fullfile(root, 'nlcsnn', 'inv_current_impl_10ms.m');
if isfile(iv)
    tok = regexp(fileread(iv), 'SH_MODE\s*=\s*([0-9.]+)', 'tokens', 'once');
    if ~isempty(tok), SH_MODE = str2double(tok{1}); end
end

timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');   %#ok<TNOW1,DATST>

if ~isfolder(outdir), mkdir(outdir); end
f = fullfile(outdir, [prefix '_params.mat']);
save(f, 'maneuver', 'maneuver_name_str', 'speed', 'horizon', 'Ts', 'Np', 'Nc', ...
     'tau_d', 'tau_d_mode', 'controller', 'Q_norm', 'R_norm', 'slack_w', ...
     'SH_MODE', 'timestamp', 'opt');
fprintf('[params] %s  (tau_d=%g, mode=%d, horizon=[%g %d %d], SH_MODE=%g)\n', ...
    [prefix '_params.mat'], tau_d, tau_d_mode, Ts, Np, Nc, SH_MODE);
end
