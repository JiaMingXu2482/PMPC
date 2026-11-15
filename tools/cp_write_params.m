function f = cp_write_params(outdir, prefix, maneuver, speed, P, extra)
%CP_WRITE_PARAMS 把本算例真正生效的口径写成 <prefix>_params.mat。
%
%   cp_write_params(outdir, prefix, 'DLC', 70, P, struct('mu',0.4,'stopTime',15))
%
% 为什么单独落一个文件：.mat 里只有结果（D/t/Icmd/tau_dtrace），没有参数；
% 三方对比时最常被问的两件事——"这次用的是哪个 tau_d"、"权重是不是默认值"
% ——必须能从磁盘直接查，不能靠跑的人回忆。
%
% 字段：
%   maneuver    'DLC' / 'Slalom' / 'FishHook'
%   speed       km/h（FishHook 用 vtarget）
%   horizon     [Ts Np Nc]
%   tau_d       最终生效的常数口径（= P.M.tau_d）。tau_d_mode=2 时这个值被忽略，
%               但照记：它是"切回 mode=1 会用的那个数"
%   tau_d_mode  1 = 常数；2 = 按四角压缩速度时变（见 pmpc_core/tau_d_effective.m）
%   tau_d_const 同 tau_d（与 .mat 里的同名变量保持一致，便于交叉核对）
%   tau_d_default pmpc_init 给的 case 默认值（opts 覆盖之前）
%   controller  P.M.controller（1=MPC 2=ZENG 3=Proposed）
%   Q_norm / R_norm / slack_w  本算例生效的权重（列向量，与 pmpc_init 一致）
%   SH_MODE     nlcsnn/inv_current_impl_10ms.m 里的减振器反解模式（0=控制器给电流）
%   timestamp   写入时刻
%   extra       调用方补充的字段（mu / stopTime / variant / cargo ...）
%
% 只读不写别的文件；不做任何参数判断，只做记录。

if nargin < 6 || isempty(extra), extra = struct(); end

prm = struct();
prm.maneuver   = maneuver;
prm.speed      = speed;
prm.Ts         = P.M.Ts;
prm.Np         = P.M.Np;
prm.Nc         = P.M.Nc;
prm.horizon    = [P.M.Ts P.M.Np P.M.Nc];
prm.tau_d      = P.M.tau_d;
prm.tau_d_mode = P.M.tau_d_mode;
prm.tau_d_const = P.M.tau_d;
if isfield(extra,'tau_d_default')
    prm.tau_d_default = extra.tau_d_default;
    extra = rmfield(extra,'tau_d_default');
end
prm.controller = P.M.controller;
prm.Q_norm     = P.M.Q_norm(:);
prm.R_norm     = P.M.R_norm(:);
prm.slack_w    = P.M.slack_w(:);
prm.SH_MODE    = sh_mode(P);
prm.timestamp  = datestr(now,31);

fn = fieldnames(extra);
for k = 1:numel(fn)
    prm.(fn{k}) = extra.(fn{k});
end

% 只在 P.M 里存在时才记录的旋钮：不写死默认值，缺省就是"这一项没被设置"。
%   tau_lead_eff —— mode 5 的超前时间（缺省取 tau_d，无法从别处反推）
%   preview_L / preview_alpha —— 力界预瞄（FishHook comp25 才有）
for nm = {'tau_lead_eff','preview_L','preview_alpha'}
    if isfield(P.M, nm{1})
        prm.(nm{1}) = P.M.(nm{1});
    end
end

if ~isfolder(outdir), mkdir(outdir); end
f = fullfile(outdir,[prefix '_params.mat']);
save(f,'-struct','prm');
end

% ------------------------------------------------------------------
function m = sh_mode(P)
%SH_MODE 从 nlcsnn/inv_current_impl_10ms.m 解析（那里是唯一来源，不在 P 里）。
m = nan;
root = fileparts(fileparts(mfilename('fullpath')));   % tools/ -> worktree 根
src = fullfile(root,'nlcsnn','inv_current_impl_10ms.m');
if isfile(src)
    txt = fileread(src);
    tok = regexp(txt,'SH_MODE\s*=\s*(\d+)','tokens','once');
    if ~isempty(tok), m = str2double(tok{1}); end
end
end
