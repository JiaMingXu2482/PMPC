function R = cert_replay(tag, outdir)
%CERT_REPLAY  跑一次仿真并离线回放 pmpc_step, 取出每拍的在线证书(论文 III-E)
%   R = cert_replay('PMPC')            数据集 <前缀>_PMPC, ERD 不另存
%   R = cert_replay('PMPC', 'erd_xx')  同时把 ERD 复制到 erd_xx\PMPC.vs/.vsb
%
%   为什么要回放:
%     证书在 pmpc_step 里算(St.cert), 但 MATLAB Function 块里不能用
%     coder.extrinsic 往外写(块会推不出输出维度), 块的 persistent 状态仿真时
%     也读不到。所以: 仿真时只打开 PMPC_MF 输入 u 与输出 sys 的信号记录
%     (**只改内存中的模型, 不存盘**), 仿真后用记录的 u 逐拍离线调用 pmpc_step,
%     从返回的 St.cert 读证书。开环回放: 内部状态由同一串输入推动, 与仿真一致
%     (编译执行 vs 解释执行只差末位舍入, 用 R.sys_err 检验)。
%
%   St.cert 固定布局 (36x1):
%     [先验认证14 | 后验证书11 | 松弛8 | gamma_DB | exitflag | 兜底标志]
%   非 PMPC 的先验认证和两个状态标志用 NaN 占位，保证 MATLAB Function
%   状态尺寸在不同控制器间一致。
%   R 字段 (MPC / ZENG, 8 松弛结构):
%     t        n x 1   控制器执行时刻
%     cert     n x 11  证书 [r+ r- a+ a- LTR+ LTR- ey+ ey- | AFS DB CDC]
%     eps      n x 8   松弛量(顺序同上)
%     gam      n x 1   gamma_DB
%   R 字段 (PMPC, PrioMode = 1, 论文 III-D/E):
%     prio     n x 14  先验认证 [d1 d2 F w1 w2 certA certB Lb certC d1_0 d2_0 F0 gamma^ maxInViol]
%     cert     n x 11  后验证书(其余 6 项沿用通用求解器证书)
%     eps      n x 8   松弛量; gam n x 1 gamma_DB; exitflag, fallback  n x 1
%     sys_err  回放输出与块实际输出的最大相对偏差(应在舍入量级)
if nargin < 2, outdir = ''; end

info = run_ds(tag, '-nosim');                % 只切 simfile 指向
evalin('base','clear PMPC_MODE PMPC_ZENGRHO PMPC_P MPC_P ZENG_P');
dl  = dir(fullfile(info.resdir,'LastRun_log.txt'));
if isempty(dl), ls0 = 0; else, ls0 = dl.datenum; end
assert(func_CarSimRunning(), 'cert_replay:NoCarSim', 'CarSim 没在运行, 先打开 CarSim。');
assignin('base', 'PMPC_VERBOSE', 0);
switch info.model
    case 'mpc_mil'
        mil_init_MPC;
        bundleName = 'MPC_P';
    case 'zeng_mil'
        mil_init_ZENG;
        bundleName = 'ZENG_P';
    case 'pmpc_mil'
        mil_init_PMPC;
        bundleName = 'PMPC_P';
end
func_CarSimLib();

mdl = info.model;
load_system(mdl);
b   = [mdl '/PMPC_MF'];
ph  = get_param(b, 'PortHandles');
local_log_input_(ph.Inport(1), 'u_pmpc');
local_log_input_(ph.Inport(2), 'h_pmpc');
local_log_input_(ph.Inport(3), 'x_pmpc');
local_log_input_(ph.Inport(4), 'v_pmpc');
local_log_input_(ph.Inport(5), 'a_pmpc');
set_param(ph.Outport(1), 'DataLogging','on', 'DataLoggingNameMode','Custom', 'DataLoggingName','sys_pmpc');
out = sim(mdl);
func_WaitERD(info.resdir, ls0);
close_system(mdl, 0);                        % 不存盘: 记录开关只在本次内存里
if ~isempty(outdir)
    if ~exist(outdir,'dir'), mkdir(outdir); end
    copyfile(fullfile(info.resdir,'LastRun.vsb'), fullfile(outdir,[tag '.vsb']));
    copyfile(fullfile(info.resdir,'LastRun.vs'),  fullfile(outdir,[tag '.vs']));
end

U  = out.logsout.getElement('u_pmpc').Values;
Y  = out.logsout.getElement('sys_pmpc').Values;
H  = out.logsout.getElement('h_pmpc').Values;
X  = out.logsout.getElement('x_pmpc').Values;
V  = out.logsout.getElement('v_pmpc').Values;
A  = out.logsout.getElement('a_pmpc').Values;
tu = U.Time;  ud = local_time_first_(U);
yd = local_time_first_(Y);

%  块在触发时刻执行; 记录可能带重复/更密的时刻, 只取 sys 真正更新的那些拍
[tu, iu] = unique(tu, 'last');  ud = ud(iu,:);
[ty, iy] = unique(Y.Time, 'last');  yd = yd(iy,:);
[t, ia, ib] = intersect(round(tu*1e6), round(ty*1e6));
t = t/1e6;  ud = ud(ia,:);  yd = yd(ib,:);
hd = local_align_(H, t);
xd = local_align_(X, t);
vd = local_align_(V, t);
ad = local_align_(A, t);

P  = evalin('base', bundleName);
clear func_QPKwik pmpc_step          %#ok<CLFUNC>  清掉 KWIK 热启动的 persistent
S  = P.S0;
n  = numel(t);
C  = nan(n, numel(S.cert));
Yr = nan(n, size(yd,2));
for k = 1:n
    [sys, S] = pmpc_step(ud(k,:).', squeeze(hd(k,:,:)), ...
        squeeze(xd(k,:)).', squeeze(vd(k,:)).', squeeze(ad(k,:)).', P.Pm, S);
    C(k,:)  = S.cert(:).';
    Yr(k,:) = sys(:).';
end
R.t    = t;
if isfield(P.S0,'Constraints') && isfield(P.S0.Constraints,'PrioModeRT') && P.S0.Constraints.PrioModeRT == 1
    %  PMPC (论文 III-D/E): St.cert = [先验14 | cert11 | eps8 | gamma_DB | exitflag | 兜底]
    R.prio = C(:, 1:14);   % [d1 d2 F w1 w2 certA certB Lb certC d1_0 d2_0 F0 gamma^ maxInViol]
    R.cert = C(:, 15:25);  % 11 个通用后验证书
    R.eps  = C(:, 26:33);  % 8 个松弛量
    R.gam  = C(:, 34);     % gamma_DB
    R.exitflag = C(:, 35);
    R.fallback = C(:, 36);
else
    R.cert = C(:, 15:25);
    R.eps  = C(:, 26:33);
    R.gam  = C(:, 34);
end
R.sys_err = max(max(abs(Yr - yd) ./ max(abs(yd), 1)));
R.sys  = yd;                         % 块的实际输出(100 Hz), sys(9) = 方向盘转角指令
R.dt   = median(diff(t));
end

function local_log_input_(inport, name)
line = get_param(inport, 'Line');
source = get_param(line, 'SrcPortHandle');
set_param(source, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', ...
    'DataLoggingName', name);
end

function data = local_time_first_(values)
data = values.Data;
n = numel(values.Time);
if size(data,1) == n
    return
end
timeDim = find(size(data) == n, 1, 'last');
assert(~isempty(timeDim), 'cert_replay:TimeDimension', ...
    '无法识别记录信号的时间维度。');
order = [timeDim 1:timeDim-1 timeDim+1:ndims(data)];
data = permute(data, order);
end

function data = local_align_(values, t)
data = local_time_first_(values);
tv = values.Time(:);
[tv, index] = unique(tv, 'last');
data = data(index,:,:,:);
[found, index] = ismember(round(t*1e6), round(tv*1e6));
assert(all(found), 'cert_replay:InputAlignment', ...
    '控制器输入记录与执行时刻不一致。');
data = data(index,:,:,:);
end
