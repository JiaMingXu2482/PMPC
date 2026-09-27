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
%   R 字段 (MPC / ZENG, 8 松弛结构):
%     t        n x 1   控制器执行时刻
%     cert     n x 11  证书 [r+ r- a+ a- LTR+ LTR- ey+ ey- | AFS DB CDC]
%     eps      n x 8   松弛量(顺序同上)
%     gam      n x 3   gamma [AFS DB CDC]
%   R 字段 (PMPC, PrioMode = 1, 论文 III-D/E):
%     prio     n x 14  先验认证 [d1 d2 F w1 w2 certA certB Lb certC d1_0 d2_0 F0 gamma^ maxInViol]
%     cert     n x 5   后验 [1'lam1/w1  1'lam2/w2  NaN  DB净乘子和/W_b  NaN]
%     eps      n x 2   [s1 s2];  gam n x 1 gamma_DB;  exitflag, fallback  n x 1
%     sys_err  回放输出与块实际输出的最大相对偏差(应在舍入量级)
if nargin < 2, outdir = ''; end

info = run_ds(tag, '-nosim');                % 只切 simfile 指向
evalin('base','clear PMPC_MODE PMPC_ZENGRHO PMPC_P');
dl  = dir(fullfile(info.resdir,'LastRun_log.txt'));
if isempty(dl), ls0 = 0; else, ls0 = dl.datenum; end
assert(func_CarSimRunning(), 'cert_replay:NoCarSim', 'CarSim 没在运行, 先打开 CarSim。');
evalin('base','setup_pmpc;');
func_CarSimLib();

mdl = info.model;
load_system(mdl);
b   = [mdl '/PMPC_MF'];
ph  = get_param(b, 'PortHandles');
lnI = get_param(ph.Inport(1), 'Line');
srcI = get_param(lnI, 'SrcPortHandle');
set_param(srcI, 'DataLogging','on', 'DataLoggingNameMode','Custom', 'DataLoggingName','u_pmpc');
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
tu = U.Time;  ud = squeeze(U.Data);  if size(ud,1) ~= numel(tu), ud = ud.'; end
yd = squeeze(Y.Data);  if size(yd,1) ~= numel(Y.Time), yd = yd.'; end

%  块在触发时刻执行; 记录可能带重复/更密的时刻, 只取 sys 真正更新的那些拍
[tu, iu] = unique(tu, 'last');  ud = ud(iu,:);
[ty, iy] = unique(Y.Time, 'last');  yd = yd(iy,:);
[t, ia, ib] = intersect(round(tu*1e6), round(ty*1e6));
t = t/1e6;  ud = ud(ia,:);  yd = yd(ib,:);

P  = evalin('base','PMPC_P');
clear func_QPKwik pmpc_step          %#ok<CLFUNC>  清掉 KWIK 热启动的 persistent
S  = P.S0;
n  = numel(t);
C  = nan(n, numel(S.cert));
Yr = nan(n, size(yd,2));
for k = 1:n
    [sys, S] = pmpc_step(ud(k,:).', P.Pm, S);
    C(k,:)  = S.cert(:).';
    Yr(k,:) = sys(:).';
end
MP = P.Pm.MPCParameters;
R.t    = t;
if isfield(MP,'PrioMode') && MP.PrioMode == 1
    %  PMPC (论文 III-D/E): St.cert = [先验 14 | 后验 5 | s1 s2 | gamma_DB | exitflag | 兜底]
    R.prio = C(:, 1:14);   % [d1 d2 F w1 w2 certA certB Lb certC d1_0 d2_0 F0 gamma^ maxInViol]
    R.cert = C(:, 15:19);  % [1'lam1/w1  1'lam2/w2  NaN  DB净乘子和/W_b  NaN]
    R.eps  = C(:, 20:21);  % [s1 s2] 归一化违反量
    R.gam  = C(:, 22);     % gamma_DB
    R.exitflag = C(:, 23);
    R.fallback = C(:, 24);
else
    ne = MP.Ne;
    R.cert = C(:, 1:ne+3);
    R.eps  = C(:, ne+4 : 2*ne+3);
    R.gam  = C(:, 2*ne+4 : 2*ne+6);
end
R.sys_err = max(max(abs(Yr - yd) ./ max(abs(yd), 1)));
R.sys  = yd;                         % 块的实际输出(100 Hz), sys(9) = 方向盘转角指令
R.dt   = median(diff(t));
end
