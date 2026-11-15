function out = taud_backid(files, varargin)
%TAUD_BACKID 由 logVec 的 Md_actual / Md_cmd 反辨识执行器一阶时间常数。
%
%   taud_backid                                  % 默认跑本仓库四个 cp_* 算例
%   taud_backid(f1)                              % 单个 .mat
%   taud_backid({f1,f2})                         % 多个 .mat 并排比
%   taud_backid(f1,'relThreshold',0.01)          % 指定主门限（占峰值 |Md| 的比例）
%   out = taud_backid(...)                       % 返回结构，便于后续处理
%
% ── 公式 ────────────────────────────────────────────────────────────────
% 预测模型把"力矩命令 -> 实际力矩"当一阶环节（build_prediction.m 的 A(7,7)）：
%
%     dMd_actual/dt = (Md_cmd - Md_actual) / tau
%
% 前向 Euler（Ts = 采样周期）：
%
%     tau(k) = (Md_cmd(k) - Md_actual(k)) / (dMd_actual/dt)(k)
%
% dMd_actual/dt 用中心差分；Ts 取 median(diff(t))。这就是"用实测反推
% 控制器应该假设的那个 tau_d"，不依赖任何 .slx 里的常数。
%
% ── 数据来源（.mat 里 D 的两行，1-based）────────────────────────────────
%     D(6,:) = c.zeta(7)        = Md_actual  实测（Bd'*Fd，来自 CarSim 四角力）
%     D(7,:) = stateNext.u_prev(3) = Md_cmd   本拍下发命令
% 两者同拍配对是对的：act(k) 只响应到 cmd(k-1)，所以 (cmd(k)-act(k)) 正是
% 驱动 act(k+1) 的那个误差。md_cmd 若取上一拍（pmpc_step.m:68 的初值）会整体
% 错一拍，必须用 :160 覆盖后的 stateNext 值。
%
% ── 适用条件与噪声地板（重要，别把结果读细了）──────────────────────────
% * 采样 100 Hz -> Ts = 10 ms。点估计的分辨率就是 ~Ts：凡是估出来的 tau 与
%   10 ms 同量级，都已经落在采样量化噪声里，不能当物理量读。
% * 分母 dMd/dt 在命令平台期接近 0，点估计会炸（除以噪声）。所以只统计
%   |Md_cmd - Md_actual| 超过门限的样本；门限按峰值 |Md| 的相对比例给
%   （默认 1%），并在默认门限之外打印一张门限扫描表 —— 门限一动分位数就动，
%   只报单一数字等于藏证据。
% * 只保留 0 < tau < 1 s 的样本，其余记作不可用。
% * 估出来的是"整条链路的表观滞后"：逆模型、四角分配、CarSim 力建立、
%   QP 的 du 限幅全在里面，不是单一执行器的时间常数。它只能定量级。
%
% 结论口径（写论文时照这个措辞）：100 Hz 下约 10 ms 是噪声地板；表观滞后
% <= 20 ms，分辨不出更细；因此 mode2 的 17-22 ms 标定量级是对的；mode1 的
% 0.20 s 不是执行器滞后，而是把预瞄、力界、离散化误差一起吸收的等效补偿量。
%
% 见 PMPC_Results/CP_RESULTS.md 的「tau_d 口径记录与反辨识」一节（§4.3）。

if nargin < 1 || isempty(files)
    root = fileparts(fileparts(mfilename('fullpath')));   % -> worktree 根
    files = {fullfile(root,'PMPC_Results','DLC_mu0.4_interp','cp_dlc_mode1.mat'), ...
             fullfile(root,'PMPC_Results','DLC_mu0.4_interp','cp_dlc_mode2.mat'), ...
             fullfile(root,'PMPC_Results','Slalom_mu0.85_interp','cp_sla_mode1.mat'), ...
             fullfile(root,'PMPC_Results','Slalom_mu0.85_interp','cp_sla_mode2.mat')};
end
if ischar(files) || isstring(files), files = {char(files)}; end

p = struct('relThreshold',0.01,'sweep',[0 0.001 0.002 0.005 0.008 0.01 0.02 0.05], ...
           'tauMax',1);
for k = 1:2:numel(varargin)
    p.(varargin{k}) = varargin{k+1};
end

out = struct('file',{},'tau_dtrace',{},'nUsable',{},'q',{},'sweep',{});fprintf('\n%s\n', repmat('=',1,78));
fprintf('tau_d 反辨识：tau = (Md_cmd - Md_actual) / (dMd_actual/dt)\n');
fprintf('Ts = 100 Hz 采样，点估计分辨率 ~10 ms；主门限 = %.1f%% of peak |Md|\n', ...
    100*p.relThreshold);
fprintf('%s\n', repmat('=',1,78));

for f = 1:numel(files)
    fn = files{f};
    fprintf('\n%s\n', fn);
    if ~isfile(fn), fprintf('  缺失，跳过\n'); continue; end
    S = load(fn);
    D = double(S.D); t = double(S.t(:));
    if size(D,1) ~= 27, D = D.'; end
    dt = median(diff(t));
    act = D(6,:); cmd = D(7,:);
    der = gradient(act, dt);
    tau = (cmd - act)./der;

    ok  = isfinite(tau) & tau > 0 & tau < p.tauMax;
    mdPk = max(abs(act));
    thr  = p.relThreshold*mdPk;

    % 门限扫描
    fprintf('  %-18s %6s %8s %8s %8s %8s %8s\n', 'threshold', 'n', 'q10', 'q25', 'q50', 'q75', 'q90');
    sw = zeros(numel(p.sweep),7);
    for j = 1:numel(p.sweep)
        m = ok & abs(cmd-act) > p.sweep(j)*mdPk;
        if any(m)
            qq = percentile(tau(m),[10 25 50 75 90])*1000;
            sw(j,:) = [p.sweep(j) sum(m) qq];
            fprintf('  %-18s %6d %8.1f %8.1f %8.1f %8.1f %8.1f\n', ...
                sprintf('%.3g x |Md|max',p.sweep(j)), sum(m), qq);
        else
            sw(j,:) = [p.sweep(j) 0 nan(1,5)];
            fprintf('  %-18s %6d %8s %8s %8s %8s %8s\n', ...
                sprintf('%.3g x |Md|max',p.sweep(j)), 0, '-','-','-','-','-');
        end
    end

    m   = ok & abs(cmd-act) > thr;
    qq  = percentile(tau(m),[10 25 50 75 90])*1000;
    fprintf('  -> 主门限 n=%d/%d, tau [ms] q10/25/50/75/90 = %.1f/%.1f/%.1f/%.1f/%.1f\n', ...
        sum(m), numel(tau), qq);
    % 对照：本算例 .mat 里记录的 tau_dtrace（run_*.m 事后复算的那个值）
    if isfield(S,'tau_dtrace')
        td = double(S.tau_dtrace);
        fprintf('  -> .mat 记录 tau_dtrace = %.5f .. %.5f s (median %.5f)\n', ...
            min(td), max(td), median(td));
    end
    fprintf('  -> |Md|max = %.0f N*m, 门限 = %.0f N*m\n', mdPk, thr);

    out(end+1) = struct('file',fn,'tau_dtrace', ...
        ternary(isfield(S,'tau_dtrace'), @() double(S.tau_dtrace), @() nan), ...
        'nUsable',sum(m),'q',qq,'sweep',sw); %#ok<AGROW>
end
fprintf('\n');
end

% ------------------------------------------------------------------
function q = percentile(x,p)
x = sort(x(:));
n = numel(x);
if n == 0, q = nan(size(p)); return; end
pos = (p(:)/100)*(n-1) + 1;
lo = floor(pos); hi = ceil(pos);
lo = min(max(lo,1),n); hi = min(max(hi,1),n);
w = pos - floor(pos);
q = x(lo).*(1-w) + x(hi).*w;
q = q(:).';
end

function v = ternary(c,a,b)
if c, v = a(); else, v = b(); end
end
