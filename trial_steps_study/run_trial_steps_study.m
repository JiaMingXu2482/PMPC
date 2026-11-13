function run_trial_steps_study(caseName, varargin)
%RUN_TRIAL_STEPS_STUDY  试算步数对力界 Flo/Fhi 的影响。
%
%   run_trial_steps_study()              % 默认 FishHook c3_c
%   run_trial_steps_study('dlc')         % DLC c3c 70
%
%   做法：用真实闭环记录的运动(CmpD/CmpRD, 100 Hz)和电流(Icmd, 1 kHz)
%   以 1 kHz 重放 force_bounds_test，把 trial_mean_force 的试算步数
%   分别设为 10/5/3/2/1，比较 Flo/Fhi 相对 10 步参考的偏差。
%   除试算步数外，force_bounds_test 与 nlcsnn/force_bounds_impl_10ms.m 逐行相同。

if nargin < 1 || isempty(caseName), caseName = 'fishhook'; end
p = inputParser;
p.addParameter('TempC', 30);          % 油温 [degC]，与 fwd_mf_impl 注释示例一致
p.addParameter('NtrialList', [10 5 3 2 1]);
p.addParameter('MakePlot', true);
p.addParameter('TraceEvery', 40);      % 每多少个 10ms tick 记录一次逐步力轨迹
p.parse(varargin{:});
opt = p.Results;

here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, 'results');
if ~exist(outDir, 'dir'), mkdir(outDir); end
addpath(fullfile(here, '..', 'nlcsnn'));
resRoot = fullfile(here, '..', 'PMPC_Results');

switch lower(caseName)
    case 'fishhook'
        sigFile = fullfile(resRoot, 'FishHook_80_mu0.9_cargo170_newplant', 'fh_c3_c_sig.mat');
        logFile = fullfile(resRoot, 'FishHook_80_mu0.9_cargo170_newplant', 'fh_c3_c.mat');
        tag = 'FishHook_c3c';
    case 'dlc'
        sigFile = fullfile(resRoot, 'DLC_mu0.4_interp', 'v6_c3c_70_sig.mat');
        logFile = fullfile(resRoot, 'DLC_mu0.4_interp', 'v6_c3c_70.mat');
        tag = 'DLC_c3c_70';
    otherwise
        error('未知工况 %s', caseName);
end

fprintf('=== 试算步数实验：%s ===\n', tag);

%% 1. 读真实运动（100 Hz）与电流（1 kHz）
S = load(sigFile);
names = flattenNames(S.names);
col = @(nm) find(strcmp(names, nm), 1);
if isempty(col('CmpD_L1'))
    error('通道名不匹配，实际前 6 个：%s', strjoin(names(1:min(6,numel(names))), ', '));
end

t100 = double(S.t(:));
% 列顺序 [L1 L2 R1 R2]
x100 = [double(S.data(:, col('CmpD_L1'))), double(S.data(:, col('CmpD_L2'))), ...
        double(S.data(:, col('CmpD_R1'))), double(S.data(:, col('CmpD_R2')))];
v100 = [double(S.data(:, col('CmpRD_L1'))), double(S.data(:, col('CmpRD_L2'))), ...
        double(S.data(:, col('CmpRD_R1'))), double(S.data(:, col('CmpRD_R2')))];

L = load(logFile);
Icmd = L.Icmd;
if isstruct(Icmd) && isfield(Icmd, 'signals')
    t1k = double(Icmd.time(:));
    I1k = double(Icmd.signals.values);
else
    t1k = double(L.t(:));
    I1k = zeros(numel(t1k), 4);
end
if size(I1k, 2) ~= 4
    error('Icmd 应有 4 列，实际 %d', size(I1k, 2));
end

% 运动插值到 1 kHz
x1k = zeros(numel(t1k), 4);
v1k = zeros(numel(t1k), 4);
for c = 1:4
    x1k(:, c) = interp1(t100, x100(:, c), t1k, 'linear', 'extrap');
    v1k(:, c) = interp1(t100, v100(:, c), t1k, 'linear', 'extrap');
end

Nk = numel(t1k);
fprintf('样本 %d（1 kHz, %.2f s），四角 CmpD %.1f..%.1f mm，CmpRD %.0f..%.0f mm/s\n', ...
    Nk, t1k(end), min(x1k(:)), max(x1k(:)), min(v1k(:)), max(v1k(:)));
fprintf('油温 %.1f degC，preview_alpha=1，bounds_keep=1\n', opt.TempC);

%% 2. 逐个试算步数重放
NP = 10;
nVar = numel(opt.NtrialList);
res = struct('ntrial', {}, 'tick', {}, 'Flo', {}, 'Fhi', {}, 'F0', {}, 'F16', {});
traceList = {};   % 逐步力轨迹（仅 iv=1 记录）

for iv = 1:nVar
    nt = opt.NtrialList(iv);
    clear force_bounds_test;                     % 清 persistent，从零开始
    nTick = floor(Nk / 10);
    tickT = zeros(nTick, 1);
    FloAll = zeros(nTick, 4, NP+1);
    FhiAll = zeros(nTick, 4, NP+1);
    F0All = zeros(nTick, 4);
    F16All = zeros(nTick, 4);
    it = 0;
    tic;
    for k = 1:Nk
        isSolve = (mod(k-1, 10) == 0);
        if isSolve
            it = it + 1;
            tickT(it) = t1k(k);
        end
        for cid = 1:4
            % 预瞄速度：用真实未来速度近似 pmpc_preview 的时域
            idx = min(Nk, k + (1:NP)*100);
            vPrev = v1k(idx, cid);
            wantTrace = (iv == 1) && isSolve && (mod(it-1, opt.TraceEvery) == 0);
            if wantTrace
                [Flo, Fhi, F0, F16, ~, tr] = force_bounds_test( ...
                    t1k(k), opt.TempC, x1k(k, cid), v1k(k, cid), ...
                    I1k(k, cid), cid, vPrev, 1, 1.0, nt);
                traceList{end+1} = tr; %#ok<SAGROW>
            else
                [Flo, Fhi, F0, F16] = force_bounds_test( ...
                    t1k(k), opt.TempC, x1k(k, cid), v1k(k, cid), ...
                    I1k(k, cid), cid, vPrev, 1, 1.0, nt);
            end
            if isSolve
                FloAll(it, cid, :) = Flo;
                FhiAll(it, cid, :) = Fhi;
                F0All(it, cid) = F0;
                F16All(it, cid) = F16;
            end
        end
        if mod(k, 1000) == 0
            fprintf('  ntrial=%2d  %4.1f%%  (%.1f s)\n', nt, 100*k/Nk, toc);
        end
    end
    res(iv).ntrial = nt;
    res(iv).tick = tickT(1:it);
    res(iv).Flo = FloAll(1:it, :, :);
    res(iv).Fhi = FhiAll(1:it, :, :);
    res(iv).F0 = F0All(1:it, :);
    res(iv).F16 = F16All(1:it, :);
    fprintf('  ntrial=%2d 完成，%d 个 10ms tick，用时 %.1f s\n', nt, it, toc);
end

%% 3. 统计
ref = res(1);
span = ref.Fhi - ref.Flo;                 % 各点力界区间宽度
fprintf('\n参考区间宽度：中位 %.0f N，p95 %.0f N\n', ...
    median(span(:)), prctile(span(:), 95));

fprintf('\n%-8s %10s %10s %10s %10s %10s\n', ...
    'ntrial', 'max|d|', 'p99|d|', 'p95|d|', 'mean|d|', 'p99/span');
fprintf('%s\n', repmat('-', 1, 62));
stats = struct();
for iv = 1:nVar
    dFlo = abs(res(iv).Flo - ref.Flo);
    dFhi = abs(res(iv).Fhi - ref.Fhi);
    d = [dFlo(:); dFhi(:)];
    drel = d ./ max([span(:); span(:)], 1);
    stats(iv).ntrial = res(iv).ntrial;
    stats(iv).max = max(d);
    stats(iv).p99 = prctile(d, 99);
    stats(iv).p95 = prctile(d, 95);
    stats(iv).mean = mean(d);
    stats(iv).p99rel = prctile(drel, 99);
    fprintf('%-8d %10.2f %10.2f %10.2f %10.2f %10.4f\n', ...
        res(iv).ntrial, stats(iv).max, stats(iv).p99, stats(iv).p95, ...
        stats(iv).mean, stats(iv).p99rel);
end

% 端点误差（F_at0 / F_at16 直接影响逆模型插值）
fprintf('\n端点 F_at0/F_at16 误差：\n');
fprintf('%-8s %12s %12s\n', 'ntrial', 'max|d|', 'p99|d|');
for iv = 1:nVar
    d = abs([res(iv).F0(:) - ref.F0(:); res(iv).F16(:) - ref.F16(:)]);
    fprintf('%-8d %12.2f %12.2f\n', res(iv).ntrial, max(d), prctile(d, 99));
end

%% 3b. 逐步力收敛分析
if ~isempty(traceList)
    TR = reshape(cell2mat(traceList(:)'), 10, 2, []);   % 10 步 x 2 候选 x N 样本
    Ns = size(TR, 3);
    cum = cumsum(TR, 1) ./ (1:10)';                     % 累积均值
    errCum = zeros(10, 1); p99Cum = zeros(10, 1);
    errStep = zeros(10, 1); p99Step = zeros(10, 1);
    for kk = 1:10
        e = abs(cum(kk, :, :) - cum(10, :, :));
        errCum(kk) = max(e(:)); p99Cum(kk) = prctile(e(:), 99);
        f = abs(TR(kk, :, :) - TR(10, :, :));
        errStep(kk) = max(f(:)); p99Step(kk) = prctile(f(:), 99);
    end
    fprintf('\n逐步力收敛（%d 个样本 x 2 候选）：\n', Ns);
    fprintf('%4s %14s %14s %14s %14s\n', 'k', '|F_k-F_10|max', '|F_k-F_10|p99', '|mean_k-mean_10|max', '|mean_k-mean_10|p99');
    for kk = 1:10
        fprintf('%4d %14.1f %14.1f %14.1f %14.1f\n', kk, ...
            errStep(kk), p99Step(kk), errCum(kk), p99Cum(kk));
    end
    save(fullfile(here, 'results', sprintf('%s_traces.mat', tag)), 'TR');
end

%% 4. 保存
save(fullfile(outDir, sprintf('%s_trial_steps.mat', tag)), 'res', 'stats', 'opt', 'tag');
fprintf('\n已保存 %s\n', fullfile(outDir, sprintf('%s_trial_steps.mat', tag)));

%% 5. 出图
if opt.MakePlot
    plotResults(res, stats, tag, outDir);
end
end

function plotResults(res, stats, tag, outDir)
nt = [stats.ntrial];
p99 = [stats.p99];
mx = [stats.max];

fig = figure('Color', 'w', 'Position', [100 100 900 380]);

subplot(1,2,1);
semilogy(nt, max(p99, 0.1), 'o-', 'LineWidth', 1.6, 'MarkerSize', 7); hold on;
semilogy(nt, max(mx, 0.1), 's--', 'LineWidth', 1.4, 'MarkerSize', 7);
yline(5, 'k:', 'LineWidth', 1.2);
grid on; box off;
ntS = sort(double(nt(:)));
set(gca, 'XDir', 'reverse');
set(gca, 'XTick', ntS, 'XTickLabel', arrayfun(@(v) sprintf('%d', v), ntS, 'UniformOutput', false));
xlabel('试算步数 n_{trial}');
ylabel('力界误差 [N]');
legend({'p99', 'max', '5 N 参考线'}, 'Location', 'northwest');
title('(a) 力界误差 vs 试算步数');

subplot(1,2,2);
ref = res(1);
d = abs(res(end).Flo(:) - ref.Flo(:));
histogram(d, 0:2:120, 'FaceColor', [0.3 0.5 0.8]);
grid on; box off;
xlabel('|F_{lo} - F_{lo,ref}| [N]');
ylabel('样本数');
title(sprintf('(b) n_{trial}=%d 的误差分布', res(end).ntrial));

sgtitle(sprintf('%s：试算步数对力界的影响', strrep(tag, '_', '\_')), 'FontWeight', 'bold');

exportgraphics(fig, fullfile(outDir, sprintf('%s_trial_steps.pdf', tag)), 'ContentType', 'vector');
exportgraphics(fig, fullfile(outDir, sprintf('%s_trial_steps.png', tag)), 'Resolution', 300);
savefig(fig, fullfile(outDir, sprintf('%s_trial_steps.fig', tag)));
close(fig);
fprintf('已出图 %s\n', fullfile(outDir, sprintf('%s_trial_steps.pdf', tag)));
end

function names = flattenNames(raw)
% 兼容 scipy 存出来的 1xN cell-of-cell / char matrix / string 数组
if isstring(raw)
    names = raw(:);
elseif ischar(raw)
    names = string(cellstr(raw));
elseif iscell(raw)
    names = strings(numel(raw), 1);
    for i = 1:numel(raw)
        c = raw{i};
        while iscell(c) && numel(c) == 1, c = c{1}; end
        names(i) = string(c);
    end
else
    error('names 类型不支持：%s', class(raw));
end
names = strtrim(names);
end
