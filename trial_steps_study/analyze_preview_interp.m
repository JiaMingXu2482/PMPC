function analyze_preview_interp(caseName)
%ANALYZE_PREVIEW_INTERP  预瞄点降采样对 Flo/Fhi 的影响（不需要重跑 NN）。
%
%   读 run_trial_steps_study 存下的 res（ntrial=10 的精确 11 列力界），
%   用真实未来速度重建每拍的 cmpRD_preview，再比较：
%     (a) 只在部分预瞄点求值 + 在速度上线性插值
%     (b) 只在速度包络两端求值 + 保守包络（不外推）
%   误差以 N 计，参考是 11 点全算。

if nargin < 1, caseName = 'fishhook'; end
here = fileparts(mfilename('fullpath'));
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

R = load(fullfile(here, 'results', sprintf('%s_trial_steps.mat', tag)));
ref = R.res(1);                       % ntrial=10
NP = size(ref.Flo, 3) - 1;            % 10

%% 重建预瞄速度
S = load(sigFile);
names = flattenNames(S.names);
col = @(nm) find(strcmp(names, nm), 1);
t100 = double(S.t(:));
v100 = [double(S.data(:, col('CmpRD_L1'))), double(S.data(:, col('CmpRD_L2'))), ...
        double(S.data(:, col('CmpRD_R1'))), double(S.data(:, col('CmpRD_R2')))];

L = load(logFile);
t1k = double(L.Icmd.time(:));
Nk = numel(t1k);
v1k = zeros(Nk, 4);
for c = 1:4
    v1k(:, c) = interp1(t100, v100(:, c), t1k, 'linear', 'extrap');
end

nTick = size(ref.Flo, 1);
V = zeros(nTick, 4, NP+1);            % 每拍每角 11 个速度（第 1 列=当前）
for it = 1:nTick
    k = (it-1)*10 + 1;
    idx = min(Nk, k + (1:NP)*100);
    for c = 1:4
        V(it, c, 1) = v1k(k, c);
        V(it, c, 2:end) = v1k(idx, c);
    end
end

%% 各种降采样方案
schemes = { ...
    'keep 1,11 (2点)',      [1 11]; ...
    'keep 1,6,11 (3点)',    [1 6 11]; ...
    'keep 1,4,8,11 (4点)',  [1 4 8 11]; ...
    'keep 1,3,6,9,11 (5点)',[1 3 6 9 11]; ...
    'keep 1,2,4,6,8,10,11', [1 2 4 6 8 10 11]};

fprintf('=== 预瞄点降采样：%s（参考 = 11 点全算）===\n', tag);
fprintf('每拍每角速度跨度：中位 %.0f mm/s，p95 %.0f mm/s\n\n', ...
    median(range(V, 3), 'all'), prctile(range(V, 3), 95, 'all'));

fprintf('%-24s %6s %10s %10s %10s %10s\n', ...
    '方案', '点数', 'max|d|', 'p99|d|', 'p95|d|', 'p99/span');
fprintf('%s\n', repmat('-', 1, 76));

spanAll = ref.Fhi - ref.Flo;
ires = struct('name', {}, 'nPts', {}, 'max', {}, 'p99', {}, 'p95', {});
for s = 1:size(schemes, 1)
    keep = schemes{s, 2};
    [FloI, FhiI] = interpBounds(V, ref.Flo, ref.Fhi, keep);
    d = abs([FloI(:) - ref.Flo(:); FhiI(:) - ref.Fhi(:)]);
    drel = d ./ max([spanAll(:); spanAll(:)], 1);
    ires(s).name = schemes{s,1};
    ires(s).nPts = numel(keep);
    ires(s).max  = max(d);
    ires(s).p99  = prctile(d,99);
    ires(s).p95  = prctile(d,95);
    fprintf('%-24s %6d %10.1f %10.1f %10.1f %10.4f\n', ...
        schemes{s,1}, numel(keep), max(d), prctile(d,99), prctile(d,95), prctile(drel,99));
end

% 保守包络：只算当前 + 速度最大/最小两个预瞄点，取外包络
[FloE, FhiE] = envelopeBounds(V, ref.Flo, ref.Fhi);
d = abs([FloE(:) - ref.Flo(:); FhiE(:) - ref.Fhi(:)]);
drel = d ./ max([spanAll(:); spanAll(:)], 1);
narrow = sum(FloE(:) > ref.Flo(:) + 1e-6) + sum(FhiE(:) < ref.Fhi(:) - 1e-6);
fprintf('%-24s %6d %10.1f %10.1f %10.1f %10.4f  (变窄 %d/%d 点)\n', ...
    '保守包络(当前+极值)', 3, max(d), prctile(d,99), prctile(d,95), prctile(drel,99), ...
    narrow, numel(FloE));

% 为什么插值不行：预瞄速度过零导致力界跳变（交集逻辑）
vCur = V(:, :, 1);
vPre = V(:, :, 2:end);
cross = sign(vCur) ~= sign(vPre);
crossAny = any(cross, 3);
fprintf('\n预瞄速度符号与当前值不一致的拍数：%d / %d (%.1f%%)\n', ...
    sum(crossAny(:)), numel(crossAny), 100*sum(crossAny(:))/numel(crossAny));
fprintf('预瞄区间内速度变号的拍数：%d / %d (%.1f%%)\n', ...
    sum(min(vPre,[],3) < 0 & max(vPre,[],3) > 0, 'all'), numel(crossAny), ...
    100*sum(min(vPre,[],3) < 0 & max(vPre,[],3) > 0, 'all')/numel(crossAny));

save(fullfile(here,'results',sprintf('%s_preview_interp.mat', tag)), 'ires', 'tag');
make_figures(tag, fullfile(here,'results'));
end

function [FloI, FhiI] = interpBounds(V, Flo, Fhi, keep)
% 在 keep 指定的列上求值，其余列按速度线性插值
[nT, nC, nP] = size(Flo);
FloI = zeros(size(Flo)); FhiI = zeros(size(Fhi));
for it = 1:nT
    for c = 1:nC
        v = squeeze(V(it, c, :));          % nP x 1
        vk = v(keep);
        fLo = squeeze(Flo(it, c, keep));
        fHi = squeeze(Fhi(it, c, keep));
        % 重复速度合并（取均值），interp1 要求 X 唯一
        [vs, ~, ic] = unique(vk);
        loU = accumarray(ic, fLo, [], @mean);
        hiU = accumarray(ic, fHi, [], @mean);
        for j = 1:nP
            if any(keep == j)
                FloI(it, c, j) = Flo(it, c, j);
                FhiI(it, c, j) = Fhi(it, c, j);
            elseif numel(vs) < 2
                FloI(it, c, j) = loU(1);
                FhiI(it, c, j) = hiU(1);
            else
                vq = min(max(v(j), vs(1)), vs(end));   % 不外推，夹到端点
                FloI(it, c, j) = interp1(vs, loU, vq, 'linear');
                FhiI(it, c, j) = interp1(vs, hiU, vq, 'linear');
            end
        end
    end
end
end

function [FloE, FhiE] = envelopeBounds(V, Flo, Fhi)
% 只算当前列 + 速度最大/最小两列，其余列取这三列的外包络
[nT, nC, nP] = size(Flo);
FloE = zeros(size(Flo)); FhiE = zeros(size(Fhi));
for it = 1:nT
    for c = 1:nC
        v = squeeze(V(it, c, :));
        [~, iMin] = min(v(2:end)); iMin = iMin + 1;
        [~, iMax] = max(v(2:end)); iMax = iMax + 1;
        idx = unique([1 iMin iMax]);
        lo = min(Flo(it, c, idx)); hi = max(Fhi(it, c, idx));
        FloE(it, c, :) = lo;
        FhiE(it, c, :) = hi;
    end
end
end

function names = flattenNames(raw)
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
