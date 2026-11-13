function test_lut_feasibility(caseName)
%TEST_LUT_FEASIBILITY  查表方案是否成立的关键判定。
%
%   问题：force_bounds_impl_10ms 的试算从持久 GRU 状态 h0 出发。
%   h0 是 1 kHz 演化的隐藏状态，带有历史。若 h0 不能用瞬时输入
%   (x,v,a,T,I_prev) 的稳态值代替，则力界不是瞬时量的函数，
%   LUT 根本不成立。本脚本直接测这个替代误差。
%
%   同时给出各输入的量程和敏感度，供 LUT 网格设计。

if nargin < 1, caseName = 'fishhook'; end
here = fileparts(mfilename('fullpath'));
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

TempC = 30;
netF = fwd_weights_m();
n = netF.norm;

%% 1. 重放，收集每拍的 (x,v,a,T,I_prev,h0,prev0)
S = load(sigFile);
names = flattenNames(S.names);
col = @(nm) find(strcmp(names, nm), 1);
t100 = double(S.t(:));
x100 = [double(S.data(:, col('CmpD_L1'))), double(S.data(:, col('CmpD_L2'))), ...
        double(S.data(:, col('CmpD_R1'))), double(S.data(:, col('CmpD_R2')))];
v100 = [double(S.data(:, col('CmpRD_L1'))), double(S.data(:, col('CmpRD_L2'))), ...
        double(S.data(:, col('CmpRD_R1'))), double(S.data(:, col('CmpRD_R2')))];
L = load(logFile);
t1k = double(L.Icmd.time(:));
I1k = double(L.Icmd.signals.values);
Nk = numel(t1k);
x1k = zeros(Nk,4); v1k = zeros(Nk,4);
for c = 1:4
    x1k(:,c) = interp1(t100, x100(:,c), t1k, 'linear', 'extrap');
    v1k(:,c) = interp1(t100, v100(:,c), t1k, 'linear', 'extrap');
end

NP = 10;
clear force_bounds_test;
nTick = floor(Nk/10);
H0 = zeros(nTick,4,2,8);   % 每拍每角的 h0
P0 = zeros(nTick,4);       % prev0
XN = zeros(nTick,4); VN = zeros(nTick,4); AN = zeros(nTick,4);
IP = zeros(nTick,4);       % I_prev (归一化)
Fref = zeros(nTick,4,2);   % [F_at0, F_at16] 参考（用真实 h0）
it = 0;
fprintf('重放收集 h0 ...\n');
for k = 1:Nk
    isSolve = (mod(k-1,10) == 0);
    if isSolve, it = it + 1; end
    for cid = 1:4
        idx = min(Nk, k + (1:NP)*100);
        [~,~,F0,F16,dbg] = force_bounds_test(t1k(k), TempC, x1k(k,cid), ...
            v1k(k,cid), I1k(k,cid), cid, v1k(idx,cid), 1, 1.0, 10);
        if isSolve
            H0(it,cid,:,:) = dbg.h0;
            P0(it,cid) = dbg.prev0;
            XN(it,cid) = dbg.x_norm; VN(it,cid) = dbg.v_norm;
            AN(it,cid) = dbg.a_norm;
            IP(it,cid) = dbg.prev0;
            Fref(it,cid,1) = F0; Fref(it,cid,2) = F16;
        end
    end
end
fprintf('收集 %d 个样本\n', it*4);

%% 2. 输入量程
fprintf('\n输入量程（归一化）：\n');
fprintf('  x_norm  %8.3f .. %8.3f\n', min(XN(:)), max(XN(:)));
fprintf('  v_norm  %8.3f .. %8.3f\n', min(VN(:)), max(VN(:)));
fprintf('  a_norm  %8.3f .. %8.3f\n', min(AN(:)), max(AN(:)));
fprintf('  I_prev  %8.3f .. %8.3f\n', min(IP(:)), max(IP(:)));
fprintf('  力      %8.1f .. %8.1f N\n', min(Fref(:)), max(Fref(:)));

%% 3. 关键判定：h0 vs 稳态 h_ss
fprintf('\n=== 判定 1：h0 能否用稳态代替 ===\n');
Fss = zeros(nTick,4,2);
NITER = 300;
tic;
for t = 1:nTick
    for cid = 1:4
        [hss, ~] = cdc_fwd.converge(netF, XN(t,cid), VN(t,cid), AN(t,cid), ...
            (TempC - n.t_ref)/n.t_scale, IP(t,cid), zeros(2,8), IP(t,cid), NITER);
        Fss(t,cid,1) = cdc_fwd.trialMean(netF, XN(t,cid), VN(t,cid), AN(t,cid), ...
            (TempC - n.t_ref)/n.t_scale, 0, hss, IP(t,cid), 10);
        Fss(t,cid,2) = cdc_fwd.trialMean(netF, XN(t,cid), VN(t,cid), AN(t,cid), ...
            (TempC - n.t_ref)/n.t_scale, 1, hss, IP(t,cid), 10);
    end
end
fprintf('  稳态迭代完成（%.1f s）\n', toc);
% 注意符号：force_bounds_test 返回 F_at0 = -F0_rig（车体符号），
% cdc_fwd.trialMean 返回 rig 符号。统一到车体符号再比。
FrefRig = -Fref;          % 车体符号 -> rig
FssRig  =  Fss;           % 已是 rig
d = abs(FssRig(:) - FrefRig(:));
span2 = abs(FrefRig(:,:,2) - FrefRig(:,:,1));
spanAll = [span2(:); span2(:)];
fprintf('  |F_ss - F_h0|：max %.1f N，p99 %.1f N，中位 %.1f N\n', ...
    max(d), prctile(d,99), median(d));
fprintf('  相对端点跨度：p99 %.1f%%\n', 100*prctile(d ./ max(spanAll,1), 99));

% 诊断：h0 是不是不动点？从 h0 出发再迭代 300 步，看漂移
fprintf('\n  诊断：从 h0 出发继续迭代，看是否不动点\n');
drift = zeros(nTick,4);
for t = 1:nTick
    for cid = 1:4
        h0 = squeeze(H0(t,cid,:,:));
        [hEnd, ~] = cdc_fwd.converge(netF, XN(t,cid), VN(t,cid), AN(t,cid), ...
            (TempC - n.t_ref)/n.t_scale, IP(t,cid), h0, IP(t,cid), NITER);
        drift(t,cid) = max(abs(hEnd(:) - h0(:)));
    end
end
fprintf('  |h(300) - h0|：max %.3f，p99 %.3f，中位 %.3f\n', ...
    max(drift(:)), prctile(drift(:),99), median(drift(:)));
fprintf('  （若远大于 0，说明 h0 不是不动点，GRU 带长记忆）\n');
if prctile(d,99) < 20
    fprintf('  → 通过：力界可由瞬时输入决定，LUT 成立\n');
else
    fprintf('  → 不通过：h0 带历史，LUT 不能只查瞬时输入\n');
end

%% 4. 敏感度：各输入维度能不能丢（小扰动，保持 h0 不变，测偏导）
% 用 ±5% 量程的扰动，避免把状态推出流形
fprintf('\n=== 判定 2：各输入维度敏感度（±5%% 量程扰动）===\n');
base = FrefRig;
names4 = {'x_norm', 'v_norm', 'a_norm', 'I_prev'};
spanX = max(XN(:))-min(XN(:)); spanV = max(VN(:))-min(VN(:));
spanA = max(AN(:))-min(AN(:));
delta = 0.05*[spanX spanV spanA 1.0];
for ti = 1:4
    F = zeros(nTick,4,2);
    for t = 1:nTick
        for cid = 1:4
            xa = XN(t,cid); va = VN(t,cid); aa = AN(t,cid); ip = IP(t,cid);
            switch ti
                case 1, xa = xa + delta(1);
                case 2, va = va + delta(2);
                case 3, aa = aa + delta(3);
                case 4, ip = min(1, ip + delta(4));
            end
            hss = squeeze(H0(t,cid,:,:));
            F(t,cid,1) = cdc_fwd.trialMean(netF, xa, va, aa, ...
                (TempC - n.t_ref)/n.t_scale, 0, hss, ip, 10);
            F(t,cid,2) = cdc_fwd.trialMean(netF, xa, va, aa, ...
                (TempC - n.t_ref)/n.t_scale, 1, hss, ip, 10);
        end
    end
    dd = abs(F(:) - base(:));
    fprintf('  %-8s +5%%  p99 %8.1f N   max %8.1f N\n', names4{ti}, ...
        prctile(dd,99), max(dd));
end

save(fullfile(here,'results',sprintf('%s_lut_feasibility.mat', tag)), ...
    'XN','VN','AN','IP','Fref','Fss','H0','P0','tag');
fprintf('\n已保存 %s\n', fullfile(here,'results',sprintf('%s_lut_feasibility.mat', tag)));
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
