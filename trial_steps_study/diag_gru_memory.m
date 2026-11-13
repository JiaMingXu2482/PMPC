function diag_gru_memory(tag)
%DIAG_GRU_MEMORY  GRU 隐藏状态的时间常数，决定 LUT 是否可能成立。
%   从真实 h0 出发，冻结输入继续迭代，看 |dh| 与力的漂移。
%   若 |dh| 在 10 步内衰减到 1e-3 以下，则 h0 ≈ 瞬时函数，LUT 可行。

if nargin < 1, tag = 'FishHook_c3c'; end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'nlcsnn'));
D = load(fullfile(here,'results',sprintf('%s_lut_feasibility.mat', tag)));
XN = D.XN; VN = D.VN; AN = D.AN; IP = D.IP; Fref = D.Fref; H0 = D.H0;
netF = fwd_weights_m(); n = netF.norm;
Tnorm = (30 - n.t_ref)/n.t_scale;

nTick = size(XN,1);
sel = round(linspace(1, nTick, 60));      % 60 个代表样本
K = 500;
ks = [1 2 5 10 20 50 100 200 500];

dh = zeros(numel(sel)*4, numel(ks));
dF = zeros(numel(sel)*4, numel(ks));
Fend = zeros(numel(sel)*4, 1);
F0all = zeros(numel(sel)*4, 1);
ii = 0;
for t = sel
    for cid = 1:4
        ii = ii + 1;
        h = squeeze(H0(t,cid,:,:));
        prev_I = IP(t,cid);
        hPrev = h;
        [F0, ~] = cdc_fwd.step(netF, XN(t,cid), VN(t,cid), AN(t,cid), ...
            Tnorm, IP(t,cid), h, prev_I);
        F0all(ii) = F0 * n.f_scale;
        for k = 1:K
            [~, h, prev_I] = cdc_fwd.step(netF, XN(t,cid), VN(t,cid), AN(t,cid), ...
                Tnorm, IP(t,cid), h, prev_I);
            if any(k == ks)
                j = find(k == ks, 1);
                dh(ii,j) = max(abs(h(:) - hPrev(:)));
                [Fk, ~] = cdc_fwd.step(netF, XN(t,cid), VN(t,cid), AN(t,cid), ...
                    Tnorm, IP(t,cid), h, prev_I);
                dF(ii,j) = abs(Fk - F0) * n.f_scale;
            end
            hPrev = h;
        end
        [Fend(ii), ~] = cdc_fwd.step(netF, XN(t,cid), VN(t,cid), AN(t,cid), ...
            Tnorm, IP(t,cid), h, prev_I);
        Fend(ii) = Fend(ii) * n.f_scale;
    end
end

fprintf('=== GRU 记忆诊断：%d 个样本 ===\n\n', ii);
fprintf('%6s %14s %14s\n', 'k', '|dh| p99', '|F-F0| p99 [N]');
fprintf('%s\n', repmat('-',1,38));
for j = 1:numel(ks)
    fprintf('%6d %14.5f %14.1f\n', ks(j), prctile(dh(:,j),99), prctile(dF(:,j),99));
end

fprintf('\n力范围对比：\n');
fprintf('  实际轨迹 F_ref   %8.1f .. %8.1f N\n', min(Fref(:)), max(Fref(:)));
fprintf('  从 h0 出发 F_0    %8.1f .. %8.1f N\n', min(F0all), max(F0all));
fprintf('  迭代 500 步 F_end %8.1f .. %8.1f N\n', min(Fend), max(Fend));
fprintf('  |F_end - F_0|     p99 %.1f N，max %.1f N\n', ...
    prctile(abs(Fend-F0all),99), max(abs(Fend-F0all)));

% 结论
k99 = prctile(dh, 99);
k10 = k99(min(find(ks>=10,1), numel(ks)));
if k10 < 1e-3
    fprintf('\n→ 10 步内 h 收敛，LUT 可行\n');
else
    fprintf('\n→ 10 步内 |dh| p99 = %.4f，未收敛；GRU 时间常数 > 10 ms\n', k10);
    fprintf('  h0 带历史，力界不是瞬时输入的函数，LUT 不成立\n');
end
end
