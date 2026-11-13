function make_figures(tag, outDir)
%MAKE_FIGURES  汇总三联图：试算步数 / 逐步收敛 / 预瞄点降采样。
%   读 results/<tag>_trial_steps.mat、<tag>_traces.mat、<tag>_preview_interp.mat

here = fileparts(mfilename('fullpath'));
if nargin < 2, outDir = fullfile(here, 'results'); end

S1 = load(fullfile(outDir, sprintf('%s_trial_steps.mat', tag)));
S2 = load(fullfile(outDir, sprintf('%s_traces.mat', tag)));
S3 = load(fullfile(outDir, sprintf('%s_preview_interp.mat', tag)));

stats = S1.stats;
TR = S2.TR;                       % 10 x 2 x N
nS = size(TR, 3);

% 逐步收敛
stepP99 = zeros(10,1); meanP99 = zeros(10,1);
cum = cumsum(TR,1) ./ (1:10)';
for k = 1:10
    f = abs(TR(k,:,:) - TR(10,:,:));  stepP99(k) = prctile(f(:), 99);
    e = abs(cum(k,:,:) - cum(10,:,:)); meanP99(k) = prctile(e(:), 99);
end

fig = figure('Color','w','Position',[80 80 1500 400]);

% (a) 试算步数
subplot(1,3,1);
nt = double([stats.ntrial]);
semilogy(nt, max([stats.p99],0.1), 'o-', 'LineWidth',1.8, 'MarkerSize',8); hold on;
semilogy(nt, max([stats.max],0.1), 's--', 'LineWidth',1.4, 'MarkerSize',7);
yline(5,'k:','LineWidth',1.2);
grid on; box off;
set(gca,'XDir','reverse');
ntS = sort(nt);
set(gca,'XTick',ntS,'XTickLabel',arrayfun(@(v) sprintf('%d',v), ntS,'UniformOutput',false));
xlabel('试算步数 n_{trial}'); ylabel('力界误差 [N]');
legend({'p99','max','5 N'},'Location','northwest');
title('(a) 砍试算步数：不可行');

% (b) 逐步收敛
subplot(1,3,2);
plot(1:10, stepP99, 'o-', 'LineWidth',1.8, 'MarkerSize',7); hold on;
plot(1:10, meanP99, 's--', 'LineWidth',1.4, 'MarkerSize',7);
yline(5,'k:','LineWidth',1.2);
grid on; box off;
xlabel('试算步数 k'); ylabel('相对 10 步的误差 [N]');
legend({'|F_k-F_{10}| p99','|mean_k-mean_{10}| p99','5 N'},'Location','northeast');
title('(b) 10 步未收敛，每步都在变');
xlim([1 10]);

% (c) 预瞄点降采样
subplot(1,3,3);
np = [S3.ires.nPts];
semilogy(np, max([S3.ires.p99],0.1), 'o-', 'LineWidth',1.8, 'MarkerSize',8); hold on;
semilogy(np, max([S3.ires.max],0.1), 's--', 'LineWidth',1.4, 'MarkerSize',7);
yline(5,'k:','LineWidth',1.2);
grid on; box off;
xlabel('预瞄点个数'); ylabel('力界误差 [N]');
legend({'p99','max','5 N'},'Location','northeast');
title('(c) 砍预瞄点：不可行');
xlim([2 11]);

sgtitle(sprintf('%s（%d 个 10ms tick × 4 角）：力界计算量能否压缩', ...
    strrep(tag,'_','\_'), size(S1.res(1).Flo,1)), 'FontWeight','bold');

exportgraphics(fig, fullfile(outDir, sprintf('%s_compute_budget.pdf', tag)), 'ContentType','vector');
exportgraphics(fig, fullfile(outDir, sprintf('%s_compute_budget.png', tag)), 'Resolution',300);
savefig(fig, fullfile(outDir, sprintf('%s_compute_budget.fig', tag)));
close(fig);
fprintf('已出图 %s\n', fullfile(outDir, sprintf('%s_compute_budget.pdf', tag)));
end
