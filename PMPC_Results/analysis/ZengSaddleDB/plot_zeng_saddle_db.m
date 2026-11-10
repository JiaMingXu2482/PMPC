function plot_zeng_saddle_db()
%PLOT_ZENG_SADDLE_DB  可视化 ZengSaddleDB_SUV.mat 的 (mu, V, delta) 三维稳定域数据。
%   plot_zeng_saddle_db
% 产物写入本文件所在目录。数据来源：pmpc_core/ZengSaddleDB_SUV.mat
% 字段：mu_grid(1x5), V_grid_kmh(1x5), delta_grid_deg(1x8), betaL/betaR/ok(5x5x8)

root = fileparts(mfilename('fullpath'));
out  = root;
matf = fullfile(fileparts(root), 'pmpc_core', 'ZengSaddleDB_SUV.mat');
assert(isfile(matf), '找不到 %s', matf);

S  = load(matf);
mu = S.mu_grid(:)';
V  = S.V_grid_kmh(:)';
de = S.delta_grid_deg(:)';
bL = S.betaL;  bR = S.betaR;  ok = logical(S.ok);

[I, J, K] = ndgrid(1:numel(mu), 1:numel(V), 1:numel(de));
sel = ok;
Xv = V(J(sel));  Yv = de(K(sel));  Zv = mu(I(sel));
Lv = bL(sel);    Rv = bR(sel);

%% 图1 三维散点：稳定域鞍点边界
f1 = figure('Color', 'w', 'Position', [80 80 1000 700]);
ax = axes(f1); hold(ax, 'on'); grid(ax, 'on'); view(ax, 3);
cmap = lines(numel(mu));
for i = 1:numel(mu)
    s = (Zv == mu(i));
    if ~any(s), continue; end
    scatter3(ax, Xv(s), Yv(s), Lv(s), 55, cmap(i,:), 'v', 'filled', ...
        'MarkerEdgeColor', 'k', 'DisplayName', sprintf('\\mu=%.1f  \\beta_L', mu(i)));
    scatter3(ax, Xv(s), Yv(s), Rv(s), 55, cmap(i,:), '^', 'filled', ...
        'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
end
xlabel(ax, '车速 V [km/h]'); ylabel(ax, '前轮转角 \delta [deg]');
zlabel(ax, '鞍点边界 \beta [rad]');
title(ax, 'ZengSaddleDB\_SUV：稳定域鞍点边界（\nabla \beta_L 下界 / \Delta \beta_R 上界）');
legend(ax, 'Location', 'northeastoutside');
exportgraphics(f1, fullfile(out, 'fig1_saddle_3d_scatter.png'), 'Resolution', 160);

%% 图2 每个 mu 的 betaL/betaR 曲面
f2 = figure('Color', 'w', 'Position', [60 60 1500 850]);
for i = 1:numel(mu)
    ax = subplot(2, 3, i); hold(ax, 'on'); grid(ax, 'on'); view(ax, 3);
    s = (Zv == mu(i));
    if nnz(s) >= 3
        trisurf(delaunay(Xv(s), Yv(s)), Xv(s), Yv(s), Rv(s), ...
            'FaceColor', [0.85 0.35 0.30], 'FaceAlpha', 0.75, 'EdgeColor', [.4 .4 .4]);
        trisurf(delaunay(Xv(s), Yv(s)), Xv(s), Yv(s), Lv(s), ...
            'FaceColor', [0.25 0.40 0.75], 'FaceAlpha', 0.75, 'EdgeColor', [.4 .4 .4]);
    end
    scatter3(ax, Xv(s), Yv(s), Rv(s), 16, [0.6 0.1 0.1], 'filled');
    scatter3(ax, Xv(s), Yv(s), Lv(s), 16, [0.1 0.1 0.6], 'filled');
    xlabel(ax, 'V [km/h]'); ylabel(ax, '\delta [deg]'); zlabel(ax, '\beta [rad]');
    title(ax, sprintf('\\mu = %.1f   (有效节点 %d/8)', mu(i), nnz(s)));
end
sgtitle(f2, '各附着系数下的稳定域边界曲面（红=\beta_R 上界，蓝=\beta_L 下界）');
exportgraphics(f2, fullfile(out, 'fig2_surfaces_per_mu.png'), 'Resolution', 160);

%% 图3 ok 掩码
f3 = figure('Color', 'w', 'Position', [60 60 1400 750]);
for i = 1:numel(mu)
    ax = subplot(2, 3, i);
    imagesc(ax, de, V, double(ok(i,:,:))); axis(ax, 'xy'); hold(ax, 'on');
    colormap(ax, flipud(gray(2))); clim(ax, [0 1]);
    set(ax, 'XTick', de, 'YTick', V);
    xlabel(ax, '\delta [deg]'); ylabel(ax, 'V [km/h]');
    title(ax, sprintf('\\mu = %.1f   ok=%d/40', mu(i), nnz(ok(i,:,:))));
    for j = 1:numel(V)
        for k = 1:numel(de)
            if ok(i,j,k)
                text(ax, de(k), V(j), sprintf('%.2f', bR(i,j,k)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 6.5, ...
                    'Color', [0.8 0.1 0.1]);
            end
        end
    end
end
sgtitle(f3, '数据库有效节点分布（灰=ok=1，白=ok=0；红字为该节点 \beta_R）');
exportgraphics(f3, fullfile(out, 'fig3_ok_mask.png'), 'Resolution', 160);

%% 图4 delta=+2 deg 切片的稳定域宽度
f4 = figure('Color', 'w', 'Position', [80 80 800 500]);
ax = axes(f4); hold(ax, 'on'); grid(ax, 'on');
k2 = find(de == 2, 1);
for i = 1:numel(mu)
    w = nan(1, numel(V));
    for j = 1:numel(V)
        if ok(i,j,k2), w(j) = bR(i,j,k2) - bL(i,j,k2); end
    end
    plot(ax, V, w, 'o-', 'DisplayName', sprintf('\\mu=%.1f', mu(i)));
end
xlabel(ax, '车速 V [km/h]'); ylabel(ax, '稳定域宽度 \beta_R - \beta_L [rad]');
title(ax, '\delta = +2\circ 切片：稳定域宽度随车速收缩');
legend(ax, 'Location', 'best');
exportgraphics(f4, fullfile(out, 'fig4_width_vs_speed.png'), 'Resolution', 160);

fprintf('已输出 4 张图到 %s\n', out);
end
