function metrics = plot_com_diamond(captureDir)
%PLOT_COM_DIAMOND Compare four COM metrics as two normalized diamonds.
% One is best within each maneuver; all four axes point outward for better.
startup_pmpc();
models = {'MPC','ZENG','PMPC'};
segments = {'DLC','J-turn'};
colors = [0.12 0.42 0.86; 0.88 0.34 0.25; 0.18 0.62 0.42];
raw = nan(3, 4, 2); % [ey max, Vx min, ey RMS, Vx RMS]

for j = 1:numel(models)
    prefix = fullfile(captureDir, 'Results', ...
        ['Run_COM_capture_' models{j}], 'LastRun');
    D = func_ReadERD(prefix);
    mv = func_ManeuverFromRun(D.Dataset, D.RunAllPar);
    C = func_CombinedCourse(mv.R, mv.turn_x);
    masks = {D.Xo <= C.dlc_end_x, D.Xo >= C.turn_x};
    for s = 1:2
        ds = slice_(D, find(masks{s}));
        assert(ds.N >= 2, 'Insufficient %s data for %s.', ...
            segments{s}, models{j});
        M = func_Metrics(ds);
        raw(j,:,s) = [M.ey_pk, min(ds.Vx), M.ey_rms, ...
            sqrt(mean(ds.Vx.^2))];
    end
end

score = nan(size(raw));
for s = 1:2
    for k = [1 3] % smaller tracking errors are better
        best = min(raw(:,k,s));
        score(:,k,s) = best ./ max(raw(:,k,s), eps);
    end
    for k = [2 4] % larger speeds mean greater speed retention
        best = max(raw(:,k,s));
        score(:,k,s) = raw(:,k,s) ./ max(best, eps);
    end
end
assert(all(isfinite(score(:)) & score(:) >= 0 & score(:) <= 1+1e-10), ...
    'Diamond normalization failed.');

f = figure('Visible','off','Color','w','Position',[100 100 1500 720]);
tiledlayout(f, 1, 2, 'TileSpacing','compact','Padding','compact');
axisNames = {'e_y max','V_x min','e_y RMS','V_x RMS'};
direction = [0 1; 1 0; 0 -1; -1 0];
for s = 1:2
    ax = nexttile; hold(ax,'on');
    for radius = [0.25 0.50 0.75 1]
        xy = radius * direction([1:4 1],:);
        plot(ax, xy(:,1), xy(:,2), '-', 'Color',[0.83 0.83 0.83], ...
            'LineWidth',0.8,'HandleVisibility','off');
    end
    plot(ax, [-1 1], [0 0], '-', 'Color',[0.55 0.55 0.55], ...
        'HandleVisibility','off');
    plot(ax, [0 0], [-1 1], '-', 'Color',[0.55 0.55 0.55], ...
        'HandleVisibility','off');
    lines = gobjects(1,3);
    for j = 1:3
        xy = direction .* reshape(score(j,:,s),[],1);
        closed = xy([1:4 1],:);
        patch(ax, closed(:,1), closed(:,2), colors(j,:), ...
            'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
        lines(j) = plot(ax, closed(:,1), closed(:,2), '-o', ...
            'Color',colors(j,:), 'MarkerFaceColor',colors(j,:), ...
            'MarkerSize',5, 'LineWidth',1.8, 'DisplayName',models{j});
    end
    text(ax, 0, 1.12, axisNames{1}, 'HorizontalAlignment','center');
    text(ax, 1.11, 0, axisNames{2}, 'HorizontalAlignment','left');
    text(ax, 0, -1.12, axisNames{3}, 'HorizontalAlignment','center');
    text(ax, -1.11, 0, axisNames{4}, 'HorizontalAlignment','right');
    text(ax, 0.04, 0.78, '1.0 = best', 'FontSize',9, ...
        'Color',[0.35 0.35 0.35]);
    title(ax, segments{s});
    axis(ax,'equal'); axis(ax,'off');
    xlim(ax,[-1.38 1.38]); ylim(ax,[-1.25 1.25]);
    if s == 2
        legend(ax, lines, models, 'Location','northeastoutside');
    end
end
sgtitle(f, 'COM performance: normalized to the best controller in each segment');
exportgraphics(f, fullfile(captureDir,'COM_diamond.png'), 'Resolution',300);
savefig(f, fullfile(captureDir,'COM_diamond.fig'));
close(f);

controller = strings(6,1); segment = strings(6,1);
values = zeros(6,8); row = 0;
for s = 1:2
    for j = 1:3
        row = row + 1;
        controller(row) = models{j}; segment(row) = segments{s};
        values(row,:) = [raw(j,:,s), score(j,:,s)];
    end
end
metrics = table(controller, segment, values(:,1), values(:,2), ...
    values(:,3), values(:,4), values(:,5), values(:,6), ...
    values(:,7), values(:,8), 'VariableNames', ...
    {'Controller','Segment','EyMax_m','VxMin_kmh','EyRms_m', ...
     'VxRms_kmh','EyMaxScore','VxMinScore','EyRmsScore','VxRmsScore'});
writetable(metrics, fullfile(captureDir,'COM_diamond_metrics.csv'));
save(fullfile(captureDir,'COM_diamond_metrics.mat'), ...
    'metrics','raw','score','models','segments');
end

function E = slice_(D, indices)
E = D;
fields = fieldnames(D);
for k = 1:numel(fields)
    v = D.(fields{k});
    if isnumeric(v) && isvector(v) && numel(v) == D.N
        E.(fields{k}) = v(indices);
    end
end
E.N = numel(indices);
end
