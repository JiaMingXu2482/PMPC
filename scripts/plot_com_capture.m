function plot_com_capture(captureDir)
%PLOT_COM_CAPTURE Reusable Vx and beta-r comparison for a COM capture.
startup_pmpc();
models = {'mpc_mil', 'zeng_mil', 'pmpc_mil'};
labels = {'MPC', 'ZENG', 'PMPC'};
colors = [0.12 0.42 0.86; 0.88 0.34 0.25; 0.18 0.62 0.42];
S = cell(1, numel(models));
for j = 1:numel(models)
    path = fullfile(captureDir, 'Results', ...
        ['Run_COM_capture_' labels{j}], 'vehicle_signals.mat');
    loaded = load(path, 'signals');
    S{j} = loaded.signals;
end

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 960 540]);
hold on;
for j = 1:numel(models)
    plot(S{j}.station_m, S{j}.vx_kmh, 'LineWidth', 1.6, ...
        'Color', colors(j,:), 'DisplayName', labels{j});
end
xlabel('Station (m)'); ylabel('V_x (km/h)');
title('COM: longitudinal speed'); grid on; legend('Location', 'best');
exportgraphics(f, fullfile(captureDir, 'COM_Vx.png'), 'Resolution', 300);
savefig(f, fullfile(captureDir, 'COM_Vx.fig'));
close(f);

% Separate the two maneuvers so the phase-plane comparison remains legible.
for segment = 1:2
    f = figure('Visible', 'off', 'Color', 'w', ...
        'Position', [100 100 800 650]);
    hold on;
    for j = 1:numel(models)
        prefix = fullfile(captureDir, 'Results', ...
            ['Run_COM_capture_' labels{j}], 'LastRun');
        D = func_ReadERD(prefix);
        mv = func_ManeuverFromRun(D.Dataset, D.RunAllPar);
        C = func_CombinedCourse(mv.R, mv.turn_x);
        if segment == 1
            mask = D.Xo <= C.dlc_end_x;
        else
            mask = D.Xo >= C.turn_x;
        end
        plot(D.Beta(mask), D.AVz(mask), 'LineWidth', 1.4, ...
            'Color', colors(j,:), 'DisplayName', labels{j});
    end
    if segment == 1, segmentName = 'DLC'; else, segmentName = 'Jturn'; end
    xlabel('\beta (deg)'); ylabel('r (deg/s)');
    title(['COM: ' segmentName ' sideslip-yaw-rate phase plane']);
    grid on; legend('Location', 'best');
    exportgraphics(f, fullfile(captureDir, ...
        ['COM_beta_r_' segmentName '.png']), 'Resolution', 300);
    savefig(f, fullfile(captureDir, ['COM_beta_r_' segmentName '.fig']));
    close(f);
end

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 800 650]);
hold on;
for j = 1:numel(models)
    plot(S{j}.beta_deg, S{j}.yaw_rate_deg_s, 'LineWidth', 1.4, ...
        'Color', colors(j,:), 'DisplayName', labels{j});
end
xlabel('\beta (deg)'); ylabel('r (deg/s)');
title('COM: sideslip-yaw-rate phase plane');
grid on; legend('Location', 'best');
exportgraphics(f, fullfile(captureDir, 'COM_beta_r.png'), 'Resolution', 300);
savefig(f, fullfile(captureDir, 'COM_beta_r.fig'));
close(f);
end
