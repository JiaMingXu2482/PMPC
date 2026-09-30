function T = analyze_nlcsnn_current_steps(outDir)
%ANALYZE_NLCSNN_CURRENT_STEPS Frozen-state current-step response of NLCSNN.
%   All cases start from the same 0.8 A-conditioned hidden state. Damper
%   kinematics are frozen at x=x_ref, v=+0.2 m/s, a=0 so that only the
%   current-step magnitude and direction differ between cases.

if nargin < 1 || isempty(outDir)
    root = fileparts(fileparts(mfilename('fullpath')));
    outDir = fullfile(root, 'scratchpad', 'nlcsnn_current_steps');
else
    root = fileparts(fileparts(mfilename('fullpath')));
end
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
addpath(fullfile(root, 'nlcsnn'));

net = nlcsnn_damper_init();
dt = 0.001;
tPre = 3.0;
tPost = 1.0;
x = net.norm.x_ref;
v = 200;             % mm/s = 0.2 m/s, rebound-positive convention
a = 0;               % mm/s^2
temp = 42.5;          % degC
i0 = 0.8;             % A, common initial current
dI = [-0.8 -0.4 -0.2 0.2 0.4 0.8];

% Establish one common hidden state at the initial operating point.
h0 = zeros(8,1);
for k = 1:round(tPre/dt)
    [~, h0] = nlcsnn_damper_step(net, x, v, a, i0, 0, temp, dt, h0);
end
F0 = nlcsnn_predict_force(net, x, v, a, i0, temp, h0);

n = round(tPost/dt) + 1;
t = (0:n-1)' * dt;
Fhist = zeros(n, numel(dI));
rows = repmat(struct(), numel(dI), 1);

for c = 1:numel(dI)
    i1 = i0 + dI(c);
    h = h0;  % identical state at the instant of every current step
    for k = 1:n
        [Fhist(k,c), h] = nlcsnn_damper_step( ...
            net, x, v, a, i1, 0, temp, dt, h);
    end

    Fss = mean(Fhist(end-99:end,c));
    total = Fss - F0;
    if abs(total) > 1e-9
        yNorm = (Fhist(:,c) - F0) / total;
        t10 = first_crossing(t, yNorm, 0.10);
        t63 = first_crossing(t, yNorm, 1-exp(-1));
        t90 = first_crossing(t, yNorm, 0.90);
        immediate = (Fhist(1,c) - F0) / total;
        overshoot = max(yNorm) - 1;
        undershoot = min(yNorm);
        tol = max(0.02 * abs(total), 0.5);
        settle = settling_time(t, Fhist(:,c), Fss, tol);
    else
        t10 = NaN; t63 = NaN; t90 = NaN; immediate = NaN;
        overshoot = NaN; undershoot = NaN; settle = NaN;
    end

    rows(c).DeltaI_A = dI(c);
    rows(c).SignDeltaI = sign(dI(c));
    rows(c).I0_A = i0;
    rows(c).I1_A = i1;
    rows(c).F0_N = F0;
    rows(c).F_at_0plus_N = Fhist(1,c);
    rows(c).Fss_N = Fss;
    rows(c).DeltaF_N = total;
    rows(c).ImmediateFraction = immediate;
    rows(c).t10_ms = 1e3*t10;
    rows(c).t63_ms = 1e3*t63;
    rows(c).t90_ms = 1e3*t90;
    rows(c).Settling2pct_ms = 1e3*settle;
    rows(c).OvershootFraction = overshoot;
    rows(c).MinNormalizedResponse = undershoot;
end

T = struct2table(rows);
writetable(T, fullfile(outDir, 'nlcsnn_current_step_metrics.csv'));
save(fullfile(outDir, 'nlcsnn_current_step_data.mat'), ...
    'T', 't', 'Fhist', 'F0', 'h0', 'x', 'v', 'a', 'temp', 'i0', 'dI');

make_plot(fullfile(outDir, 'nlcsnn_current_step_response.png'), ...
    T, t, Fhist, F0, dI, v);

disp(T);
fprintf('Saved current-step diagnostics to:\n  %s\n', outDir);
end

function tc = first_crossing(t, y, level)
idx = find(y >= level, 1, 'first');
if isempty(idx)
    tc = NaN;
else
    tc = t(idx);
end
end

function ts = settling_time(t, y, yss, tol)
outside = find(abs(y-yss) > tol, 1, 'last');
if isempty(outside)
    ts = 0;
elseif outside >= numel(t)
    ts = NaN;
else
    ts = t(outside+1);
end
end

function make_plot(outFile, T, t, Fhist, F0, dI, v)
fig = figure('Visible','off','Color','w','Position',[100 100 1280 820]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing','compact', 'Padding','compact');

ax1 = nexttile(tl,1); hold(ax1,'on'); grid(ax1,'on');
for c = find(dI > 0)
    plot(ax1, 1e3*t, Fhist(:,c), 'LineWidth',1.4, ...
        'DisplayName',sprintf('\\DeltaI=+%.1f A',dI(c)));
end
yline(ax1,F0,'k:','F_0');
xlim(ax1,[0 300]); xlabel(ax1,'Time after step (ms)'); ylabel(ax1,'Force (N)');
title(ax1,'Increasing-current steps from 0.8 A'); legend(ax1,'Location','best');

ax2 = nexttile(tl,2); hold(ax2,'on'); grid(ax2,'on');
for c = find(dI < 0)
    plot(ax2, 1e3*t, Fhist(:,c), 'LineWidth',1.4, ...
        'DisplayName',sprintf('\\DeltaI=%.1f A',dI(c)));
end
yline(ax2,F0,'k:','F_0');
xlim(ax2,[0 300]); xlabel(ax2,'Time after step (ms)'); ylabel(ax2,'Force (N)');
title(ax2,'Decreasing-current steps from 0.8 A'); legend(ax2,'Location','best');

ax3 = nexttile(tl,3); hold(ax3,'on'); grid(ax3,'on');
for c = 1:numel(dI)
    yn = (Fhist(:,c)-F0) / T.DeltaF_N(c);
    plot(ax3, 1e3*t, yn, 'LineWidth',1.25, ...
        'DisplayName',sprintf('%+.1f A',dI(c)));
end
yline(ax3,0.1,'k:','10%'); yline(ax3,0.9,'k--','90%');
xlim(ax3,[0 300]); xlabel(ax3,'Time after step (ms)');
ylabel(ax3,'Normalized force response');
title(ax3,'Normalized response (same initial hidden state)');
legend(ax3,'Location','best','NumColumns',2);

ax4 = nexttile(tl,4); hold(ax4,'on'); grid(ax4,'on');
neg = T.DeltaI_A < 0; pos = T.DeltaI_A > 0;
plot(ax4,abs(T.DeltaI_A(neg)),T.ImmediateFraction(neg),'o-','LineWidth',1.4, ...
    'DisplayName','Immediate fraction, \DeltaI<0');
plot(ax4,abs(T.DeltaI_A(pos)),T.ImmediateFraction(pos),'s-','LineWidth',1.4, ...
    'DisplayName','Immediate fraction, \DeltaI>0');
yline(ax4,0.9,'k--','90%');
xlabel(ax4,'|\DeltaI| (A)'); ylabel(ax4,'Fraction completed at t=0+');
title(ax4,'Direct feedthrough versus step magnitude');
legend(ax4,'Location','best');

title(tl,sprintf(['NLCSNN frozen-state current steps: v=%.1f mm/s, ' ...
    'x=x_{ref}, I_0=0.8 A, T=42.5 ^oC'],v));
exportgraphics(fig,outFile,'Resolution',180);
close(fig);
end
