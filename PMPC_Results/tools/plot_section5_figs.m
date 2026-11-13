function plot_section5_figs()
%PLOT_SECTION5_FIGS  Section V figures (15) for the TVT revision.
%   Follows TEX/SECTION_V_MIL_FIGURE_PLAN.md:
%     * MATLAB only; every figure saved as .fig + vector .pdf (+300 dpi .png);
%     * ControllerA/B/C/D colours fixed; legend order A->B->C->D, bounds last;
%     * legends outside the axes (north), so nothing covers the data;
%     * Mz and Md figures carry their own upper/lower bounds;
%     * D1 Y-axis fixed to [-0.5 5] m with an out-of-frame marker.
%
%   Usage:  matlab -batch "cd('<...>/Jturn/PMPC_Results/tools'); plot_section5_figs"

tools = fileparts(mfilename('fullpath'));               % .../PMPC_Results/tools
res   = fileparts(tools);                                % .../PMPC_Results
jturn = fileparts(res);                                  % .../Jturn
out   = fullfile(fileparts(jturn), 'TEX', 'newfigures');
if ~isfolder(out), mkdir(out); end

cfg = config();
cfg.res = res; cfg.out = out;

% ---------------- DLC ----------------
dlcXY(cfg);
dlcBetaR(cfg);
dlcMz(cfg);
dlcVx(cfg);
dlcEy(cfg);
dlcEpsi(cfg);
dlcSw(cfg);
dlcYaw(cfg);
% ---------------- FishHook ----------------
fhLtr(cfg);
fhRoll(cfg);
fhMd(cfg);
fhRollRate(cfg);
fhMz(cfg);
fhVx(cfg);
fhSw(cfg);
% ---------------- Slalom ----------------
slEy(cfg);
slMz(cfg);
slLtr(cfg);
slEpsi(cfg);
slSw(cfg);
slMd(cfg);
slAxle(cfg, 'front');
slAxle(cfg, 'rear');
end

% ====================================================================== cfg
function cfg = config()
c.A = [ 44 160  44]/255;      % #2CA02C
c.B = [ 31 119 180]/255;      % #1F77B4
c.C = [214  39  40]/255;      % #D62728
c.D = [140  86  75]/255;      % #8C564B
c.PAS = [127 127 127]/255;    % #7F7F7F
c.REF = [0 0 0];
c.BND = [0.45 0.45 0.45];

ls.A = '--';  mk.A = 'o';
ls.B = '-.';  mk.B = 's';
ls.C = '-';   mk.C = 'none';
ls.D = ':';   mk.D = '^';
ls.PAS = ':'; mk.PAS = 'none';

cfg.col = c; cfg.ls = ls; cfg.mk = mk;
cfg.nm = struct('A','Controller A','B','Controller B', ...
                'C','Controller C','D','Controller D','PAS','Passive');
cfg.fsz = 6.5;        % tick font size [pt]
cfg.lsz = 6;          % legend font size [pt]
cfg.lw  = 1.0;        % main line width
cfg.lwB = 0.6;        % bound line width
% vehicle parameters of this batch (pmpc_init.m / prepare_step.m)
cfg.veh = struct('m',1830,'lf',1.219,'lr',1.731,'g',9.81, ...
                 'tw',1.575,'Re',0.393,'CarHat',103.8e3,'CbarR',0.0);
cfg.dir = struct('DLC','DLC_mu0.4_interp', ...
                 'FH','FishHook_80_mu0.9_cargo170_newplant', ...
                 'SL','Slalom_mu0.85_interp');
% figure sizes [cm]: full column / half column
cfg.full = [8.5 6.0]; cfg.half = [4.2 3.2];
end

% ================================================================== helpers
function h = newFig(cfg, kind)
if strcmpi(kind,'full'), wh = cfg.full; else, wh = cfg.half; end
h.f = figure('Units','centimeters','Position',[2 2 wh(1) wh(2)], ...
             'Color','w','Visible','off');
pos = [0.245 0.215 0.735 0.695];
h.ax = axes('Parent',h.f,'Units','normalized','Position',pos,'Box','on');
hold(h.ax,'on'); grid(h.ax,'on');
set(h.ax,'FontSize',cfg.fsz,'TickDir','in','LineWidth',0.5, ...
    'GridAlpha',0.18,'GridLineStyle',':');
h.cfg = cfg;

end

function saveFig(h, name)
cfg = h.cfg;
set(h.ax,'FontSize',cfg.fsz);
f = h.f;
savefig(f, fullfile(cfg.out, [name '.fig']));
exportgraphics(f, fullfile(cfg.out, [name '.pdf']), 'ContentType','vector');
exportgraphics(f, fullfile(cfg.out, [name '.png']), 'Resolution',300);
close(f);
fprintf('saved %s.fig/.pdf/.png\n', name);
end

function p = plotLine(ax, x, y, cfg, key, lw, lab, isBound)
if nargin < 6 || isempty(lw), lw = cfg.lw; end
if nargin < 8, isBound = false; end
c = cfg.col.(key);
if isBound
    c = cfg.col.(key) * 0.5 + 0.5;      % same hue, lighter
end
if isBound
    p = plot(ax, x, y, cfg.ls.(key), 'Color', c, 'LineWidth', cfg.lwB, ...
             'HandleVisibility','off');
    return
end
if strcmp(cfg.mk.(key),'none')
    p = plot(ax, x, y, cfg.ls.(key), 'Color', c, 'LineWidth', lw);
else
    n = numel(x);
    p = plot(ax, x, y, cfg.ls.(key), 'Color', c, 'LineWidth', lw, ...
             'Marker', cfg.mk.(key), 'MarkerSize', 2.8, ...
             'MarkerIndices', 1:max(1,round(n/32)):n, ...
             'MarkerFaceColor','none','MarkerEdgeColor',c);
end
if nargin >= 7 && ~isempty(lab), set(p,'DisplayName',lab); end
end

function putLegend(h, order, ncols, loc)
% Legend inside the axes (plan 2.2). When loc is 'auto', the corner with the
% fewest data points under the estimated legend box is chosen; if no location
% is free the least covered one is kept, for later manual adjustment in .fig.
ax = h.ax; cfg = h.cfg;
if nargin < 3 || isempty(ncols), ncols = 1; end
if nargin < 4 || isempty(loc), loc = 'auto'; end
ch = findobj(ax,'Type','line');
handles = []; labels = {};
for k = 1:numel(ch)
    lab = get(ch(k),'DisplayName');
    if isempty(lab) || strcmp(lab,''), continue; end
    handles(end+1) = ch(k); labels{end+1} = lab; %#ok<AGROW>
end
if isempty(handles), return; end
if nargin < 2 || isempty(order), order = {}; end
rank = zeros(1,numel(labels));
for k = 1:numel(labels)
    r = numel(order) + 1;
    for j = 1:numel(order)
        if contains(labels{k}, order{j}), r = j; break; end
    end
    rank(k) = r;
end
[~, idx] = sort(rank);
if strcmpi(loc,'auto')
    loc = bestCorner(ax, handles, numel(idx), ncols);
end
fs = cfg.lsz; if ncols == 1, fs = cfg.lsz - 0.5; end
lg = legend(ax, handles(idx), labels(idx), 'Location', loc, ...
            'NumColumns', ncols, 'Box','on', 'FontSize', fs, ...
            'Interpreter','none');
set(lg,'Color','w','ItemTokenSize',[9 7],'LineWidth',0.4);
end

function loc = bestCorner(ax, handles, n, ncols)
% Score the four corners by the number of data points under the box.
xl = xlim(ax); yl = ylim(ax);
w = 0.30 + 0.20*(ncols-1);          % estimated box width  (axes fraction)
hgt = 0.09*ceil(n/ncols) + 0.05;    % estimated box height (axes fraction)
cands = {'northeast','northwest','southeast','southwest'};
box = [1-w 1-hgt w hgt; 0 1-hgt w hgt; 1-w 0 w hgt; 0 0 w hgt];
cnt = zeros(1,4);
for k = 1:numel(handles)
    x = get(handles(k),'XData'); y = get(handles(k),'YData');
    if numel(x) > 2000, i = round(linspace(1,numel(x),2000)); x = x(i); y = y(i); end
    fx = (x - xl(1))/diff(xl); fy = (y - yl(1))/diff(yl);
    ok = isfinite(fx) & isfinite(fy);
    fx = fx(ok); fy = fy(ok);
    for j = 1:4
        cnt(j) = cnt(j) + sum(fx >= box(j,1) & fx <= box(j,1)+box(j,3) & ...
                              fy >= box(j,2) & fy <= box(j,2)+box(j,4));
    end
end
[~, ib] = min(cnt);
loc = cands{ib};
end

function windowMarkers(ax, nTarget)
% Re-space markers so that ~nTarget markers are visible in the current x-window.
if nargin < 2, nTarget = 14; end
xl = xlim(ax);
ch = findobj(ax,'Type','line');
for k = 1:numel(ch)
    if strcmp(get(ch(k),'Marker'),'none'), continue; end
    x = get(ch(k),'XData');
    i = find(x >= xl(1) & x <= xl(2));
    if numel(i) < 3, continue; end
    sel = i(round(linspace(1, numel(i), min(nTarget, numel(i)))));
    set(ch(k),'MarkerIndices', sel);
end
end

function finishXY(h, xlab, ylab, xl, yl)
ax = h.ax;
xlabel(ax, xlab); ylabel(ax, ylab);
if nargin >= 4 && ~isempty(xl), xlim(ax, xl); end
if nargin >= 5 && ~isempty(yl), ylim(ax, yl); end
set(ax,'Layer','top');
windowMarkers(ax, 14);
end

% --------------------------------------------------------------- data load
function d = loadCase(cfg, group, stem)
key = [group '_' stem];
persistent cache
if isempty(cache), cache = containers.Map(); end
if isKey(cache, key), d = cache(key); return; end
dirn = cfg.dir.(group);
S = load(fullfile(cfg.res, dirn, [stem '.mat']));
d.D = S.D; d.t = S.t(:).';
V = load(fullfile(cfg.res, dirn, [stem '_sig.mat']));
names = string(V.names(:));
for k = 1:numel(names)
    d.C.(char(names(k))) = V.data(:,k);
end
d.tv = V.t(:).';
cache(key) = d;
end

function [names, M] = readVS(path)
fid = fopen(path,'r'); txt = fread(fid, inf, '*char').'; fclose(fid);
depth = 0; inStr = false; esc = false; last = 0;
for i = 1:numel(txt)
    ch = txt(i);
    if inStr
        if esc, esc = false;
        elseif ch == '\', esc = true;
        elseif ch == '"', inStr = false; end
        continue
    end
    if ch == '"', inStr = true;
    elseif ch == '{', depth = depth + 1;
    elseif ch == '}'
        depth = depth - 1;
        if depth == 0, last = i; break; end
    end
end
hdr = jsondecode(txt(1:last));
ch = hdr.VsChannelGroup.Channels;
names = strings(1, numel(ch));
for i = 1:numel(ch)
    a = ch(i).NameAliases;
    if iscell(a), names(i) = string(a{1}); else, names(i) = string(a); end
end
fid = fopen([path(1:end-3) '.vsb'],'r');
h6 = fread(fid, 6, 'int32'); nch = h6(6);
M = fread(fid, [nch, Inf], 'float32'); fclose(fid);
M = M.';
end

function d = addVS(cfg, group, stem, d, want)
dirn = cfg.dir.(group);
[names, M] = readVS(fullfile(cfg.res, dirn, [stem '.vs']));
idx = zeros(1,numel(want));
for k = 1:numel(want)
    j = find(strcmp(names, want{k}), 1);
    if isempty(j), error('channel %s not found', want{k}); end
    idx(k) = j;
end
d.V = M(:, idx); d.tvs = M(:, find(strcmp(names,'T_Record'),1));
d.vnames = want;
end

% ------------------------------------------------------- Mz / Md boundaries
function [lo, hi] = mzBounds_(cfg, d, mu)
V = cfg.veh;
Fz = [d.C.Fz_L1 d.C.Fz_L2 d.C.Fz_R1 d.C.Fz_R2];
Fy = [d.C.Fy_L1 d.C.Fy_L2 d.C.Fy_R1 d.C.Fy_R2];
if ~isfield(d,'V')
    d = addVS(cfg, d.group, d.stem, d, ...
              {'MY_DR_L1','MY_DR_L2','MY_DR_R1','MY_DR_R2'});
end
Td = d.V;
delta = deg2rad(d.C.Steer_L1);
bm = max(V.Re*sqrt(max((mu*Fz).^2 - Fy.^2, 0)) - max(Td,0), 0);
hi = zeros(size(delta)); lo = zeros(size(delta));
sm = abs(delta) < 0.15;
hi(sm) =  V.tw/(2*V.Re)*(bm(sm,1)+bm(sm,2));
lo(sm) = -V.tw/(2*V.Re)*(bm(sm,3)+bm(sm,4));
lg = ~sm;
hi(lg) = ( V.tw/2*(bm(lg,1).*cos(delta(lg))+bm(lg,2)) ...
          - V.lf*sin(delta(lg)).*bm(lg,1))/V.Re;
lo(lg) = (-V.tw/2*(bm(lg,3).*cos(delta(lg))+bm(lg,4)) ...
          - V.lf*sin(delta(lg)).*bm(lg,3))/V.Re;
hi = max(hi,0); lo = min(lo,0);
end

% ====================================================================== DLC
function dlcRef = dlcPath(x)
dlcRef = 3.556/2*(1+tanh(4/27.852*(x-80.059))) ...
       - 3.556/2*(1+tanh(4/24.008*(x-132.357)));
end

function d1 = dlcXY(cfg)
d1 = newFig(cfg,'full');
ax = d1.ax;
xr = linspace(25,200,600);
plot(ax, xr, dlcPath(xr), '-.', 'Color', cfg.col.REF, 'LineWidth', 1.0, ...
     'DisplayName','Reference');
cases = {'x3_c3c_75','C'; 'x3_c2_75','B'; 'x3_c1_75','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'DLC',cases{k,1});
    plotLine(ax, d.C.Xo, d.C.Yo, cfg, cases{k,2}, cfg.lw, cfg.nm.(cases{k,2}));
end
finishXY(d1,'X (m)','Y (m)',[20 200],[-1 5]);
set(d1.ax,'XTick',[25 85 145 200]);
putLegend(d1, {cfg.nm.A,cfg.nm.B,cfg.nm.C,'Reference'}, 1, 'northeast');
saveFig(d1,'fig_dlc_xy');
end

function d2 = dlcBetaR(cfg)
d2 = newFig(cfg,'full'); ax = d2.ax;
mu = 0.4; vx = 75/3.6; V = cfg.veh;
rmax = mu*V.g/vx;
fzr = V.m*V.g*V.lf/(V.lf+V.lr);
a_sat = min(atan(mu*fzr/max(abs(V.CbarR),1e3)), ...
            atan(3*mu*fzr/abs(V.CarHat)));
rr = linspace(-rmax, rmax, 120).';
bLo = atan(V.lr/vx*rr - a_sat); bHi = atan(V.lr/vx*rr + a_sat);
bx = rad2deg([bLo; flipud(bHi)]); by = rad2deg([rr; flipud(rr)]);
fill(ax, bx, by, [0.93 0.93 0.93], 'EdgeColor','none', ...
     'HandleVisibility','off');
plot(ax, rad2deg(bLo), rad2deg(rr), '--', 'Color', cfg.col.BND, ...
     'LineWidth',0.9,'DisplayName','Stability boundary');
plot(ax, rad2deg(bHi), rad2deg(rr), '--', 'Color', cfg.col.BND, ...
     'LineWidth',0.9,'HandleVisibility','off');
plot(ax, [-6 6], rad2deg([rmax rmax]), '--', 'Color', cfg.col.BND, ...
     'LineWidth',0.9,'HandleVisibility','off');
plot(ax, [-6 6], -rad2deg([rmax rmax]), '--', 'Color', cfg.col.BND, ...
     'LineWidth',0.9,'HandleVisibility','off');
cases = {'x3_c3c_75','C'; 'x3_c2_75','B'; 'x3_c1_75','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'DLC',cases{k,1});
    b = d.C.Beta; r = d.C.AVz;
    plotLine(ax, b, r, cfg, cases{k,2}, cfg.lw, cfg.nm.(cases{k,2}));
    n = numel(b);
    for fr = [0.22 0.62]
        i = max(2, round(fr*n));
        quiver(ax, b(i), r(i), b(i+2)-b(i), r(i+2)-r(i), 0, ...
               'Color', cfg.col.(cases{k,2}), 'MaxHeadSize', 6, ...
               'HandleVisibility','off');
    end
    plot(ax, b(1), r(1), 'p', 'Color','k','MarkerFaceColor','k', ...
         'MarkerSize',4,'HandleVisibility','off');
end
plot(ax, [0 0], [-18 18], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
plot(ax, [-6 6], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(d2,'\beta (deg)','r (deg/s)',[-6 6],[-18 18]);
putLegend(d2, {cfg.nm.A,cfg.nm.B,cfg.nm.C,'Stability boundary'}, 1);
saveFig(d2,'fig_dlc_betar');
end

function d3 = dlcMz(cfg)
d3 = newFig(cfg,'half'); ax = d3.ax;
mu = 0.4;
cases = {'x3_c3c_75','C'; 'x3_c2_75','B'; 'x3_c1_75','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'DLC',cases{k,1}); d.group='DLC'; d.stem=cases{k,1};
    s = interp1(d.tv, d.C.Xo, d.t, 'linear','extrap');
    plotLine(ax, s, d.D(18,:), cfg, cases{k,2}, cfg.lw, cfg.nm.(cases{k,2}));
end
plot(ax, [30 195], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(d3,'X (m)','M_z (N·m)',[30 195],[-2600 2600]);
putLegend(d3, {cfg.nm.A,cfg.nm.B,cfg.nm.C}, 1);
saveFig(d3,'fig_dlc_mz');
end

function d4 = dlcVx(cfg)
d4 = newFig(cfg,'half'); ax = d4.ax;
cases = {'x3_c3c_75','C'; 'x3_c2_75','B'; 'x3_c1_75','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'DLC',cases{k,1});
    plotLine(ax, d.C.Xo, d.C.Vx, cfg, cases{k,2}, 1.0, cfg.nm.(cases{k,2}));
end
plot(ax, [30 195], [75 75], '-.', 'Color', cfg.col.REF, 'LineWidth',0.9, ...
     'DisplayName','Reference');
finishXY(d4,'X (m)','V_x (km/h)',[30 195],[66 78]);
putLegend(d4, {cfg.nm.A,cfg.nm.B,cfg.nm.C,'Target'}, 1);
saveFig(d4,'fig_dlc_vx');
end

function d5 = dlcEy(cfg)
d5 = newFig(cfg,'half'); ax = d5.ax;
cases = {'x3_c3c_75','C'; 'x3_c2_75','B'; 'x3_c1_75','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'DLC',cases{k,1});
    s = interp1(d.tv, d.C.Xo, d.t, 'linear','extrap');
    plotLine(ax, s, d.D(1,:), cfg, cases{k,2}, 1.0, cfg.nm.(cases{k,2}));
end
plot(ax, [30 195], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(d5,'X (m)','e_y (m)',[30 195],[-2 2]);
putLegend(d5, {cfg.nm.A,cfg.nm.B,cfg.nm.C}, 1);
saveFig(d5,'fig_dlc_ey');
end

function d6 = dlcEpsi(cfg)
d6 = newFig(cfg,'half'); ax = d6.ax;
cases = {'x3_c3c_75','C'; 'x3_c2_75','B'; 'x3_c1_75','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'DLC',cases{k,1});
    s = interp1(d.tv, d.C.Xo, d.t, 'linear','extrap');
    plotLine(ax, s, rad2deg(d.D(2,:)), cfg, cases{k,2}, 1.0, cfg.nm.(cases{k,2}));
end
plot(ax, [30 195], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(d6,'X (m)','e_\psi (deg)',[30 195],[-20 20]);
putLegend(d6, {cfg.nm.A,cfg.nm.B,cfg.nm.C}, 1);
saveFig(d6,'fig_dlc_epsi');
end

% ================================================================ FishHook
function f1 = fhLtr(cfg)
f1 = newFig(cfg,'half'); ax = f1.ax;
cases = {'x3t_q40','C'; 'x3t_q40nc','D'; 'x3m_q15','A'; 'p1a_190','PAS'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'FH',cases{k,1});
    tv = d.tv; y = d.C.LTR;
    if strcmp(cases{k,2},'PAS')
        i = find(abs(y) > 0.995, 1);
        if ~isempty(i), tv = tv(1:i+2); y = y(1:i+2); end
    end
    plotLine(ax, tv, y, cfg, cases{k,2}, cfg.lw, cfg.nm.(cases{k,2}));
end
plot(ax, [0 5], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
for y = [0.8 -0.8]
    plot(ax, [0 5], [y y], '--', 'Color','k','LineWidth',0.8, ...
         'HandleVisibility','off');
end
for y = [1 -1]
    plot(ax, [0 5], [y y], ':', 'Color','k','LineWidth',0.8, ...
         'HandleVisibility','off');
end
d0 = loadCase(cfg,'FH','x3m_q15');
[~, i1] = max(abs(d0.C.Steer_SW));
t1 = d0.tv(i1);
j = find(sign(d0.C.Steer_SW(i1:end)) ~= sign(d0.C.Steer_SW(i1)), 1);
for tr = [t1, d0.tv(i1+j-1)]
    plot(ax, [tr tr], [-1.08 1.08], ':', 'Color',[0.55 0.55 0.55], ...
         'LineWidth',0.5,'HandleVisibility','off');
end
finishXY(f1,'time (s)','LTR (-)',[0 5],[-1.08 1.08]);
set(f1.ax,'YTick',[-1 -0.8 0 0.8 1],'YTickLabel',{'-1','-0.8','0','0.8','1'});
putLegend(f1, {cfg.nm.A,cfg.nm.C,cfg.nm.D,'Passive'}, 1, 'southwest');
saveFig(f1,'fig_fh_ltr');
end

function f2 = fhRoll(cfg)
f2 = newFig(cfg,'half'); ax = f2.ax;
cases = {'x3t_q40','C'; 'x3t_q40nc','D'; 'x3m_q15','A'; 'p1a_190','PAS'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'FH',cases{k,1});
    plotLine(ax, d.tv, d.C.Roll, cfg, cases{k,2}, cfg.lw, cfg.nm.(cases{k,2}));
end
plot(ax, [0 5], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(f2,'time (s)','\phi (deg)',[0 5],[-10 12]);
putLegend(f2, {cfg.nm.A,cfg.nm.C,cfg.nm.D,'Passive'}, 1, 'northwest');
saveFig(f2,'fig_fh_roll');
end

function f3 = fhMd(cfg)
f3 = newFig(cfg,'half'); ax = f3.ax;
cases = {'x3m_q15','A'; 'x3t_q40','C'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'FH',cases{k,1});
    plotLine(ax, d.t, d.D(6,:)/1000, cfg, cases{k,2}, 1.0, cfg.nm.(cases{k,2}));
end
plot(ax, [0 5], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(f3,'time (s)','M_d (kN·m)',[0 5],[-5.5 5.5]);
putLegend(f3, {cfg.nm.A,cfg.nm.C}, 1);
saveFig(f3,'fig_fh_md');
end

% ================================================================== Slalom
function s1 = slEy(cfg)
s1 = newFig(cfg,'half'); ax = s1.ax;
cases = {'d3_td200','C'; 'd3_sla_nc','D'; 'd3_sla_mpc','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'SL',cases{k,1});
    s = interp1(d.tv, d.C.Xo, d.t, 'linear','extrap');
    plotLine(ax, s, d.D(1,:), cfg, cases{k,2}, cfg.lw, cfg.nm.(cases{k,2}));
end
plot(ax, [30 200], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(s1,'X (m)','e_y (m)',[30 200],[-1.2 1.2]);
putLegend(s1, {cfg.nm.A,cfg.nm.C,cfg.nm.D}, 1);
saveFig(s1,'fig_sl_ey');
end

function s2 = slMz(cfg)
s2 = newFig(cfg,'half'); ax = s2.ax;
cases = {'d3_td200','C'; 'd3_sla_nc','D'; 'd3_sla_mpc','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'SL',cases{k,1}); d.group='SL'; d.stem=cases{k,1};
    s = interp1(d.tv, d.C.Xo, d.t, 'linear','extrap');
    plotLine(ax, s, d.D(18,:), cfg, cases{k,2}, cfg.lw, cfg.nm.(cases{k,2}));
end
plot(ax, [30 200], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(s2,'X (m)','M_z (N·m)',[30 200],[-6500 6500]);
putLegend(s2, {cfg.nm.A,cfg.nm.C,cfg.nm.D}, 1);
saveFig(s2,'fig_sl_mz');
end

function s3 = slLtr(cfg)
s3 = newFig(cfg,'half'); ax = s3.ax;
cases = {'d3_td200','C'; 'd3_sla_nc','D'; 'd3_sla_mpc','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'SL',cases{k,1});
    plotLine(ax, d.tv, d.C.LTR, cfg, cases{k,2}, 1.0, cfg.nm.(cases{k,2}));
end
for y = [0.8 -0.8], plot(ax,[0 9],[y y],'--','Color','k','LineWidth',0.8, ...
        'HandleVisibility','off'); end
for y = [1 -1], plot(ax,[0 9],[y y],':','Color','k','LineWidth',0.8, ...
        'HandleVisibility','off'); end
finishXY(s3,'time (s)','LTR (-)',[0 9],[-1.08 1.08]);
set(s3.ax,'YTick',[-1 -0.8 0 0.8 1],'YTickLabel',{'-1','-0.8','0','0.8','1'});
putLegend(s3, {cfg.nm.A,cfg.nm.C,cfg.nm.D}, 1, 'northeast');
saveFig(s3,'fig_sl_ltr');
end

function s4 = slMd(cfg)
s4 = newFig(cfg,'half'); ax = s4.ax;
cases = {'d3_td200','C'; 'd3_sla_nc','D'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'SL',cases{k,1});
    plotLine(ax, d.t, d.D(6,:), cfg, cases{k,2}, 1.0, cfg.nm.(cases{k,2}));
end
plot(ax, [1.5 3.6], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(s4,'time (s)','M_d^a (N·m)',[1.5 3.6],[-3200 3400]);
putLegend(s4, {cfg.nm.C,cfg.nm.D}, 1);
saveFig(s4,'fig_sl_md');
end

function s5 = slAxle(cfg, axle)
switch axle
    case 'front', ia = [10 12]; it = [14 16]; nm = {'LF','RF'};
    case 'rear',  ia = [11 13]; it = [15 17]; nm = {'LR','RR'};
end
s5 = newFig(cfg,'half'); ax = s5.ax;
d = loadCase(cfg,'SL','d3_td200');
c1 = [0.70 0.13 0.13]; c2 = [0.85 0.42 0.40];
cols = {c1, c2}; mks = {'o','s'};
for k = 1:2
    p = plot(ax, d.t, d.D(it(k),:), '--', 'Color', cols{k}, 'LineWidth',0.8, ...
             'DisplayName', [nm{k} ' Command']);
    plot(ax, d.t, d.D(ia(k),:), '-', 'Color', cols{k}, 'LineWidth',1.1, ...
         'Marker', mks{k}, 'MarkerSize',3, ...
         'MarkerIndices', 1:max(1,round(numel(d.t)/20)):numel(d.t), ...
         'MarkerFaceColor','none','MarkerEdgeColor',cols{k}, ...
         'DisplayName',[nm{k} ' Actual']);
    set(p,'DisplayName',[nm{k} ' Command']);
end
plot(ax, [1.5 3.6], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(s5,'time (s)','F_d (N)',[1.5 3.6],[-2400 3100]);
putLegend(s5, {[nm{1} ' Command'],[nm{1} ' Actual'], ...
               [nm{2} ' Command'],[nm{2} ' Actual']}, 1, 'northwest');
if strcmp(axle,'front')
    saveFig(s5,'fig_sl_fdf');
else
    saveFig(s5,'fig_sl_fdr');
end
end

% ---------------------------------------------------- additional panels ---
function t = dlcSw(cfg)
t = newFig(cfg,'half'); ax = t.ax;
cases = {'x3_c3c_75','C'; 'x3_c2_75','B'; 'x3_c1_75','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'DLC',cases{k,1});
    s = interp1(d.tv, d.C.Xo, d.tv, 'linear','extrap');
    plotLine(ax, s, d.C.Steer_SW, cfg, cases{k,2}, 0.8, cfg.nm.(cases{k,2}));
end
plot(ax, [30 195], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(t,'X (m)','\delta_{sw} (deg)',[30 195],[-320 800]);
putLegend(t, {cfg.nm.A,cfg.nm.B,cfg.nm.C}, 1);
saveFig(t,'fig_dlc_sw');
end

function t = dlcYaw(cfg)
t = newFig(cfg,'half'); ax = t.ax;
cases = {'x3_c3c_75','C'; 'x3_c2_75','B'; 'x3_c1_75','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'DLC',cases{k,1});
    plotLine(ax, d.C.Xo, d.C.AVz, cfg, cases{k,2}, 0.8, cfg.nm.(cases{k,2}));
end
plot(ax, [30 195], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(t,'X (m)','r (deg/s)',[30 195],[-20 20]);
putLegend(t, {cfg.nm.A,cfg.nm.B,cfg.nm.C}, 1);
saveFig(t,'fig_dlc_yaw');
end

function t = fhRollRate(cfg)
t = newFig(cfg,'half'); ax = t.ax;
cases = {'x3t_q40','C'; 'x3t_q40nc','D'; 'x3m_q15','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'FH',cases{k,1});
    plotLine(ax, d.t, rad2deg(d.D(4,:)), cfg, cases{k,2}, 0.8, cfg.nm.(cases{k,2}));
end
plot(ax, [0 5], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(t,'time (s)','p (deg/s)',[0 5],[-25 25]);
putLegend(t, {cfg.nm.A,cfg.nm.C,cfg.nm.D}, 1);
saveFig(t,'fig_fh_pdot');
end

function t = fhMz(cfg)
t = newFig(cfg,'half'); ax = t.ax;
cases = {'x3t_q40','C'; 'x3t_q40nc','D'; 'x3m_q15','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'FH',cases{k,1});
    plotLine(ax, d.t, d.D(18,:)/1000, cfg, cases{k,2}, 0.8, cfg.nm.(cases{k,2}));
end
plot(ax, [0 5], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(t,'time (s)','M_z (kN·m)',[0 5],[-11 9]);
putLegend(t, {cfg.nm.A,cfg.nm.C,cfg.nm.D}, 1);
saveFig(t,'fig_fh_mz');
end

function t = fhVx(cfg)
t = newFig(cfg,'half'); ax = t.ax;
cases = {'x3t_q40','C'; 'x3t_q40nc','D'; 'x3m_q15','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'FH',cases{k,1});
    plotLine(ax, d.tv, d.C.Vx, cfg, cases{k,2}, 0.8, cfg.nm.(cases{k,2}));
end
finishXY(t,'time (s)','V_x (km/h)',[0 5],[40 85]);
putLegend(t, {cfg.nm.A,cfg.nm.C,cfg.nm.D}, 1);
saveFig(t,'fig_fh_vx');
end

function t = fhSw(cfg)
t = newFig(cfg,'half'); ax = t.ax;
d = loadCase(cfg,'FH','x3t_q40');
plot(ax, d.tv, d.C.Steer_SW, 'k-', 'LineWidth',1.0);
plot(ax, [0 5], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(t,'time (s)','\delta_{sw} (deg)',[0 5],[-380 380]);
saveFig(t,'fig_fh_sw');
end

function t = slEpsi(cfg)
t = newFig(cfg,'half'); ax = t.ax;
cases = {'d3_td200','C'; 'd3_sla_nc','D'; 'd3_sla_mpc','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'SL',cases{k,1});
    s = interp1(d.tv, d.C.Xo, d.t, 'linear','extrap');
    plotLine(ax, s, rad2deg(d.D(2,:)), cfg, cases{k,2}, 0.8, cfg.nm.(cases{k,2}));
end
plot(ax, [30 200], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(t,'X (m)','e_\psi (deg)',[30 200],[-9 9]);
putLegend(t, {cfg.nm.A,cfg.nm.C,cfg.nm.D}, 1);
saveFig(t,'fig_sl_epsi');
end

function t = slSw(cfg)
t = newFig(cfg,'half'); ax = t.ax;
cases = {'d3_td200','C'; 'd3_sla_nc','D'; 'd3_sla_mpc','A'};
for k = 1:size(cases,1)
    d = loadCase(cfg,'SL',cases{k,1});
    plotLine(ax, d.C.Xo, d.C.Steer_SW, cfg, cases{k,2}, 0.8, cfg.nm.(cases{k,2}));
end
plot(ax, [30 200], [0 0], 'k-', 'LineWidth',0.4,'HandleVisibility','off');
finishXY(t,'X (m)','\delta_{sw} (deg)',[30 200],[-560 560]);
putLegend(t, {cfg.nm.A,cfg.nm.C,cfg.nm.D}, 1);
saveFig(t,'fig_sl_sw');
end

