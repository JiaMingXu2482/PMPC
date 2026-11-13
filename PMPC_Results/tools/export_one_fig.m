function export_one_fig(figFile, outFile)
%EXPORT_ONE_FIG  Export a 3x editing .fig to the 1x paper-size vector PDF.
%
%   export_one_fig('fig_fh_ltr.fig', 'fig_fh_ltr.pdf')
%
% The .fig files produced by plot_section5_figs.m are saved on a 3x canvas
% (12.6 x 9.6 cm for a half-column panel) with fonts and line widths also
% scaled 3x, so they open at a comfortable editing size. This helper scales
% fonts/lines back by 1/3, shrinks the canvas to the final 1x size, and writes
% a vector PDF. Use it after editing a .fig by hand.
%
% If outFile is omitted, the PDF is written next to the .fig with the same
% base name.

if nargin < 2 || isempty(outFile)
    [p, n] = fileparts(figFile);
    outFile = fullfile(p, [n '.pdf']);
end

f = openfig(figFile, 'invisible');
pos = get(f, 'Position');                 % 3x canvas [cm]
scaleFig(f, 1/3);                         % fonts/lines back to 1x
set(f, 'Units', 'centimeters', ...
    'Position', [pos(1) pos(2) pos(3)/3 pos(4)/3]);
exportgraphics(f, outFile, 'ContentType', 'vector');
close(f);
fprintf('exported %s\n', outFile);
end

function scaleFig(f, s)
axs = findobj(f, 'Type', 'axes');
for a = axs.'
    set(a, 'FontSize', get(a, 'FontSize') * s);
    set(a, 'LineWidth', get(a, 'LineWidth') * s);
end
txt = findobj(f, 'Type', 'text');
for t = txt.'
    set(t, 'FontSize', get(t, 'FontSize') * s);
end
ln = findobj(f, 'Type', 'line');
for l = ln.'
    set(l, 'LineWidth', get(l, 'LineWidth') * s);
end
lg = findobj(f, 'Type', 'legend');
for g = lg.'
    set(g, 'FontSize', get(g, 'FontSize') * s);
    its = get(g, 'ItemTokenSize');
    set(g, 'ItemTokenSize', its * s);
end
end
