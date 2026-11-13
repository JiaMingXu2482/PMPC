function f = openfig_big(name, widthPx)
%OPENFIG_BIG  Open a saved .fig at a comfortable on-screen size.
%
%   The Section V figures are created with
%       figure('Units','centimeters','Position',[2 2 4.2 3.2])
%   so a half-column figure reopens as a ~4x3 cm window parked in the bottom
%   left corner of the screen -- it looks like nothing happened.
%
%   f = openfig_big('fig_dlc_sw.fig')          % 900 px wide, centred
%   f = openfig_big('fig_dlc_sw.fig', 1200)    % wider
%
%   Then edit, and re-export with:
%       exportgraphics(f, 'fig_dlc_sw.pdf', 'ContentType', 'vector');
%       exportgraphics(f, 'fig_dlc_sw.png', 'Resolution', 300);
%       savefig(f, 'fig_dlc_sw.fig');

if nargin < 1 || isempty(name)
    [file, folder] = uigetfile('*.fig', 'Select a .fig');
    if isequal(file, 0), f = []; return; end
    name = fullfile(folder, file);
end
if nargin < 2 || isempty(widthPx), widthPx = 900; end

f = openfig(name, 'new', 'visible');
set(f, 'Units', 'pixels');
p = get(f, 'Position');
aspect = p(4) / p(3);
heightPx = round(widthPx * aspect);

scr = get(0, 'ScreenSize');
x = max(1, round((scr(3) - widthPx) / 2));
y = max(1, round((scr(4) - heightPx) / 2));
set(f, 'Position', [x, y, widthPx, heightPx]);
set(f, 'Visible', 'on');
figure(f);
end
