function d = func_ErdDir(sub)
%FUNC_ERDDIR  返回项目内的 ERD 输出目录 (不存在则创建)
%   d = func_ErdDir()        -> <Simulink_Model>\simulation_results\output
%   d = func_ErdDir('x')     -> <Simulink_Model>\simulation_results\output\x
%
%   历史教训 (2026-09-17): 原来 run3b/sweepZ/plot_week 把输出目录硬编码成了
%   C:\Users\...\AppData\Local\Temp\claude\<会话UUID>\scratchpad —— 那是
%   Windows 临时目录, 且路径里带会话 ID, 换个会话或磁盘清理一跑就全失效。
%   一律改用本函数取项目内的相对路径。
if nargin < 1, sub = ''; end
root = func_ProjectRoot();
d = fullfile(root, 'simulation_results', 'output');
if ~isempty(sub), d = fullfile(d, sub); end
if ~exist(d, 'dir'), mkdir(d); end
end
