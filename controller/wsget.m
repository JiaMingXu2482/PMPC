function v = wsget(name, dflt)
% 读运行配置。优先级:
%   1) CarSim 数据集 —— 展开的 Run_all.par 里的 DEFINE_PARAMETER
%   2) base workspace 变量
%   3) 缺省值
% 这样模式绑定在 Run Control 上, 不会出现"用 PMPC 数据集跑出 baseline"。
% 不缓存: simfile 每次 Send to Simulink 都会改指向, persistent 会读到上一次的
runall = '';
%  不硬编码 CarSim 安装路径 —— 走 func_CarSimResDir 从 simfile.sim 现解析。
%  硬编码那版在换数据集(run_ds)后会读到旧的 Run_all.par, 且毫无提示。
try
    par = fullfile(func_CarSimResDir(), 'Run_all.par');
    if exist(par,'file') == 2
        runall = fileread(par);
    end
catch
end
if ~isempty(runall)
    tok = regexp(runall, ['(?m)^\s*(?:DEFINE_PARAMETER\s+)?' name '\s*=\s*([-\d.eE+]+)'], ...
                 'tokens', 'once');
    if ~isempty(tok)
        v = str2double(tok{1});
        if ~isnan(v), return; end
    end
end
if evalin('base', ['exist(''' name ''',''var'')'])
    v = evalin('base', name);
else
    v = dflt;
end
