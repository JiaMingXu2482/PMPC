function ok = func_CarSimLib()
%FUNC_CARSIMLIB  把 CarSim 的 Solver_SF 库挂上 MATLAB 路径(已在就什么都不做)
%   ok = func_CarSimLib()   true = 现在能找到 Solver_SF
%
%   为什么需要:
%     cq3_2019 / pmpc_mil 里的 CarSim 块是对库 Solver_SF 的链接引用。
%     CarSim 点 "Send to Simulink" 时会把该库目录加进**那个** MATLAB 会话的
%     路径, 所以平时手工跑没事。但用 matlab -batch 另起一个干净会话时没有,
%     sim 会报:
%         无法解析 'pmpc_mil/CarSim' 引用的库 'Solver_SF'
%     以前是靠调用方在命令行里自己 addpath —— 忘一次就炸一次, 所以挪进
%     run_ds 自动做。
%
%   路径从 simfile.sim 的 PROGDIR 推, **不写死机器路径**。

ok = ~isempty(which('Solver_SF'));
if ok, return; end

if exist('simfile.sim','file') ~= 2
    return      % 没有 simfile 就不是能跑仿真的环境, 交给上层报错
end
tk = regexp(fileread('simfile.sim'), '(?m)^PROGDIR\s+(.+?)\s*$', 'tokens', 'once');
if isempty(tk), return; end
progdir = strtrim(tk{1});

%  R2014b 之后的库在 Matlab84+ 子目录, 老版本在 solvers 根下
for sub = { fullfile('Programs','solvers','Matlab84+'), ...
            fullfile('Programs','solvers') }
    p = fullfile(progdir, sub{1});
    if exist(p,'dir') == 7, addpath(p); end
end
ok = ~isempty(which('Solver_SF'));
if ~ok
    warning('func_CarSimLib:NotFound', ...
        ['没能在 %s 下找到 Solver_SF 库。\n' ...
         '若接下来报"无法解析引用的库 Solver_SF", 先在 CarSim 里点一次 ' ...
         'Send to Simulink, 或手工 addpath 到 CarSim 的 solvers 目录。'], progdir);
end
end
