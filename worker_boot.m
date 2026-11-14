function worker_boot(tree_dir)
%WORKER_BOOT 新建 MATLAB 进程的完整启动序列。
%
%   matlab -batch "worker_boot('<worktree 目录>')"
%
% 为什么需要它：pmpc_base.slx / pmpc_mil.slx 里的减振器正模型引用了
% Ki / Kp / D_init 等变量，但项目里没有任何代码调用 init_valve2，也没有
% 任何地方定义 D_init —— 一直靠常驻 MATLAB 的 base 工作区残留着这些变量，
% 所以从没暴露。新建的 -batch 进程必须显式补上。
%
% 只读调用，不修改任何模型或 .m 文件。

if nargin < 1 || isempty(tree_dir), tree_dir = pwd; end
tree_dir = char(tree_dir);

% --- 1. CarSim S-Function 库 ---
cs = 'D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers';
if isfolder(cs) && isempty(which('vs_sf'))
    addpath(cs, '-begin');
end

% --- 2. 本树自己的目录（-begin，避免被其他 worktree 同名文件抢占）---
for sub = {'pmpc_core', 'nlcsnn', 'baseline_mpc', ''}
    d = fullfile(tree_dir, sub{1});
    if isfolder(d), addpath(d, '-begin'); end
end

% --- 3. CDC 电流环参数（进基础工作区）---
iv = fullfile(tree_dir, 'init_valve2.m');
if isfile(iv)
    evalin('base', sprintf('run(''%s'')', iv));
else
    warning('worker_boot:noInitValve', '找不到 init_valve2.m：%s', iv);
end

% --- 4. D_init：开关模型 PI 积分器初值，仓库里没有定义处 ---
% init_valve2 只产生平均值模型的 D_init_avg（0.1），而模型块
% "Discrete PID Controller3/4"（InitialConditionForIntegrator）用的是
% 开关模型的 D_init。该值长期只存在于常驻 MATLAB 工作区（0.12551，
% 对应 R*I/Vdc = 3*1.004/24，即约 1.0 A 稳态占空比）。
% 已经存在时不覆盖，避免踩掉别的 agent 显式设的值。
if evalin('base', 'exist(''D_init'',''var'')') == 0
    Vdc = 24; R = 3;
    assignin('base', 'D_init', (R*1.0041)/Vdc);   % 0.12551
end

% --- 5. 清掉可能残留的编译缓存 ---
bdclose('all');
end
