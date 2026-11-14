function build_pmpc_base(mdl)
%BUILD_PMPC_BASE 程序化搭建基线 MPC 的 Simulink 模型 pmpc_base.slx。
%
%   build_pmpc_base          % 生成/覆盖 pmpc_base.slx
%   build_pmpc_base('name')  % 指定模型名
%
% 结构（比 pmpc_mil.slx 简单——没有边界模型、没有逆模型）：
%
%   [CarSim] --54ch--> [Estimator] --meas--> [基线 MPC] --SW-----+
%        ^                                    |--Icmd(4)--> [减振器正模型] --Fa(4)-->
%        |                                    |--Tb(4)-----------------------------> [Mux 11]
%        +------------------------------------+                       
%                                    [Unit Delay]<--stateNext
%
% CarSim 侧 IO（由 simfile 指向的 Run_bl_* 数据集决定，顺序不可乱）：
%   MYBK_L1/L2/R1/R2(制动力矩) FD_L1/L2/R1/R2(减振器力) STEER_SW THROTTLE PCON_BK
%   => Mux(11) = [Tb(1:4); Fa(1:4); SW; 0; 0]
%
% 减振器：控制器给固定电流 M.cdc_fixed_I（四角相同），经正模型（nlcsnn，1000 Hz）
% 换算成阻尼力 Fa 再送进 CarSim。逆模型被常数电流取代——这正是基线本意。
%
% InitFcn 调 pmpc_base_setup（在工作区放 params/Reftraj/state0/meas0/bus）。

if nargin < 1 || isempty(mdl), mdl = 'pmpc_base'; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(fullfile(here,[mdl '.slx']), 'file')
    delete(fullfile(here,[mdl '.slx']));
end
new_system(mdl);
open_system(mdl);
addpath(here);

% 与 pmpc_mil 同一套求解器设置：CarSim S-Function 自己算，Simulink 侧 1 ms 定步长
set_param(mdl, 'Solver', 'ode1', 'FixedStep', '0.001', 'StopTime', '15');

%% CarSim S-Function（与 pmpc_mil 同一个块：库引用，SIMFILE 由脚本设置）
% 不能用裸 'built-in/S-Function' + FunctionName='vs_sf'：vs_sf 的参数
% （SIMFILE / MultithreadedSim）由 Solver_SF 库块提供，裸块传 0 个参数，
% 编译时报 "Unexpected S-Function parameter count"。
cs_solvers = 'D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers';
if isfolder(cs_solvers) && isempty(which('vs_sf'))
    addpath(cs_solvers, '-begin');
end
if ~bdIsLoaded('Solver_SF')
    load_system(fullfile(cs_solvers, 'Solver_SF.mdl'));
end
add_block('Solver_SF/CarSim S-Function', [mdl '/CarSim'], ...
    'Position', [40 320 110 380]);

%% Estimator：54 路 -> meas 结构
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Estimator'], 'Position', [180 320 250 380]);
estChart = sfroot().find('-isa','Stateflow.EMChart','Path',[mdl '/Estimator']);
estChart.Script = sprintf([ ...
'function meas = fcn(u) %%#codegen\n' ...
'u = double(u(:));\n' ...
'meas = struct(''X'',u(40),''Y'',u(41),''Vx'',u(42)/3.6,''Vy'',u(43)/3.6, ...\n' ...
'    ''yaw'',u(39)*pi/180,''r'',u(34)*pi/180, ...\n' ...
'    ''phi'',u(38)*pi/180,''p'',u(33)*pi/180, ...\n' ...
'    ''delta_front'',[u(46);u(47)]*pi/180,''steering_wheel'',u(48)*pi/180, ...\n' ...
'    ''Fx'',u(9:12),''Fy'',u(13:16),''Fz'',u(17:20), ...\n' ...
'    ''Fd'',u(21:24),''Td'',u(29:32),''vd'',u(1:4)/1000);\n' ...
'end\n']);
% 结构体信号必须显式声明为 Bus 类型，否则 Simulink 推不出输出大小/类型
% （报“无法确定模块的输出大小和/或类型”）。与 pmpc_mil 的 Estimator 同做法。
set_dt([mdl '/Estimator'], 'meas', 'Bus: PMPC_Base_MeasBus');

%% 主控制块
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Baseline MPC'], 'Position', [300 250 390 450]);
ctlChart = sfroot().find('-isa','Stateflow.EMChart','Path',[mdl '/Baseline MPC']);
ctlChart.Script = sprintf([ ...
'function [SW,Icmd,Tb,Xlocal,Ylocal,logVec,stateNext] = ...\n' ...
'    fcn(meas,delta_d,state) %%#codegen\n' ...
'[SW,Icmd,Tb,stateNext,Xlocal,Ylocal,logVec] = ...\n' ...
'    pmpc_base_step(meas,Reftraj,params,state,delta_d);\n' ...
'end\n']);
% Reftraj / params 声明为 Parameter，才能在被调函数里访问
add_param(ctlChart, 'Reftraj');
add_param(ctlChart, 'params');
% MPC 必须在 0.01 s（= Ts_exec）执行，否则会继承 CarSim 的 1 ms 步长，
% 每拍都算一遍 QP：物理上错（预测步长 Ts 与执行周期不符），速度上也撑不住。
lock_sample_time([mdl '/Baseline MPC'], '0.01');
set_dt([mdl '/Baseline MPC'], 'meas',      'Bus: PMPC_Base_MeasBus');
set_dt([mdl '/Baseline MPC'], 'state',     'Bus: PMPC_Base_StateBus');
set_dt([mdl '/Baseline MPC'], 'stateNext', 'Bus: PMPC_Base_StateBus');
set_dt([mdl '/Baseline MPC'], 'SW',   'double');
set_dt([mdl '/Baseline MPC'], 'Icmd', 'double');
set_dt([mdl '/Baseline MPC'], 'Tb',   'double');

%% Unit Delay（保存 stateNext -> state，1 个执行周期）
add_block('simulink/Discrete/Unit Delay', [mdl '/Unit Delay'], ...
    'SampleTime', '0.01', 'InitialCondition', 'state0', ...
    'Position', [470 470 510 510]);

%% 驾驶意图（这里只保留 delta_d=0）
add_block('simulink/Sources/Constant', [mdl '/delta_d'], ...
    'Value', '0', 'Position', [180 430 200 460]);

%% 减振器正模型（从 pmpc_mil 复制；5 入 4 出，1000 Hz）
% 输入：4 路电流 + CarSim 输出向量；输出：四角阻尼力 Fa
srcMdl = 'pmpc_mil';
srcFile = fullfile(root, [srcMdl '.slx']);
assert(isfile(srcFile), '缺少 %s（正模型从这里复制）', srcFile);
if ~bdIsLoaded(srcMdl), load_system(srcFile); end
add_block([srcMdl '/减振器正模型（1000Hz）'], [mdl '/DamperPlant'], ...
    'Position', [180 120 280 240]);

%% 信号拆分 / 合成
add_block('simulink/Signal Routing/Demux', [mdl '/DemuxTb'], ...
    'Outputs', '4', 'Position', [420 300 425 360]);
add_block('simulink/Signal Routing/Demux', [mdl '/DemuxIcmd'], ...
    'Outputs', '4', 'Position', [300 160 305 220]);
add_block('simulink/Signal Routing/Mux', [mdl '/CarSimIn'], ...
    'Inputs', '11', 'DisplayOption', 'bar', 'Position', [520 300 525 400]);
add_block('simulink/Sources/Constant', [mdl '/zero_throttle'], ...
    'Value', '0', 'Position', [420 430 450 450]);
add_block('simulink/Sources/Constant', [mdl '/zero_brake'], ...
    'Value', '0', 'Position', [420 470 450 490]);

%% 输出记录 / 空置输出
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace'], ...
    'VariableName', 'pmpc_log', 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', 'inf', 'Position', [560 540 620 570]);
% Icmd 也要记：基线定义性特征就是“四角恒 1.0 A”，得能拿数据证明
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace Icmd'], ...
    'VariableName', 'pmpc_Icmd', 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', 'inf', 'Position', [560 200 620 230]);
add_block('simulink/Sinks/Terminator', [mdl '/Term Xlocal'], ...
    'Position', [470 380 490 400]);
add_block('simulink/Sinks/Terminator', [mdl '/Term Ylocal'], ...
    'Position', [470 420 490 440]);

%% 连线 —— 控制回路
add_line(mdl, 'CarSim/1',        'Estimator/1',    'autorouting','on');
add_line(mdl, 'Estimator/1',     'Baseline MPC/1', 'autorouting','on');
add_line(mdl, 'delta_d/1',       'Baseline MPC/2', 'autorouting','on');
add_line(mdl, 'Unit Delay/1',    'Baseline MPC/3', 'autorouting','on');
% 图表输出顺序：[SW, Icmd, Tb, Xlocal, Ylocal, logVec, stateNext]
add_line(mdl, 'Baseline MPC/4',  'Term Xlocal/1',  'autorouting','on');
add_line(mdl, 'Baseline MPC/5',  'Term Ylocal/1',  'autorouting','on');
add_line(mdl, 'Baseline MPC/6',  'To Workspace/1', 'autorouting','on');
add_line(mdl, 'Baseline MPC/7',  'Unit Delay/1',   'autorouting','on');

%% 连线 —— 减振器：固定电流 -> 正模型 -> CarSim FD
add_line(mdl, 'Baseline MPC/2',  'DemuxIcmd/1',    'autorouting','on');
add_line(mdl, 'Baseline MPC/2',  'To Workspace Icmd/1', 'autorouting','on');
add_line(mdl, 'CarSim/1',        'DamperPlant/5',  'autorouting','on');
for k = 1:4
    add_line(mdl, sprintf('DemuxIcmd/%d',k),   sprintf('DamperPlant/%d',k), 'autorouting','on');
    add_line(mdl, sprintf('DamperPlant/%d',k), sprintf('CarSimIn/%d',k+4),  'autorouting','on');
end

%% 连线 —— 制动力矩 + 转角 + 空置的纵向输入
add_line(mdl, 'Baseline MPC/3',  'DemuxTb/1',      'autorouting','on');
for k = 1:4
    add_line(mdl, sprintf('DemuxTb/%d',k), sprintf('CarSimIn/%d',k), 'autorouting','on');
end
add_line(mdl, 'Baseline MPC/1',  'CarSimIn/9',     'autorouting','on');
add_line(mdl, 'zero_throttle/1', 'CarSimIn/10',    'autorouting','on');
add_line(mdl, 'zero_brake/1',    'CarSimIn/11',    'autorouting','on');
add_line(mdl, 'CarSimIn/1',      'CarSim/1',       'autorouting','on');

%% InitFcn
set_param(mdl, 'InitFcn', sprintf('pmpc_base_setup(''%s'')', ...
    strrep(here, '''', '''''')));
set_param(mdl, 'StopTime', '15');

save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('已生成 %s\n', fullfile(here, [mdl '.slx']));
fprintf('CarSim 输入：[Tb(4); Fa(4); SW; throttle; base_brake]\n');
end

%% ------------------------------------------------------------------
function add_param(chart, name)
d = Stateflow.Data(chart);
d.Name  = name;
d.Scope = 'Parameter';
end

function lock_sample_time(blkPath, Ts)
%LOCK_SAMPLE_TIME 把 MATLAB Function 块锁到固定离散采样时间。
% 块参数大多不支持直接设 SampleTime，真正生效的是图表属性 ChartUpdate/SampleTime。
ok = false;
try
    set_param(blkPath, 'SampleTime', Ts);
    ok = strcmp(get_param(blkPath, 'SampleTime'), Ts);
catch
end
if ~ok
    ch = sfroot().find('-isa','Stateflow.EMChart','Path',blkPath);
    assert(~isempty(ch), '找不到图表 %s', blkPath);
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime  = Ts;
    fprintf('采样时间经图表属性设置：%s -> %s\n', blkPath, ch.SampleTime);
else
    fprintf('采样时间经块参数设置：%s -> %s\n', blkPath, Ts);
end
end

function set_dt(blkPath, name, dt)%SET_DT 设置 MATLAB Function 块里信号的数据类型/总线类型
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blkPath);
assert(~isempty(ch), '找不到图表 %s', blkPath);
d = ch.find('-isa','Stateflow.Data','Name',name);
assert(numel(d)==1, '找不到数据 %s/%s', blkPath, name);
d.DataType = dt;
end
