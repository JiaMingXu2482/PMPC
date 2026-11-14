function pmpc_base_setup(here, maneuver, horizon, opts)
%PMPC_BASE_SETUP 在基础工作区准备基线 MPC 运行所需的变量。
%   由 pmpc_base.slx 的 InitFcn 调用，也可手动执行：
%     pmpc_base_setup(pwd)
%     pmpc_base_setup(pwd, 1, struct('Ts',0.04,'Np',10,'Nc',5))
%
% 放置的变量：
%   params / Reftraj / state0 / meas0
%   PMPC_Base_MeasBus / PMPC_Base_StateBus / PMPC_Base_PredBus

if nargin < 1 || isempty(here), here = pwd; end

% CarSim S-Function 库（Solver_SF.mdl / vs_sf.mexw64）必须在路径上，
% 否则 CarSim 块会退化成未解析的 Reference block。
cs_solvers = 'D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers';
if isfolder(cs_solvers) && isempty(which('vs_sf'))
    addpath(cs_solvers, '-begin');
end
% InitFcn 只传 here，其余从基础工作区的 base_* 变量读（run_baseline 负责设置）
if nargin < 2 || isempty(maneuver)
    maneuver = 1;
    if evalin('base','exist(''base_maneuver'',''var'')')
        maneuver = evalin('base','base_maneuver');
    end
end
if nargin < 3 || isempty(horizon)
    horizon = struct();
    if evalin('base','exist(''base_horizon'',''var'')')
        horizon = evalin('base','base_horizon');
    end
end
if nargin < 4 || isempty(opts)
    opts = struct();
    if evalin('base','exist(''base_opts'',''var'')')
        opts = evalin('base','base_opts');
    end
end

addpath(here, '-begin');
% 纵向仲裁与协调控制器共用 pmpc_core/lon_arbitrate.m（只读引用，不要复制）
if isfolder(fullfile(fileparts(here),'pmpc_core'))
    addpath(fullfile(fileparts(here),'pmpc_core'), '-begin');
end

[params, Reftraj, state0, meas0] = pmpc_base_init(maneuver, horizon, opts);
[mb, sb, pb] = pmpc_base_bus(meas0, state0);

assignin('base', 'params',  params);
assignin('base', 'Reftraj', Reftraj);
assignin('base', 'state0',  state0);
assignin('base', 'meas0',   meas0);
assignin('base', 'PMPC_Base_MeasBus',  mb);
assignin('base', 'PMPC_Base_StateBus', sb);
assignin('base', 'PMPC_Base_PredBus',  pb);

% v_target 常量的值（km/h）；未设置时默认 80
if ~evalin('base','exist(''base_vtarget'',''var'')')
    assignin('base', 'base_vtarget', 80);
end

fprintf(['基线 MPC: maneuver=%d  mu=%.2f  Ts=%.3f Np=%d Nc=%d  ' ...
         'Q=[%s] R=[%s] I=%.2f A\n'], ...
    maneuver, params.road.mu, params.M.Ts, params.M.Np, params.M.Nc, ...
    strjoin(arrayfun(@(x) sprintf('%g',x), params.M.Q_norm.', 'UniformOutput',false), ' '), ...
    strjoin(arrayfun(@(x) sprintf('%g',x), params.M.R_norm.', 'UniformOutput',false), ' '), ...
    params.M.cdc_fixed_I);
end
