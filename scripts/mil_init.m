function mil_init(mode)
%MIL_INIT  R2018a 机器上跑 MIL(CarSim + Simulink)的初始化
%
%   mil_init        控制器由 CarSim 数据集名自动决定(推荐)
%   mil_init(1)     强制本文 PMPC
%   mil_init(2)     强制 baseline MPC
%
%   步骤: CarSim 里选数据集 -> Send to Simulink -> 回 MATLAB 跑 mil_init -> sim
%
%   MIL 保留每拍打印(方便看跟踪误差和 QP 成败)，控制器默认由数据集名判定。
if nargin < 1, mode = 0; end

assignin('base', 'PMPC_VERBOSE', 1);        % MIL 保留每拍打印
if mode > 0
    assignin('base', 'PMPC_MODE',    mode);
    assignin('base', 'PMPC_ZENGRHO', 0);
else
    evalin('base', 'clear PMPC_MODE PMPC_ZENGRHO');
end
evalin('base', 'clear PMPC_P');
evalin('base', 'setup_pmpc;');

P = evalin('base','PMPC_P');
fprintf('\n===== MIL 就绪 =====\n');
if isfield(P.S0.Constraints,'ControllerMode') && P.S0.Constraints.ControllerMode == 1
    fprintf('  控制器    ③ 本文 PMPC (统一QP: Ne=%d, Nr=%d)\n', ...
        P.Pm.MPCParameters.Ne, P.Pm.MPCParameters.Nr);
elseif isfield(P.S0.Constraints,'ZengRho_on') && P.S0.Constraints.ZengRho_on
    fprintf('  控制器    ② Zeng rho 调权\n');
else
    fprintf('  控制器    ① 固定权重 MPC / 见上方 setup_pmpc 的打印\n');
end
fprintf('  每拍打印  %s\n', local_onoff(P.Pm.MPCParameters.Verbose));
fprintf('  PMPC_P    Pm %d 字段 / S0 %d 字段\n', ...
        numel(fieldnames(P.Pm)), numel(fieldnames(P.S0)));
fprintf('  模型      pmpc_mil\n');
fprintf('\n  跑 sim(''pmpc_mil'')，结果用 chk_regress 检查。\n\n');
end

function s = local_onoff(v)
if v, s = '开'; else, s = '关'; end
end
