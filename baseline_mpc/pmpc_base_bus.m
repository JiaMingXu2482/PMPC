function [PMPC_Base_MeasBus, PMPC_Base_StateBus, PMPC_Base_PredBus] = ...
    pmpc_base_bus(meas0, state0)
%PMPC_BASE_BUS 基线 MPC 的接口总线定义（只在 InitFcn 里执行一次）。
% 不放进 MATLAB Function，不参与控制拍内计算。
% 字段、尺寸、类型全部由传入的模板结构体决定，所以改 Np/Nc 后
% 必须重新调用 pmpc_base_init + pmpc_base_bus + assignin。
%
% 注意：Simulink.Bus 在 R2024b 是【值类】，不能在 cell 数组里
% 用 buses{k}.Elements = ... 赋值（改的是副本），必须直接赋给变量。

PMPC_Base_MeasBus  = Simulink.Bus;
PMPC_Base_StateBus = Simulink.Bus;
PMPC_Base_PredBus  = Simulink.Bus;

% 预测总线模板：Y 为 6*N x 2*Nc，y 为 6*N x 1，AI 为 2*Nc x 2*Nc
N  = (numel(state0.active_main) - 4*numel(state0.plan) - 8) / 10;
Nc = numel(state0.plan)/2;
pred0 = struct('Y', zeros(6*N, 2*Nc), ...
               'y', zeros(6*N, 1), ...
               'AI', zeros(2*Nc, 2*Nc), ...
               'Ut', zeros(2*Nc, 1), ...
               'valid', false);

PMPC_Base_MeasBus.Elements  = make_elements(meas0);
PMPC_Base_StateBus.Elements = make_elements(state0);
PMPC_Base_PredBus.Elements  = make_elements(pred0);
end

function elements = make_elements(t)
%MAKE_ELEMENTS 由模板结构体的字段生成 BusElement 数组。
names = fieldnames(t);
for j = 1:numel(names)
    elements(j) = Simulink.BusElement;
    value = t.(names{j});
    elements(j).Name           = names{j};
    elements(j).Dimensions     = size(value);
    elements(j).DimensionsMode = 'Fixed';
    elements(j).Complexity     = 'real';
    if islogical(value)
        elements(j).DataType = 'boolean';
    else
        elements(j).DataType = class(value);
    end
end
end
