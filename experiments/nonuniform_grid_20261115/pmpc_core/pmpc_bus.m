function [PMPC_MeasBus, PMPC_StateBus, PMPC_PredictionBus] = pmpc_bus(meas0,state0)
%PMPC_BUS 仅在 InitFcn 执行；由初始化模板创建固定尺寸的接口总线。
% 不放进 MATLAB Function，不参与控制拍内计算。
PMPC_MeasBus = Simulink.Bus;
PMPC_StateBus = Simulink.Bus;
PMPC_PredictionBus = Simulink.Bus;
nd=numel(state0.plan);
N=(numel(state0.active_main)-4*nd-10)/12;
prediction0=struct('Y',zeros(7*N,nd),'y',zeros(7*N,1), ...
    'AI',zeros(nd,nd),'Ut',zeros(nd,1),'valid',false);
% 按模板定义字段、固定尺寸及类型；活动集必须保持 boolean。
templates = {meas0,state0,prediction0};
for k = 1:3
    names = fieldnames(templates{k});
    for j = 1:numel(names)
        elements(j) = Simulink.BusElement;
        value = templates{k}.(names{j});
        elements(j).Name = names{j};
        elements(j).Dimensions = size(value);
        elements(j).DimensionsMode = 'Fixed';
        elements(j).Complexity = 'real';
        if islogical(value)
            elements(j).DataType = 'boolean';
        else
            elements(j).DataType = class(value);
        end
    end
    if k == 1
        PMPC_MeasBus.Elements = elements;
    elseif k == 2
        PMPC_StateBus.Elements = elements;
    else
        PMPC_PredictionBus.Elements = elements;
    end
    % 模板字段数不同，清空数组以免遗留上一个总线的字段。
    clear elements
end
end
