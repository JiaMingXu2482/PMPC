function mdl = func_SimModel(allPar, expectedModel)
%FUNC_SIMMODEL  Verify an explicit CarSim/Simulink model pairing.
%   expectedModel is optional for legacy callers and defaults to pmpc_mil.

if nargin < 2 || isempty(expectedModel)
    expectedModel = 'pmpc_mil';
end
validModels = {'mpc_mil','zeng_mil','pmpc_mil','pmpc_nodelay_mil'};
if ~any(strcmp(expectedModel, validModels))
    error('func_SimModel:InvalidExpectedModel', ...
        'expectedModel must be mpc_mil, zeng_mil, pmpc_mil, or pmpc_nodelay_mil.');
end
mdl = expectedModel;
if exist(allPar,'file') ~= 2
    warning('func_SimModel:NoPar', '找不到 %s, 退回默认模型 %s', allPar, mdl);
    return
end
t  = fileread(allPar);
tk = regexp(t, '(?m)^SIMULINK_MODEL_FILE\s+(.+?)\s*$', 'tokens', 'once');
if isempty(tk)
    warning('func_SimModel:NoField', ...
            '%s 里没有 SIMULINK_MODEL_FILE, 退回默认模型 %s', allPar, mdl);
    return
end
[~, nm] = fileparts(strtrim(tk{1}));
if isempty(nm)
    warning('func_SimModel:BadPath', 'SIMULINK_MODEL_FILE 解析不出模型名, 退回 %s', mdl);
    return
end
if ~strcmp(nm, mdl)
    error('func_SimModel:WrongModel', ...
        'CarSim 数据集指向模型 %s；当前控制器需要 %s，请修改后重新 Send to Simulink。', ...
        nm, mdl);
end
end
