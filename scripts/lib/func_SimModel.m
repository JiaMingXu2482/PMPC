function mdl = func_SimModel(allPar)
%FUNC_SIMMODEL  Verify that a CarSim run is configured for pmpc_mil.
%   PMPC now has one Simulink model only. A stale CarSim dataset that still
%   points elsewhere is rejected explicitly instead of running the wrong model.

mdl = 'pmpc_mil';
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
        'CarSim 数据集仍指向模型 %s；请改为 pmpc_mil 后重新 Send to Simulink。', nm);
end
end
