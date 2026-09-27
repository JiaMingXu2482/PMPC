function mdl = func_SimModel(allPar)
%FUNC_SIMMODEL  从 CarSim 的展开结果里读出"该跑哪个 Simulink 模型"
%   mdl = func_SimModel(<Results\Run_xxx\Run_all.par>)   -> 'cq3_2019' / 'pmpc_mil' / ...
%
%   为什么要有这个函数:
%     模型路径由 CarSim 的 "Models: Simulink" 数据集决定(界面上那个
%     Simulink Model 输入框), 点 Send to Simulink 时会展开进 Run_all.par 的
%     SIMULINK_MODEL_FILE。而 run_ds 以前是**写死** sim('cq3_2019') 的 ——
%     CarSim 那边改指向、MATLAB 这边没改, 就会出现"以为在跑 pmpc_mil、
%     其实跑的是 cq3_2019"这种**静默错位**: 两个模型都含 CarSim S-Function、
%     都读同一个 simfile.sim, 跑起来不报任何错, 只是结果是另一个模型的。
%     所以这里直接跟着 CarSim 走, 从结构上消除两边不同步的可能。
%
%   读不到就退回 'cq3_2019' 并**明确警告** —— 宁可吵, 不要静默跑错。
%
%   编码: Run_all.par 是 GBK(路径里有中文)。只取basename, 全是 ASCII,
%   所以目录部分即使解码成乱码也不影响结果。

mdl = 'cq3_2019';
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
mdl = nm;
end
