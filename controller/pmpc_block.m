function [sys, i_cmd] = pmpc_block(u, h_plant, x_plant, v_plant, a_plant, PMPC_P) %#codegen
%PMPC_BLOCK  MATLAB Function block 的全部内容(块里只写一行调用它)
%   u      : 54x1 CarSim 量测 (顺序见 func_StateEstimation)
%   h_plant/x_plant/v_plant/a_plant: 1 kHz plant 在拍首采样的状态
%     (控制器角序 [L1;R1;L2;R2]; h_plant 8x4, 其余 4x1; 经 Rate Transition 进来)
%   PMPC_P : 参数, 块里声明为 Scope=Parameter 且 Tunable=false
%            (不可调 = 块层面的 coder.Constant; 否则 nvars 这类维度
%             推不出常量, 块会报"无法确定输出大小")
%            pmpc_mil 固定使用 ControllerVariant=3 和七状态预测模型。
%   sys    : 54x1 输出
%   i_cmd  : 4x1 电流指令 [A], 控制器角序; 经 Rate Transition ZOH 送 plant
%
%   跨拍状态用 persistent —— 生成代码里就是一个静态变量。
%   首拍用 PMPC_P.S0 初始化, 与 S-function 里 InitialParams 的初值一致。
%
%   多速率 (2026-09-29): NLCSNN 正模型搬到 1 kHz plant 子系统(func_NLCSNNPlant4),
%   本块只做 100 Hz 逆解, 不再推进隐状态 h。

%  Simulink 传进来的是**一维** 54 元素向量, 而 pmpc_step 按列向量写的。
%  归一化方向, 否则块推不出输出维度。
if numel(u) ~= 54
    error('pmpc_block:InvalidInputSize', ...
        'Expected 54 CarSim export channels, received %d.', numel(u));
end
if PMPC_P.Pm.MPCParameters.ControllerVariant ~= 3
    error('pmpc_block:WrongVariant', 'pmpc_block requires ControllerVariant=3.');
end
uc = u(:);

persistent St
if isempty(St)
    St = PMPC_P.S0;
end
[sys, St, i_cmd] = pmpc_step(uc, PMPC_P.Pm, St, ...
                             h_plant, x_plant, v_plant, a_plant);
end
