function [sys, F_des, long_diag] = pmpc_block(u, F_lo, F_hi, PMPC_P) %#codegen
%PMPC_BLOCK  MATLAB Function block 的全部内容(块里只写一行调用它)
%   u      : 54x1 CarSim 量测 (顺序见 func_StateEstimation)
%   F_lo/F_hi: external damper interface's attainable CarSim-force bounds
%     (4x1, controller corner order [L1;R1;L2;R2], N)
%   PMPC_P : 参数, 块里声明为 Scope=Parameter 且 Tunable=false
%            (不可调 = 块层面的 coder.Constant; 否则 nvars 这类维度
%             推不出常量, 块会报"无法确定输出大小")
%            pmpc_mil 固定使用 ControllerVariant=3 和七状态预测模型。
%   sys    : 54x1 输出
%   F_des  : 4x1 desired CarSim-polarity damper force [N]; inverse is external
%   long_diag: 20x1 priority/terminal diagnostics for simulation logging only
%     [delta1 delta2 F w1 w2 certA certB s1 s2 gammaDB exitflag fallback]
%     followed by eight terminal-construction/membership diagnostics.
%
%   跨拍状态用 persistent —— 生成代码里就是一个静态变量。
%   首拍用 PMPC_P.S0 初始化, 与 S-function 里 InitialParams 的初值一致。
%
%   This block never calls the NLCSNN forward or inverse. Its external
%   damper interface supplies force bounds and converts F_des to current.

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
% In the external-twin specialization, the legacy x_plant/v_plant slots
% carry the attainable lower/upper force bounds; no internal plant state
% is used by PMPC. Other controller variants keep their original contract.
[sys, St, F_des] = pmpc_step(uc, PMPC_P.Pm, St, ...
                             zeros(217,4), reshape(F_lo,4,1), ...
                             reshape(F_hi,4,1), zeros(4,1));
long_diag = func_PMPCTerminalDiagnostics(St.cert);
end
