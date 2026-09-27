function sys = pmpc_block(u, PMPC_P) %#codegen
%PMPC_BLOCK  MATLAB Function block 的全部内容(块里只写一行调用它)
%   u      : 50x1 CarSim 量测 (顺序 = CarSim Export 列表, 见 func_PortMap)
%   PMPC_P : 参数, 块里声明为 Scope=Parameter 且 Tunable=false
%            (不可调 = 块层面的 coder.Constant; 否则 nvars 这类维度
%             推不出常量, 块会报"无法确定输出大小")
%   sys    : 54x1 输出
%
%   跨拍状态用 persistent —— 生成代码里就是一个静态变量。
%   首拍用 PMPC_P.S0 初始化, 与 S-function 里 InitialParams 的初值一致。

%  Simulink 传进来的是**一维** 50 元素向量, 而 pmpc_step 按列向量写的。
%  归一化方向, 否则块推不出输出维度。
uc = u(:);

persistent St
if isempty(St)
    St = PMPC_P.S0;
end
[sys, St] = pmpc_step(uc, PMPC_P.Pm, St);
end
