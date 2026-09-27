function [Fd, s_act] = func_DamperActuator(Fd_cmd, Fdl, Fdu, s_act, Ts, tau) %#codegen
%FUNC_DAMPERACTUATOR  被控对象侧的阻尼器响应时延(执行器模型, 2026-09-25)
%   [Fd, s_act] = func_DamperActuator(Fd_cmd, Fdl, Fdu, s_act, Ts, tau)
%
%   Fd_cmd : 4x1 分配层给出的指令阻尼力 [fl; fr; rl; rr] (N)
%   Fdl/Fdu: 4x1 当前速度下最小/最大电流对应的力 (与 func_CostWeighting 同序)
%   s_act  : 4x1 状态, 力在上下界间的相对位置 (0 = 下界, 1 = 上界); <0 = 未初始化
%   Ts     : 执行周期 (s),  tau : 响应时间常数 (s)
%   Fd     : 4x1 实际施加到 CarSim 的阻尼力 (N)
%
%   为什么滞后作用在 s 上而不是力上:
%     物理上滞后的是线圈电流/阀开度, 即"力在当前可达区间里的位置"; 力本身
%     由当前活塞速度和阀开度共同决定。直接对力做一阶滞后, 速度过零或反向后
%     会保持旧方向的力 —— 不满足半主动耗散性(审稿意见指出的正是这一点)。
%     对 s 滞后后 Fd = F_lo(v) + s*(F_hi(v) - F_lo(v)) 始终落在当前区间内,
%     速度为零时区间退化为 {0}, 输出力自然为零。
%   与预测模型的关系: 速度在时域内冻结时, 对 s 的滞后与论文式 damper_delay
%     对 Md 的一阶滞后等价(Md 对 s 是仿射的)。
%
%   这是被控对象的一部分, 不属于控制器; 放在 pmpc_step 输出端只是为了不改 .slx。
%   开关 Constraints.dmp_act_on, 关掉即恢复"指令力当拍直达 CarSim"。

lo = min(Fdl(:), Fdu(:));
hi = max(Fdl(:), Fdu(:));
sp = hi - lo;

s_cmd = zeros(4,1);
for i = 1:4
    if sp(i) > 1e-6
        s_cmd(i) = min(max((Fd_cmd(i) - lo(i)) / sp(i), 0), 1);
    else
        s_cmd(i) = s_act(i);          % 区间退化(速度~0): 阀开度不可观, 保持
    end
end

if any(s_act < 0)                     % 首拍: 以指令为初值, 不引入起步瞬态
    s_act = s_cmd;
    for i = 1:4
        if s_act(i) < 0, s_act(i) = 0; end
    end
end

a     = exp(-Ts / tau);
s_act = a*s_act + (1 - a)*s_cmd;
Fd    = lo + s_act .* sp;
end
