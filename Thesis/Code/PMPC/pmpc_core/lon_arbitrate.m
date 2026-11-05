function [throttle, p_brake, u_applied] = lon_arbitrate(u_lon, Tb) %#codegen
%LON_ARBITRATE  纵向速度环与 DYC(差动制动) 的协调。放在 100 Hz 控制器子系统里。
%
%   输入  u_lon   速度 PID 的输出，已限幅到 [-1, 1]（>0 要驱动，<0 要制动）
%         Tb      PMPC 输出的四轮制动力矩 [N*m]，4x1，非负
%   输出  throttle   油门 [0,1]            -> CarSim IMP_THROTTLE_ENGINE
%         p_brake    制动主缸压力           -> CarSim IMP_PCON_BK（单位以你的 CarSim 变量为准）
%         u_applied  实际生效的归一化指令    -> 经 Unit Delay 接 PID 块的 Tracking 口，防积分饱和
%
%   规则（稳定性优先于速度保持）:
%     1) DYC 激活时：速度环不制动；油门不超过激活瞬间的值
%     2) DYC 解除后：油门上限按斜率逐步放开，不一步跳回 PID 输出
%     3) 未激活时：u_lon>0 给油门，u_lon<-DEAD 给制动，中间死区两者都不给

% ---- 可调参数（示例值，需要按工况调整；稳定后可移到 pmpc_init 的 params 里）----
DT      = 0.01;      % 执行周期 [s]，必须等于所在子系统的采样时间
TB_ON   = 50;        % ΣTb 超过它，认为 DYC 激活 [N*m]
TB_OFF  = 10;        % ΣTb 低于它，才开始计时退出 [N*m]
HOLD_T  = 0.2;       % 低于 TB_OFF 后继续保持的时间 [s]
RATE_UP = 0.5;       % 解除后油门上限的放开斜率 [1/s]
DEAD    = 0.02;      % 驱动/制动切换死区
P_BK_MAX = 6;        % u_lon=-1 对应的主缸压力，需按你的 CarSim 制动系统核对

persistent active hold thr_cap thr_prev
if isempty(active)
    active = false;  hold = 0;  thr_cap = 1;  thr_prev = 0;
end

% ---- DYC 是否激活（带迟滞）----
if all(isfinite(Tb(:)))
    sTb = sum(max(Tb(:), 0));
else
    sTb = 0;
end
if sTb > TB_ON
    if ~active
        thr_cap = thr_prev;          % 激活瞬间锁定油门上限
    end
    active = true;
    hold   = HOLD_T;
elseif active
    if sTb < TB_OFF
        hold = hold - DT;
        if hold <= 0
            active = false;
        end
    else
        hold = HOLD_T;               % 介于 TB_OFF 与 TB_ON 之间：保持激活
    end
end

% ---- 速度环请求 -> 油门/制动 ----
u_lon   = min(max(u_lon, -1), 1);
thr_req = max(u_lon, 0);
brk_req = max(-u_lon - DEAD, 0);

if active
    brk = 0;                                   % 制动只给 DYC
    thr = min(thr_req, thr_cap);               % 油门不超过锁定值
else
    brk = brk_req;
    thr_cap = min(thr_cap + RATE_UP*DT, 1);    % 解除后上限逐步抬高
    thr = min(thr_req, thr_cap);
end

thr_prev  = thr;
throttle  = thr;
p_brake   = brk * P_BK_MAX;
u_applied = thr - brk;
end
