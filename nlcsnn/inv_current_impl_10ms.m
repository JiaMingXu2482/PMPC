function I_cmd = inv_current_impl_10ms(Fd_N, t, Temp_C, cmpD, cmpRD, cid, F_at0, F_at16)
%INV_CURRENT_IMPL_10MS 力界线性插值反解减振器电流（替代原逆 NN）。
%#codegen
% 每角调用一次 / 1 ms；电流每 10 ms 更新一次，间隔内保持。
%
% 输入：
%   Fd_N   期望阻尼力 [N]，CarSim 符号（压缩为正）
%   t      仿真时间 [s]
%   Temp_C 油温 [degC]，保留接口
%   cmpD   减振器位移 [mm]，保留接口
%   cmpRD  减振器速度 [mm/s]，保留接口
%   cid    1=L1, 2=L2, 3=R1, 4=R2
%   F_at0  当前速度下 0 A 对应的 10 ms 平均力 [N]
%   F_at16 当前速度下 1.6 A 对应的 10 ms 平均力 [N]
%
% 输出：
%   I_cmd  电流指令 [A]，范围 [0, 1.6]
%
% 反解律：F(v,I) 在 [F_at0, F_at16] 上近似线性，
%   I = 1.6*(Fdes - F_at0)/(F_at16 - F_at0)
% 单调、端点精确，不依赖神经网络。
% 力界来自 force_bounds_impl_10ms（正模型 fwd_mf_impl），与正模型一致。
%
% 为什么不用逆 NN：V23-B 报告实测逆 NN 对 F_des 的控制权限只有满量程 5.4%，
% 82% 样本违反单调性，本质是"保持上一拍电流"。插值版把闭环力误差从
% 120-190 N 降到 40-90 N。

MAX_I = 1.6;
NC = 4;
% 0 = 力界线性插值（默认）；2 = SH 全硬（恒 0 A，用于开关 CDC 对照）。
SH_MODE = 0;    % 0=力界插值（默认）；2=固定电流基线（值见 SH_I）
SH_I    = 1.0;  % SH_MODE==2 时的恒定电流 [A]
% 电流每 10 ms 最大变化 [A]。插值在低速度时力跨度很小，逆解对力极敏感，
% 不加限幅会在 0/1.6 A 间跳变。1.6 等价于不限幅。
MAX_DI = 1.6;   % 1.6 = 不限幅（实车电流 0-1.6 A 阶跃 <10 ms）

persistent last_ticks I_holds
if isempty(last_ticks)
    last_ticks = -ones(1, NC);
    I_holds = zeros(1, NC);
end
cid = max(1, min(NC, round(cid)));

tick = floor(t / 0.01 + 1e-9);
if tick ~= last_ticks(cid)
    last_ticks(cid) = tick;
    if SH_MODE == 2
        I_holds(cid) = SH_I;         % 固定电流基线（SH_I=0 即全硬）
    else
        den = F_at16 - F_at0;
        if isfinite(Fd_N) && isfinite(F_at0) && isfinite(F_at16) && abs(den) > 1e-6
            I = MAX_I * (Fd_N - F_at0) / den;
            I = min(max(I, 0), MAX_I);
            I = min(max(I, I_holds(cid)-MAX_DI), I_holds(cid)+MAX_DI);
            I_holds(cid) = I;
        end
    end
end
I_cmd = I_holds(cid);
end
