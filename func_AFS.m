function [delta_out, delta_wheel] = func_AFS(VehiclePara, VehStateMeasured, MPCParameters, ParaHAT, delta_total)
% func_AFS  前轮转角指令 -> 方向盘转角（AFS 控制量已改为 Delta-delta）
% -------------------------------------------------------------------------
% 上层给出的是转角修正 Ddelta，主函数已合成 delta_total = delta_robot + Ddelta。
% 本函数只做单位换算与机械限幅，不再需要 Fiala 逆解
% （实测转角粗糙度 100% 来自那一步逆解，且执行器速率约束无法在力空间精确表达）。
% -------------------------------------------------------------------------
isw = VehiclePara.isw;
delta_wheel = delta_total;                                       % rad @前轮
delta_sw    = max(min(isw*delta_wheel, deg2rad(720)), -deg2rad(720));
delta_out   = rad2deg(delta_sw);                                 % deg @方向盘
end
