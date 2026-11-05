# 电流与力边界拆分（2026-11-04）

只拆原 inv_mf_impl_10ms.m，保留当前权重、归一化、特征、GRU、预热、加速度滤波、电流限幅、10 ms tick、两端电流10步平均力及CarSim符号。未改物理正模型。原组合函数及 inv_mf.m 保留，方便未改接线的模块继续调用；新接线不再调用它们。

## 两个新入口（每角各一个 MATLAB Function 块）

电流块：
```matlab
function I_cmd = fcn(Fdes,t,Temper,cmpD,cmpRD,cid)
%#codegen
I_cmd = inv_current_impl_10ms(Fdes,t,Temper,cmpD,cmpRD,cid);
end
```

边界块：
```matlab
function [Flo,Fhi] = fcn(t,Temper,cmpD,cmpRD,I_hist,cid)
%#codegen
[Flo,Fhi] = force_bounds_impl_10ms(t,Temper,cmpD,cmpRD,I_hist,cid);
end
```

两块都按原组合函数每1 ms调用一次；内部仍每10 ms更新电流/边界。不要直接把调用周期改成0.01，否则历史推进和加速度滤波不再等同原逻辑。PMPC本身仍为100 Hz。这里不承诺100 Hz硬件任务可以无代价执行原来的1 kHz历史计算，也未测运行时间。

所有端口double标量。cid固定为1=L1、2=L2、3=R1、4=R2；四角组合为[L1;L2;R1;R2]。单位t秒、Temper摄氏度、cmpD毫米、cmpRD毫米/秒、力N、电流A。两块使用同一时钟与运动信号。

## 接线

1. 旧每角逆块换成上面的电流块，仅输出 I_cmd；其电流到执行器的现有路径不变。
2. 新建边界块，不接 Fdes。其 Flo/Fhi 按四角顺序分别 Mux 后送 PMPC。
3. 每角 I_cmd 另分一条历史支路，经过 Unit Delay（采样时间0.001、初值0）送本角边界块的 I_hist。只放历史支路，不串在执行器路径，不改成0.01。
4. 删除 Flo/Fhi 到 PMPC 支路上原来用于打断反馈的0.01 s Unit Delay。控制器跨拍 state 的0.01 s Unit Delay 保持原样。
5. 边界块及其历史寄存与电流块应分成独立子系统；边界子系统只接历史电流和运动，不接本拍 Fdes/I_cmd。不要再次将二者包进同一个原子子系统并经该原子块反馈边界，以免重建人工代数环。PMPC的100 Hz及原测量速率转换保留；本次没有设置跨速率多任务通信策略。
6. F_clipped/F_ach 是原组合函数的附加输出，新电流/边界两个入口不提供。它们未参与原电流求解或边界计算；若旧Scope接了这两路，先断开对应观测线。不要把 F_ach 接成实际力反馈。

信号顺序：
```
运动、温度、时间 + I_hist -> 边界 -> Flo/Fhi -> PMPC -> Fdes -> 电流块 -> I_cmd -> 执行器
                           ^                                             |
                           +------ 历史支路 Unit Delay(1 ms, IC=0) -------+
```

## 为什么历史支路有1 ms寄存

原函数先用进入函数时的 I_holds 推进试算历史，最后才写入本次新 I_cmd。拆开后，I_hist就是该旧 I_holds：在第n次1 ms调用时取上一次调用的命令。不是另加物理电流时延，也不是让 Flo/Fhi 晚一拍。不能直接把本拍 I_cmd 接回边界块，也不能擅自改为实测电流，否则不再保持原历史计算口径。

边界代码使用独立的持久试算状态，不读取真实部件或仿真plant正模型的隐藏状态。候选试算使用副本，不重复推进持久状态。与原代码一样，Flo/Fhi是冻结当前运动、候选0/1.6 A的10 ms平均力范围；本次按用户要求不改成稳态力范围、不修改它与MPC等效Md滞后的建模关系。

## 初始化与状态

确认 nlcsnn 已 addpath。切换接线后从仿真初始时刻重新开始，不从旧运行状态续跑。解释执行前可用：
```matlab
clear inv_current_impl_10ms force_bounds_impl_10ms
```
Simulink模块各自保存持久状态；不要同时让旧组合块与新块都驱动同一个电流或边界信号，也不要为同一角重复调用新入口。此处与原函数一样按每基础采样调用一次设计。

只完成代码拆分和接线说明，没有修改SLX、运行编译/仿真或做数值等价测试；不能据此宣称闭环结果相同。给定相同Fdes、运动和正确的I_hist序列时保留原计算关系；移除边界10 ms延迟后，闭环命令本来就可能变化。
