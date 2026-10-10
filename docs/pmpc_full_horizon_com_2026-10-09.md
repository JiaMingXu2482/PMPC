# PMPC 全预测域约束 COM 对照（2026-10-09）

## 范围与结论

两次隔离 CarSim–Simulink 联仿均完成至 19.999 s。只将稳定性及 LTR
约束节点从 1:5 扩展至 1:10；Np=10、Nc=5、Ts=0.1 s 不变，道路
约束本来就是 1:10。未增加 Vx 状态、纵向限速或改变调参。

仅扩大约束时域没有解决 PMPC 的 J-turn 跟踪与 DB 协调问题，不采用为默认配置。
正式 setup_pmpc.m、调参文件、SLX 和 CarSim LastRun 均未被本实验保存覆盖。

## 输入与留档

- 道路输入：scratchpad/com_r75_db_20261008/PMPC_before/Run_all.par，实际 R=75 m、入弯 X=330 m。
- 数据集名称仍含 R70，不能用名称判断实际半径。
- 基线目录：scratchpad/com_r75_db_20261008/horizon_20261009_164537_baseline/Results/Run_trial
- 实验目录：scratchpad/com_r75_db_20261008/horizon_20261009_164537_full_horizon/Results/Run_trial
- 各目录保存 parameters.mat、pmpc_mil_logs.mat、LastRun.vs/vsb、Run_all.par、trial.sim、report.mat。
- 汇总：scratchpad/com_r75_db_20261008/horizon_20261009_164537_comparison.mat。
- 运行入口：scratchpad/com_r75_db_20261008/run_horizon_comparison.m。

## 本次实测

| 指标 | 基线 | 全预测域 |
|---|---:|---:|
| DLC ey 峰值 / m | 0.834303 | 0.841788 |
| DLC ey RMS / m | 0.294122 | 0.297859 |
| J-turn ey 峰值 / m | 0.610504 | 0.580109 |
| J-turn ey RMS / m | 0.435977 | 0.430123 |
| DLC Vx RMS / km/h | 88.628 | 88.625 |
| J-turn Vx RMS / km/h | 77.932 | 78.037 |
| 全程绝对 LTR 峰值 | 0.6555 | 0.6580 |

以上跟踪分段按 func_EyConsoleSummary 的道路站距规则计算，不混用历史截图结果。
J-turn RMS 仅降低约 1.34%，DLC RMS 增加约 1.27%。

附加控制诊断沿用 analyze_db_trial 的固定时间窗（不是精确道路分段）：

| 指标 | 基线 | 全预测域 |
|---|---:|---:|
| t<=8 s 方向盘相邻采样增量 RMS / deg | 2.1888 | 2.6276 |
| t>=12.5 s 方向盘相邻采样增量 RMS / deg | 1.8498 | 1.8008 |
| t>=12.5 s exitflag~=1 比例 | 12.1333% | 13.8667% |
| t>=12.5 s abs(Mb)>50 Nm 比例 | 72.8% | 72.4% |

后段不是完全没有 DB，但本试验没有使其产生明显更好的控制效果。
求解非成功比例未改善；不能把当前现象全部归因于预测时域不足。
后续应把预测风险对 DB 的有效作用及求解数值稳定性作为一个整体修改并端到端对照，
而不是继续只增加约束节点或不断加大惩罚权重。此试验不构成严格优先级理论证明。
