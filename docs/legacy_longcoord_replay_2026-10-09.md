# 30 m 持续弯道纵向协调版：COM / Slalom 复跑

> 2026-10-09 更正：下面首次 COM 复跑使用的是后来调整的 R75、330 m 入弯道路，不能用于复现 2026-10-02 历史 R70、280 m 入弯结果。进一步核对发现：2026-10-02 另有自动生成的 R70 道路，其布局也不同于取得较好成绩的 CarSim 手工分段道路。现以保存的手工 R70 展开文件建立 `simulation_results/legacy_longcoord_manual_R70_20261009` 做配对复跑。首次 R75 图表保留为独立试验；自动生成 R70 的未完成复跑停在 `simulation_results/legacy_longcoord_original_R70_20261009`，均不作为旧手工道路的复现结论。`4fb1637` 也早于正式 Slalom 路径接入；本文中的 Slalom 使用了后来的路径识别适配，属于移植测试。

## 版本定位

- 基准：`4fb1637`，`feat: stabilize PMPC speed control in combined maneuver`。
- 原始 Git 快照：`scratchpad/legacy_longcoord_4fb1637.zip`。
- 隔离运行目录：`scratchpad/legacy_longcoord_4fb1637`。
- 当前主线的控制器、调参文件和 SLX 未由本次历史复跑覆盖。
- `config/Atuning_pmpc.m`：`LongCoordMode=1`、`Long_min_sustain_m=30`。
- `controller/func_PMPCSpeedCoordinator.m`：持续超出曲率安全车速的预览区间达到 30 m，才保留该区间的曲率限速，再反向计算制动距离。30 m 是持续距离，不是转弯半径，也不是无条件制动开关。严重实测道路余量损失时，旧实现可将持续距离降至 15 m；本次保持原样。
- 旧版 ZENG 同样保留原有 `ZengLong_on=1`，不是将 ZENG 的纵向能力人为关闭。
- 正逆减振器模型均用该提交自带的旧 NLCSNN，不与当前 v14 混用。

## 工况与隔离方式

本次是“历史控制器在当前道路上的复跑”，不是原 R70 工况逐字节复现。

- COM：初始 90 km/h，DLC 后摩擦系数由 0.5 切换到 0.85，J-turn 起点 330 m、半径 75 m，仿真 20 s。
- Slalom：初始 70 km/h、摩擦系数 0.85，三组均使用相同 CarSim 展开文件，停止站 160 m、最大时间 20 s。
- 同一工况三控制器的车辆、道路和输入文件一致，只改变控制器模型与标签。
- 使用当前项目生成的准确参考路径覆盖历史初始化中的参考路径；不改控制算法、权重或纵向协调参数。
- 历史数据集解析器添加 Slalom 标签识别，避免原始数据集名称中的 `mu1` 被误认为真实路面附着（展开文件实际为 0.85）。
- CarSim 输出重定向至 `simulation_results/legacy_longcoord_20261009/Results/Run_<工况>_<控制器>`。不覆盖 GUI 当前 Run 的 LastRun。

## 保存内容

每组目录包含 `Run_all.par`、CarSim `LastRun.vs/.vsb`、`parameters.mat`、`*_logs.mat` 和完成标记 `COMPLETE.mat`。PMPC 另保存纵向协调诊断（模型提供时）。

后处理入口：`scripts/plot_legacy_longcoord_replay.m`。输出各组 `vehicle_signals.mat/.csv`、总指标 `metrics_common_station.mat/.csv`，以及 `figures` 中 PNG / 矢量 PDF / 可编辑 FIG。

图包括：速度、横向误差、LTR、分段 beta-r 相图及四指标菱形图。COM 分 DLC / J-turn 统计；Slalom 单独统计。

## 比较口径

- 误差为质心相对共同参考折线的有符号法向距离。
- 仅比较三控制器共同覆盖的参考里程；COM 的 DLC 截至 200 m，J-turn 从 330 m 开始。
- 主 RMS 用相同路段内的原始等时间采样计算，同时附上 0.1 m 等距离重采样 RMS，方便核查速度差带来的采样权重影响。
- 菱形图：误差轴采用最小值 / 本组值；速度轴采用本组值 / 最大值，外侧表示更小误差或更多速度保留。速度保留不等同安全性能，不以多边形面积定义总体优劣。
- 这次允许各控制器自己的纵向控制，因此结果反映完整控制策略表现，不能单独归因于横向优先级机制。
- 历史日志中的 R70 PMPC 优秀结果没有同次完整重跑 MPC/ZENG 的记录，不能先验宣称本次所有指标必然最优。

## 本次实测结果（六组均已完成）

以下为共同路段上的横向误差 RMS，单位 m；完整峰值、速度、LTR 与等距离 RMS 在指标 CSV 中。

| 路段 | MPC | ZENG | PMPC |
|---|---:|---:|---:|
| COM / DLC | 0.355614 | 0.275332 | 0.283467 |
| COM / J-turn | 0.071849 | 0.068698 | 0.182996 |
| Slalom | 0.462444 | 0.447450 | 0.470514 |

- COM / DLC：PMPC 的误差优于 MPC，但略差于 ZENG；PMPC 的速度 RMS 为 88.667 km/h，ZENG 为 83.087 km/h，速度差明显。
- COM / J-turn：PMPC 纵向协调制动力请求峰值 2700.4 N，确实介入；但横向跟踪仍不如另两组。PMPC 的速度 RMS 为 76.641 km/h，MPC 为 77.747 km/h，ZENG 为 77.430 km/h。
- Slalom：PMPC 纵向协调制动力请求全程为 0；不能因此认为无差动制动。PMPC 的横向误差 RMS 略大于另两组，LTR 峰值约 0.917（MPC 0.853，ZENG 0.850）。
- 本次不能得出“恢复该版本后 COM、Slalom 全面优于另两控制器”的结论，也不能因联仿跑完就称所有安全约束满足。
- COM 均运行至 19.999 s；Slalom 均由同一 160 m driver station 限制正常终止，停止时间 MPC 8.493 s、ZENG 8.603 s、PMPC 8.408 s。保存采样的末时刻可能略早于求解器停止时刻。
- PMPC 纵向诊断图中的 `Total allocated braking` 是分配器总制动力（含 DB），不是纯纵向协调请求的实现值；初始化时刻 t=0 的未初始化速度上限仅在绘图中排除，原始日志不改。

## 历史手工 R70 工况复现（修正后的配对结果）

使用 `simulation_results/archive/results_pmpc_comb_fix/20261003_092605/LastRun_all.par` 保存的 CarSim 手工分段道路输入：直线 280 m、R70 左转，初始车速 90 km/h，三组同一展开输入。恢复的控制器仍为 Git `4fb1637`，PMPC `LongCoordMode=1`、`Long_min_sustain_m=30`。三个新 `Run_all.par` 除数据集标签和 Simulink 模型文件以外的字节一致（归一化 SHA-256 一致）。本次所有三个模型均完成 19.999 s 联仿。

按原历史版本的 `func_EyConsoleSummary` 统计，以便和 2026-10-02 日志逐项比对：

| 路段 | 控制器 | max\|e_y\| (m) | e_y RMS (m) | Vx RMS (km/h) |
|---|---|---:|---:|---:|
| DLC | MPC | 1.008707 | 0.356887 | 88.008 |
| DLC | ZENG | 0.726614 | 0.276260 | 83.060 |
| DLC | PMPC | 0.789390 | 0.284490 | 88.665 |
| J-turn | MPC | 2.982056 | 1.327494 | 76.802 |
| J-turn | ZENG | 2.365082 | 1.020657 | 76.340 |
| J-turn | PMPC | 0.327497 | 0.141965 | 76.104 |

PMPC 四个误差数字及两个 Vx RMS 均逐位复现旧日志。本次新图表另采用三者共同里程，指标见 `simulation_results/legacy_longcoord_manual_R70_20261009/metrics_common_station.csv`，所以 RMS 有轻微差异；原始旧口径结果另存 `metrics_legacy_console.mat`。

结论：初次 R75/330 m 复跑未复现旧成绩，主要由道路不一致导致；改回原手工 R70/280 m 后复现。PMPC 在 J-turn 峰值与 RMS 上明显优于 MPC/ZENG；DLC 则优于 MPC、略逊于 ZENG，但 ZENG 的速度 RMS 比 PMPC 低约 5.6 km/h。因此“PMPC 两段的误差都严格优于 MPC 和 ZENG”不是这份历史数据支持的表述。旧版 Slalom 使用现行参考路径适配，单独存放在首次输出目录，其比较也不能当成 2026-10-02 旧版原样复现。
