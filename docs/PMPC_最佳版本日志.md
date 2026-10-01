# PMPC 最佳版本日志

## 2026-10-01：道路可行性纵向协调版

- 当前结论：这是截至 2026-10-01 综合表现最好的 PMPC 控制器版本，后续修改以此为基准。
- 控制器基准提交：`f3c95f1 feat: coordinate PMPC braking with road feasibility`
- Git 分支：`codex/pmpc-longcoord`
- GitHub 分支：`origin/codex/pmpc-longcoord`
- 默认运行开关：`PMPC_LONGCOORD = 1`

### 核心配置

- 初始车速：80 km/h，由 CarSim Procedure 设置。
- PMPC 状态维数：7，包含减振器执行器时延状态。
- 固定执行器时延常数：`tau_d = 0.015 s`。
- 道路可行性协调器根据前方路径曲率和道路余量提前生成纵向制动需求。
- 持续弯道判据：`Long_min_sustain_m = 30 m`，用于排除 DLC 的短时曲率峰值，避免不必要的大幅减速。
- `PMPC_LONGCOORD = 0` 为原 PMPC 对照；`PMPC_LONGCOORD = 2` 为曲率限速消融模式。

### 80 km/h 验证结果

J-turn 指标统一统计 0–200 m，避免 PMPC 与 ZENG 停止站不同造成不公平比较。

| 工况 | 控制器 | max\|e_y\| (m) | e_y RMS (m) | 角点越界 (%) | 最低车速 (km/h) |
|---|---|---:|---:|---:|---:|
| DLC，mu=0.5 | 原 PMPC | 0.700 | 0.279 | 0.00 | 77.98 |
| DLC，mu=0.5 | 当前最佳 PMPC | 0.700 | 0.279 | 0.00 | 77.98 |
| DLC，mu=0.5 | ZENG | 0.910 | 0.293 | 5.45 | 68.57 |
| J-turn，R69、mu=0.85 | 原 PMPC | 4.469 | 2.348 | 59.33 | 75.16 |
| J-turn，R69、mu=0.85 | 当前最佳 PMPC | 1.398 | 0.671 | 31.37 | 73.14 |
| J-turn，R69、mu=0.85 | ZENG | 1.519 | 0.644 | 27.15 | 73.53 |

J-turn 中，当前 PMPC 的纵向制动需求约从 0.88 s 开始，请求峰值约 4.34 kN，实际分配总制动力峰值约 5.26 kN。相较原 PMPC，峰值横向误差降低约 68.7%，RMS 降低约 71.4%；DLC 性能没有退化。

### 验证状态

- MATLAB 相关回归测试：29 项通过，0 项失败。
- Simulink/CarSim 实际运行验证完成。
- 比较脚本：`compare_pmpc_longcoord`。
- 完整数据摘要：`results_pmpc_longcoord_v2/summary.md`。

### 已知限制

- J-turn 中当前 PMPC 的峰值误差优于 ZENG，但 RMS 和角点越界时间仍略差于 ZENG，尚不能认为道路约束已被完全满足。
- 当前没有记录每个控制周期的 QP 退出码和最坏计算耗时。
- 后续调参或修改结构后，必须同时复测 DLC 与 J-turn；如果综合结果变差，应以提交 `f3c95f1` 恢复控制器基准。
