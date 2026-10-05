# 共用纵向控制链路清理设计（2026-10-05）

## 目标与边界

当前项目为后续 PIL/S32K144 生成代码保留四个控制器：MPC、ZENG、PMPC、PMPC-noDelay。四者继续使用现有速度 PID、COM 工况的共用 `func_CombinedSpeedReference` 和 `func_QPA_DB` 纵向制动分配；只移除已关闭的控制器专属纵向策略和过时的 8×4 `Vx/Fx` 实验分支。算法的横向、侧倾、减振器时延设计及调参值不变。回退版本为用户指定的 `Simulink_Model_10_05`，历史 8×4 实验亦保留在 Git 标签。

## 方案选择

- 采用：删除休眠策略及其配置、状态和仅服务该策略的函数，同时维持 Simulink 对外端口形状。这样可以让当前控制代码和生成代码都不再携带专属纵向状态。
- 不采用：只隐藏 `Atuning_*` 中的参数。该方法不会移除 `pmpc_step` 的分支、`LongCoord` 跨拍状态及代码生成依赖。
- 不采用：重新构建一个 Simulink Library 速度控制器并替换四个模型的 PID 子系统。那会引入与本次清理无关的模型接口风险。

## 具体修改

1. 删除 `Atuning_pmpc.m`、`Atuning_pmpc_nodelay.m` 的 `LongCoordMode/Long_*` 参数，以及 `Atuning_zeng.m` 中已关闭的 `ZengLong_on/Zg_*` 纵向限速参数；保留 `rho` 稳定性调权本身。
2. `setup_pmpc.m` 不再生成上述约束字段、纵向专属预瞄参数和 `S0.LongCoord` 状态。保留执行器制动上限、车辆参数、共同速度目标及原有四种横向控制器配置。
3. `pmpc_step.m` 删除 PMPC 纵向预瞄、可控裕度回授、ZENG 预测横摆限速和 8×4 总纵向制动力决策分支。所有控制器只从 `func_CombinedSpeedReference` 获取 COM 共用车速目标与服务制动请求，其余工况沿用现有速度 PID 目标；差动制动请求仍由 QP 生成并由 `func_QPA_DB` 分配。
4. 删除只被休眠策略使用的协调器、证据计算器及 8×4 辅助函数；`run_combined_carsim.m` 不再强行设置 `PMPC_LONGCOORD`，仍使用共用 COM 车速调度。清理专属诊断/对照脚本和已失效的测试，替换为共享纵向链路的回归测试。历史论文/实验日志保留并标记为旧版本记录。
5. `pmpc_block.m` 与 `pmpc_nodelay_block.m` **保留第三个 `long_diag` 输出端口，固定输出 `zeros(12,1)`**。现有 Simulink 连线和 Scope 可继续加载；不改变 54 路 CarSim 输入输出或四角电流接口。模型内若有仅为休眠策略服务的 PID 第三输入，改接常数零而不改变 PID 本身的油门逻辑。

## 验收与风险

- 静态检查：当前控制路径不再引用 `LongCoordMode`、`ZengLong_on`、`Long_*`、`Zg_*` 或 8×4 专属函数；`long_diag` 在两个 PMPC 变体中始终是 12×1 零向量。
- MATLAB/Simulink：四个 `setup_pmpc` 变体可初始化；四个模型可更新图；针对共用速度目标、制动分配和诊断尺寸的测试通过。完整测试集的旧实验断言需逐项清理，不能仅因删除测试而宣称全通过。
- 对照：在相同 CarSim Run 和参数下抽查 COM、J-turn、Slalom 的关键输出，确认清理前后没有非预期行为改变。实际 S32K PIL 构建与时序验证另行执行，不由桌面测试替代。
- 主要风险是旧模型或脚本暗含 `LongCoord` 字段依赖，以及 8×4 分支删除时误伤现用 7×3 路径；通过引用扫描、模型更新和工况回归逐层发现。
