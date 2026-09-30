# MPC、ZENG、PMPC 独立模型与时延状态设计

## 目标

把当前由一个 `pmpc_mil.slx` 和运行时模式开关承载的三种控制器拆成三个明确的 CarSim/Simulink 入口：

- `mpc_mil.slx`：固定权重 MPC，预测模型为 6 状态，不考虑聚合减振器执行时延；
- `zeng_mil.slx`：ZENG rho 调权控制器，预测模型为 7 状态，考虑聚合减振器执行时延；
- `pmpc_mil.slx`：PMPC，预测模型为 7 状态，考虑聚合减振器执行时延。

三种控制器都继续使用相同的 NLCSNN 正模型、逆模型和半主动耗散约束。拆分只改变控制器入口、模式专属参数和预测模型维数，不复制车辆参数、状态估计、路径规划、轮胎模型、约束构造或 NLCSNN 基础实现。

## 当前问题

当前已提交版本为保持同一 QP 尺寸，三个控制器都使用 7 状态：

```text
[Vy, r, phi, phidot, ey, epsi, Md_a]
```

启用 NLCSNN 时，聚合状态 `Md_a` 的时间常数被设为 `1e-4 s`。因此第 7 状态实际上只是近瞬时的尺寸占位，并未有意义地预测执行时延。

当前工作树已有未完成的变体草稿：`setup_pmpc.m` 能产生 `Nx=6/7`，但 `pmpc_step.m` 仍固定拼接 7 状态初值，`func_DynamicalModel.m` 仍无条件调用 `local_delay`，三个独立 SLX 和模式入口也尚未建立。实现必须接续这些改动，不覆盖已有 NLCSNN 多速率工作。

## 总体架构

### 独立顶层模型

三个模型均由当前工作的 `pmpc_mil.slx` 派生，并保留下列共同拓扑：

- 54 路 CarSim 输入；
- 100 Hz 控制器；
- 100 Hz 电流命令到 1 kHz 被控对象的确定性 Rate Transition/ZOH；
- 1 kHz `NLCSNN_Forward_1kHz`；
- 1 kHz 半主动耗散投影；
- 相同的 CarSim 输出打包顺序。

模型文件本身唯一决定控制器，不再根据 CarSim Run 数据集名称推测模式。CarSim 中对应的三个 Run/Simulink 数据集分别指向三个 SLX 文件，从而避免工作区残留变量或数据集命名导致跑错控制器。

### 独立入口、共享核心

每个模型的 MATLAB Function 控制块使用独立且可读的薄入口：

```matlab
[sys, i_cmd] = mpc_block(...);
[sys, i_cmd] = zeng_block(...);
[sys, i_cmd] = pmpc_block(...);
```

模式专属入口负责：

- 校验其参数包中的控制器变体；
- 初始化独立的跨拍 persistent 状态；
- 调用共享控制核心；
- 固定编译期的控制器模式和状态维数。

共享核心保留公共的数据流和算法步骤，模式差异通过编译期常量决定。不得复制三份完整控制算法，以免公共缺陷修复和参数接口发生漂移。

控制器 MATLAB Function 块中只保留上述入口调用和端口说明。完整算法继续存放在外部 M 文件中，以便 Git 文本审查、MATLAB 单元测试、代码搜索和三个模型共享。算法不直接复制到二进制 `.slx` 内部。

## 状态与时延模型

### MPC

MPC 的预测状态严格为：

```text
x_MPC = [Vy, r, phi, phidot, ey, epsi]
Nx = Ny = 6
```

其车辆预测模型中 CDC 控制量 `Md_c` 当拍作用于侧倾动力学，不调用 `local_delay`，也不构造、投影或索引 `Md_a`。这只是控制器预测模型不考虑时延；真实被控对象仍经过 NLCSNN 正模型和逆模型。

### ZENG 与 PMPC

两者预测状态为：

```text
x_delay = [Vy, r, phi, phidot, ey, epsi, Md_a]
Nx = Ny = 7
```

第 7 状态满足：

```text
tau_d * d(Md_a)/dt = -Md_a + Md_c
```

当前阶段固定：

```text
tau_d = 0.015 s
```

`Md_a` 初值由 NLCSNN/CarSim 当前四角实际阻尼力合成的滚转力矩得到，并投影到当前可达区间。控制器中的聚合时延状态仅用于预测；外部 NLCSNN 仍是实际被控对象，不增加旧 `func_DamperActuator` 路径。

### 未来状态相关时延

新增共享接口 `func_DamperDelayTau`。当前实现返回固定 `0.015 s`，接口保留未来查表所需输入：四角有符号减振器速度、电流变化绝对值和升降流方向。

未来查表形式为：

```text
tau_j = LUT(v_d,j, abs(DeltaI_j), sign(DeltaI_j))
```

在仍只有单个聚合状态 `Md_a` 时，ZENG/PMPC 使用四角 `tau_j` 的最大值作为保守聚合时延。查表未部署前不得伪造状态相关规律，也不得用电流变化率限幅代替时延。

## ZENG rho 信号

`func_ZengRho` 计算出的自适应调度量 `rho_zeng`保持标量，且继续作为 ZENG 稳定性松弛权重调度输入。

`zeng_mil.slx` 顶层从控制器诊断向量中独立选出 `rho_zeng`，生成一根明确命名为 `rho_ZENG` 的标量信号并连接到顶层 Outport。用户可以直接在该信号线上添加 Scope。模型不预置 Scope，避免改变用户调试布局。

MPC/PMPC 的同位置诊断量不得被误标为 ZENG rho。三个控制器的公共 `sys` 和 CarSim 54 路接口保持不变。

## 参数组织

新增三个单一入口初始化命令：

```matlab
mil_init_MPC
mil_init_ZENG
mil_init_PMPC
```

每个入口清理会导致串台的模式覆盖变量，设置固定变体，再生成对应参数包。控制器权重仍在文本 M 文件中分区维护：公共车辆/约束参数保持一份，MPC、ZENG、PMPC 的模式专属权重和开关在各自入口或专属配置段中明确设置。

三个模型不得依赖数据集名称自动识别控制器。自动识别逻辑仅可作为旧 `pmpc_mil` 兼容入口保留，不能影响新的三个固定模型。

## MPC 车道约束核查

拆分完成后必须对用户报告的 J-turn 现象进行证据化核查，不凭轨迹外观判断约束是否生效。至少记录：

- `ey`质心法向误差峰值；
- `ey`上下界松弛量；
- QP `exitflag`和兜底是否触发；
- AFS、DB、CDC 实际权限/饱和状态；
- 预测车道约束边界与实际 `ey`。

核查应区分三种原因：约束没有进入 QP、约束进入但软权重不足、执行器/轮胎能力不足导致只能通过松弛保持可行。此次拆分不擅自修改 MPC 权重；若确认权重或约束实现有缺陷，再单独测试和修复。

## 测试与验收

### 代码级测试

- MPC 参数包产生 `Nx=Ny=6`；ZENG/PMPC 产生 `Nx=Ny=7`；
- MPC 动力学矩阵为 6 状态且不调用 `local_delay`；
- ZENG/PMPC 动力学矩阵包含第 7 状态，固定 `tau_d=0.015 s`；
- 三个变体均启用同一 NLCSNN 配置、相同四角顺序和 `0~1.6 A`电流范围；
- ZENG 输出的`rho_ZENG`有限且随`func_ZengRho`变化；
- MPC/PMPC 不把其他优先级因子误当作`rho_ZENG`；
- QP维数与各自`Nx/Ny`一致，公共执行器输入维数、松弛变量和证书布局保持有效。

### 模型级测试

- 三个 SLX 均能完成 `load_system`、`set_param(...,'SimulationCommand','update')`；
- 三个模型保持控制器100 Hz、NLCSNN plant 1 kHz；
- 三个模型保持54路CarSim接口和四路电流输出；
- `zeng_mil`顶层存在可连接Scope的`rho_ZENG`标量线；
- 不出现端口尺寸、代数环或不可确定输出尺寸错误。

### 联合仿真

- 三个模型分别完成短时 CarSim smoke run；
- ZENG/PMPC 的第7状态和MPC的6状态均没有NaN/Inf；
- NLCSNN四角力满足半主动耗散约束；
- 对同一J-turn工况记录三控制器`ey`峰值和MPC约束诊断。

## 非目标

- 本阶段不部署状态相关时延查表；
- 不重新训练NLCSNN；
- 不把完整算法复制进Simulink MATLAB Function块；
- 不改变CarSim 54路输入输出顺序；
- 不因MPC滑出道路而未经诊断直接提高权重；
- 不删除当前工作树中与NLCSNN多速率部署相关的未提交改动。
