# 运行配置

## 在 CarSim 手动改 R69 初速度并运行

1. 首次使用时重启 CarSim，让新建的 Run 出现在列表中。只选 **PMPC** 类别中的 `JT72_R69_mu0.85_PMPC`、`_MPC` 或 `_ZENG`，不要选旧的 `JT80` Run。进入其 **Procedure**：`J-turn 72km/h, Mu=0.85`。修改初速度 `SV_VXS`（界面单位 **km/h**；20 m/s 填 **72 km/h**）并保存。这三个 Run 共用该 Procedure，只需改一次速度。
2. 逐个选择三个 Run，分别点击 **Send to Simulink**。Run 的 `SIMULINK_MODEL_FILE` 必须与控制器固定配对：

| CarSim Run 后缀 | Simulink 模型 | 控制器预测状态 |
|---|---|---|
| `_MPC` | `mpc_mil.slx` | 6 状态，无控制器侧减振器时延 |
| `_ZENG` | `zeng_mil.slx` | 7 状态，固定 15 ms |
| `_PMPC` | `pmpc_mil.slx` | 7 状态，固定 15 ms |

   Send 后，在项目根目录的 MATLAB 命令行执行对应的一条命令：

   ```matlab
   run_current_carsim('mpc_mil')
   run_current_carsim('zeng_mil')
   run_current_carsim('pmpc_mil')
   ```

   该命令会自动加载对应的参数包和模型；不能拿 `_ZENG` Run 去运行 `mpc_mil`，路径校验会直接报错，避免控制器串台。

   结果写入该 Run 在 CarSim 中的 `LastRun`，可直接用 CarSim Plot 查看；画图时也要选新的 `JT72` Run，不要继续叠加旧 `JT80` 结果。执行前脚本会核对当前有效初速度与 Send 后的展开值；若未重新 Send，会直接报错而不会用旧车速运行。

三个 Run 的纵向设置相同，ZENG 独有纵向限速设为 0。运行中实际车速可能因各控制器的制动力不同而分开。`JT72` 是当前数据集名称；以后改车速后可在 CarSim 中按新速度重命名，避免图例误导。

### 直接在 Simulink 调参/运行

不需要依赖数据集名称推断控制器。按下面的配对初始化后，修改该模型对应的参数，再点击 Simulink 运行即可：

```matlab
mil_init_MPC;  open_system('mpc_mil')
mil_init_ZENG; open_system('zeng_mil')
mil_init_PMPC; open_system('pmpc_mil')
```

ZENG 的顶层有标量线 `rho_ZENG`，它来自控制器诊断向量第 14 项。用户可在这根线上自行添加 Scope；MPC 和 PMPC 的同一诊断位置不是 ZENG rho，不能用于观察 ZENG 调权。

MATLAB Function 块只保留各自可见的入口（`mpc_block`、`zeng_block`、`pmpc_block`）。QP、NLCSNN、状态估计、车辆模型等公共算法仍放在外部 M 文件，便于三个模型共用、Git 对比、单元测试和检索。

---

## NLCSNN 减振器配置

MPC、ZENG 和 PMPC 全部使用同一个 NLCSNN 包络和被控对象，不再使用
`func_MRDamper` 或 `func_DamperActuator` 作为三个控制器的实际减振器。默认参数：

| 参数 | 默认值 |
|---|---:|
| `PMPC_NLCSNN` | `1`（启用） |
| 电流范围 | `0~1.6 A` |
| 初始电流 | `0 A` |
| 温度 | `42.5 degC` |
| 参考长度 `x_ref` | `281.0645 mm` |
| 控制周期 | `0.01 s` |
| 加速度低通时常数 | `0.02 s` |

旧的 `PMPC_DMP_ACT` 一阶延迟在 NLCSNN 启用时不进入主路径；NLCSNN 自己的
8 维隐状态和电流反演描述真实执行器动态。若要显式回退到旧模型：

```matlab
PMPC_NLCSNN = 0;
```

### CarSim 54 路 Export 顺序

原有 1–50 路顺序不变；只在末尾追加：

| 通道 | CarSim 变量 |
|---:|---|
| 51 | `CmpD_L1` |
| 52 | `CmpD_L2` |
| 53 | `CmpD_R1` |
| 54 | `CmpD_R2` |

`CmpD`/`CmpRD` 在 CarSim 中以压缩为正，适配器只在一个位置转换为 NLCSNN
的回弹为正约定。切换 Run Control、修改工况或车速后，必须重新点击
**Send to Simulink**；这会重新生成 `Run_all.par` 和 `simfile.sim`。发送后确认
`simfile.sim` 中是 `PORTS_EXP 1,54`，然后只需一条命令：

```matlab
run_current_carsim
```

---

三个 base workspace 变量控制运行模式。都在 `mdlInitializeSizes` 里读取，
**必须在仿真开始前设好**。仿真启动时命令窗口会打印一行配置横幅，以此确认。

| 变量 | 缺省 | 取值 |
|---|---|---|
| `PMPC_BASELINE` | 0 | 0 = PMPC 协调控制；1 = 无控制 baseline |
| `PMPC_FISHHOOK` | 0 | 0 = DLC 路径跟踪；1 = 鱼钩纯防侧翻 |
| `PMPC_CARGO` | 0 | 0 = 空载；170 = 顶置货箱 (kg) |
| `PMPC_BASELINE_I` | 2.0 | baseline 模式下 MR 的标称电流 (A) |

常用组合：

```matlab
% DLC + PMPC
clear PMPC_BASELINE PMPC_FISHHOOK PMPC_CARGO

% 鱼钩 + baseline
PMPC_BASELINE = 1; PMPC_FISHHOOK = 1; PMPC_CARGO = 170;

% 鱼钩 + PMPC
clear PMPC_BASELINE
PMPC_FISHHOOK = 1; PMPC_CARGO = 170;
```

---

## Baseline 模式做什么

跳过整个 MPC，四个角直接输出实测 MR 阻尼器在标称电流下的力
`func_MRDamper(v_i, 2.0)`，AFS 增量和制动力矩都输出 0。

Baseline 走**同一个 Simulink 模型**，所以被控对象与 PMPC 完全一致，唯一差别
是控制律。这是为了让对比干净——之前用纯 CarSim 跑 baseline 时，因为多了一个
CarSim 自带的速度控制器，反向段车速差了 3–4 km/h，足以把侧翻变成不侧翻。

为什么标称电流取 2.0 A 而不是 0 A：半主动悬架正常工作时不会断电。实测
0 A 的等效阻尼只有 929 N·s/m（ζ_前 ≈ 0.05，几乎无阻尼）；2.0 A 时
ζ_前 = 0.37 / ζ_后 = 0.75，与该车原装被动减振器（CarSim "Big SUV"：
0.34 / 0.69）等效。0 A 仍是控制器的真实下界 `Gl`，只是不适合当 baseline。

## 鱼钩模式做什么

鱼钩是开环转向的稳定性试验，没有目标轨迹，控制目标只有"别翻"：

| 权重 | DLC | 鱼钩 |
|---|---|---|
| Q1 Vy / Q2 偏航角速度 | 0 / 0 | 0 / 0 |
| Q3 侧倾角 | 1e2 | 2e4（主目标） |
| Q4 侧倾角速度 | 0 | 2e3 |
| Q5 横向误差 / Q6 航向误差 | 2e3 / 2e2 | 0 / 0 |
| LTR 包络约束 | 有 | 保持 |

`func_RefTraj_LocalPlanning` 整个跳过，路径曲率 kappa = 0，ey/epsi 置 0。

**Q3 / Q4 是拍的起始值，未经标定。** 它们经 `phimax²` / `dphimax²` 归一化。
代价里现在只剩侧倾项 + 控制代价 + 松弛惩罚，若出现大量未收敛或转角乱跳，
可以把 Q2 开一个小值（即便 `ref_r = 0`，也等于抑制偏航角速度）来正则化。

## 转向的两种接法

| 工况 | `IMP_STEER_SW` | Simulink 输出 | 原因 |
|---|---|---|---|
| DLC | Replace | 总转角 | Simulink 做路径跟踪，产生完整转角 |
| 鱼钩 | **Add** | **只有增量** | CarSim 转向机器人出 294° 鱼钩，AFS 只叠加增量 |

鱼钩下若仍输出总转角，机器人的 294° 会被叠加两次。

`delta_robot`（机器人给的前轮转角）由 `实测总转角 - 上一步 AFS 增量` 重构。
控制量 `u` 是它之上的增量，总转角 = `delta_robot + u`。

## 已知问题：`Fy0`

`Fy0` 是 u = 0 时的前轴侧向力，用于 AFS 的幅值边界
`dlt_ub = (Flim - Fy0)/Cm`。正确式：

```
Fy0 = fb_f + cb_f*(delta_f - delta_robot)
```

原式 `cb_f*(ab_f - delta_robot) + F_off_f` 化简后是 `fb_f - 2*cb_f*delta_robot`，
少了 `cb_f*delta_f` 一项（delta_f = 5° 时约 7000 N）。

**鱼钩分支已用正确式，DLC 分支仍用原式**，以免动到已标定的结果。DLC 那边的
bug 还在，修了要重新验证 `dlt_rate_max` 和 `R1`。

（`F_off_f` 里的 `- cb_f*delta_robot` 是**对的**，不要改：模型 A 矩阵用
`(Vy+lf*r)/V` 算侧偏角、不含 δ，偏置必须吸收这一项。）

## 车辆参数

数值全部取自 CarSim echo 文件的 `! CALC --` 只读计算量，不要手推
（我手推的 I_zz 两次都错了 3%）：

| | 空载 | 载 170 kg | echo 关键字 |
|---|---|---|---|
| m | 1860 | 2030 | `M_TL` |
| m_s | 1590 | 1760 | `M_SU` / `M_SL` |
| lf | 1.2466 | 1.3514 | `LX_CG_TL` |
| I_zz | 3438.54 | 3742.62 | `IZZ_TL` |
| I_x | 894.4 | 1403.80 | `IXX_SU` / `IXX_SL` |
| h_S2R | 0.570 | 0.742 | `H_CG_SL` − 0.15 |

## 轮胎表 C0

`TireF/TireR = 90.7e3 / 109e3 @ mu 0.9`，取自 Wang et al., IEEE TVT 72(10)
2023, Table I（同一辆 CarSim E-Class SUV）。

这是**等效轴刚度**，含悬架/转向柔性，不是轮胎本身的刚度。注意它与轮胎对不上：
原版车前轴荷 57.7%，前轴应更硬（carpet 实测轴级 131600/97800），而 Wang 给的
是前软后硬。方向与我们自己标定 235/55R18 时得到的 80e3/150e3 一致，说明 Wang
报的同样是含柔性的等效值。后续若要更准，应改回按 `FY_TIRE_CARPET` 建表并单独
标定前后柔性因子。

## ⚠️ 不要删的文件

`simfile.sim` —— CarSim "Send to Simulink" 写出的句柄文件，指向本次运行的
`Results\Run_<GUID>\`。`func_CarSimResDir` / `func_RunMode` / `wsget` 都要读它，
删掉后所有仿真脚本第一步就报 `错误使用 fileread`。它看着像临时文件，但不是。

恢复办法见记忆 `carsim-dataset-editing`：CarSim 在
`CarSim2019.0_Data\simfile.sim` 留了精简版主副本可查 GUID，
拿同项目旧副本换 GUID 即可，其余字段稳定。
