# 运行配置

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
