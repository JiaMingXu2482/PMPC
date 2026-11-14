# baseline_mpc — 独立基线 MPC（6 状态 / 2 控制量）

## 这是什么

论文对比用的**下界基线控制器**，与 `pmpc_core/` 的协调控制器（7 状态 / 3 控制量）
完全独立实现，不复用其代码。

与协调控制器相比，**只少了 Md**：没有 Md 状态、没有 Md 控制量、因此没有时滞建模，
减振器退化为固定 1 A。侧倾自由度（$\varphi$、$p$）、LTR 约束、Zeng 稳定域、道路包络
一律保留，被控对象模型与协调控制器同一套。

| | **本目录（基线）** | `pmpc_core/`（协调控制器） |
|---|---|---|
| 状态 | $[v_y,\ r,\ e_y,\ \varepsilon_\psi]$（4） | $[v_y,\ r,\ \varphi,\ p,\ e_y,\ \varepsilon_\psi,\ M_d]$（7） |
| 控制量 | $[\delta,\ M_z]$（2） | $[\delta,\ M_z,\ M_d]$（3） |
| 减振器 | **固定 1 A**，不是决策量 | 控制器给电流 |
| LTR 约束 | **有**（$|\mathrm{LTR}|\le 0.8$） | 有 |
| 时滞建模 | **无** | 有（`tau_d`） |
| 松弛变量 | 4（横摆 / 侧偏 / LTR / 道路） | 4 |
| Simulink 模型 | `pmpc_base.slx` | `pmpc_mil.slx` |

## 文件

| 文件 | 作用 |
|---|---|
| `pmpc_base_init.m` | 参数、参考轨迹、初始状态；`pmpc_base_bus` 的输入 |
| `pmpc_base_bus.m` | 测 bus 定义（6 状态 / 2 控制量） |
| `pmpc_base_prepare.m` | 局部参考投影、轮胎线性化、执行器边界、约束行 |
| `pmpc_base_predict.m` | 6 状态 ZOH 预测矩阵 |
| `pmpc_base_step.m` | 主控制拍：预测 → QP → 分配 |
| `build_pmpc_base.m` | 脚本：程序化搭建 `pmpc_base.slx` |
| `run_baseline.m` | 跑一个算例（DLC / Slalom / FishHook） |

## 数学

状态方程（连续，$v_x$ 冻结）：

$$\dot x=A_0(v_x)x+B_0(v_x)u+D_0(v_x)w,\qquad w=[\kappa,\ \Delta F_{yf},\ \Delta F_{yr}]^\top$$

$$A_0=\begin{bmatrix}
\frac{C_f c_\delta+C_r}{mv_x} & \frac{C_f l_f c_\delta-C_r l_r}{mv_x}-v_x & 0 & 0\\
\frac{l_fC_f c_\delta-l_rC_r}{I_zv_x} & \frac{l_f^2C_f c_\delta+l_r^2C_r}{I_zv_x} & 0 & 0\\
1 & 0 & 0 & v_x\\
0 & 1 & 0 & 0
\end{bmatrix},\quad
B_0=\begin{bmatrix}
-\frac{C_f c_\delta}{m} & 0\\
-\frac{l_fC_f c_\delta}{I_z} & \frac{1}{I_z}\\
0 & 0\\ 0 & 0
\end{bmatrix}$$

$$D_0=\begin{bmatrix}
0 & \frac{c_\delta}{m} & \frac{1}{m}\\
0 & \frac{l_f c_\delta}{I_z} & -\frac{l_r}{I_z}\\
0&0&0\\ -v_x&0&0
\end{bmatrix}$$

零阶保持离散化后增广上一拍输入：

$$\begin{bmatrix}x_{k+1}\\u_k\end{bmatrix}
=\begin{bmatrix}A_d&B_d\\0&I\end{bmatrix}\begin{bmatrix}x_k\\u_{k-1}\end{bmatrix}
+\begin{bmatrix}B_d\\I\end{bmatrix}\Delta u_k+D_d w_k$$

目标函数：

$$J=\sum_{j=1}^{N_p}\|x_j-x_{ref,j}\|_Q^2+\sum_{j=1}^{N_c}\|\Delta u_j\|_R^2+\sum_{i\in\{r,\alpha,env\}}w_i\sigma_i$$

约束：控制量盒约束（含 $M_z$ 的附着极限）+ Zeng 鞍结稳定域（横摆/后轴侧偏）
+ **LTR 约束**（$|\mathrm{LTR}|\le 0.8$，含转向输入与轮胎偏置项）+ 道路包络。
LTR 约束与协调控制器同一套实现（`pmpc_base_prepare.m` 的 `Hr`/`Gr`/`Or`）。

## 减振器

`pmpc_base_step` 输出恒定电流 `M.cdc_fixed_I`（默认 1.0 A），四角相同。
CarSim 侧的减振器由 `nlcsnn/inv_current_impl_10ms` 之外的路径施加：
`pmpc_base.slx` 直接把 `Icmd` 送到 CarSim 的 `IMP_MY_SUS`（或对应 import），
不经过力界/逆解。

## 用法

```matlab
% 1. 生成模型（只需一次）
build_pmpc_base

% 2. 跑算例
run_baseline('dlc', 80)
run_baseline('slalom', 80)
run_baseline('fishhook', 80, 'cargo', 190)

% 3. 参数单独调
[P,R,s0,m0] = pmpc_base_init(2, struct('Ts',0.05,'Np',12,'Nc',6));
```

## 注意

- 本目录**不修改** `pmpc_core/` 任何文件，两套控制器可并存。
- `pmpc_base.slx` 里没有边界模型和逆模型，只有 Estimator + 主控制块 + Unit Delay。
- CarSim 数据集需与协调控制器用**同一辆车、同一路面、同一转向表**，
  否则三个控制器不可比。
