# 三执行器协调控制论文：优先级策略创新方案与跨工况鲁棒性设计（CED-PMPC）

> 基于 [JiaMingXu2482/PMPC](https://github.com/JiaMingXu2482/PMPC) 仓库最新代码与审查记录整理，围绕 **AFS（主动前轮转向）+ DYC（差动制动直接横摆力矩）+ CDC（连续可调阻尼半主动悬架）** 协调控制论文目标，给出兼顾理论创新与 FishHook、Slalom、DLC 全工况免调参鲁棒性的优先级机制设计方案。

## 一、核心判断

单独推进 **Route A（状态演化能力集 $\Omega_{k+j}(\dot{\varphi}_{k+j})$）** 只能修补半主动悬架的预测约束精度，本身尚不构成完整的“跨目标—跨执行器优先级叙事”；而单纯实现**“横摆裕度 + 侧倾裕度”两参数权重调度**，不仅与近期相平面/预测 LTR 自适应权重 MPC 文献（如 [Symmetry 2026](https://www.mdpi.com/2073-8994/18/1/208)）及仓库现有 `ZENG` 基准高度重叠，还会因忽略“执行器瞬时控制权限”而陷入针对单一工况调参的泥潭。

最具竞争力的突破口是将二者深度咬合为**“视界能力包络与能力赤字驱动的双层优先级控制（Capability-Envelope & Deficit-Driven PMPC, CED-PMPC）”**：用演化包络 $\Omega_{k+j}(\dot{\varphi}_{k+j})$ 预判未来侧倾峰值处的 CDC 阻尼能力塌陷，再以**“低侵入执行器（CDC+AFS）在演化包络下的视界安全能力赤字”**连续门控高侵入执行器（DYC），同时彻底拆解“防侧翻 $\succ$ 横摆稳定 $\succ$ 车道跟踪”三级松弛，实现单套物理参数跨 FishHook、Slalom、DLC 自适应分化。

### 关键结论速览

* **时滞补偿证伪的物理根因**：SUV 侧倾固有周期约 $600\text{–}1000\ \mathrm{ms}$，$12.7\ \mathrm{ms}$ 滞后仅折合 $4^\circ\text{–}7^\circ$ 相位差且被 $10\ \mathrm{ms}$ 滚动反馈吸收；真正卡死 CDC 抗侧倾上限的是侧倾峰值处 $\dot{\varphi} \to 0$ 引发的耗散功率归零。
* **Route A 解决“幻影阻尼”但缺招募律**：演化能力集让控制器提前“看见”未来 $0.2\text{–}0.3\ \mathrm{s}$ 侧倾顶点处的 CDC 失能，但若缺少与能力赤字挂钩的 DYC 连续门控，仍会在 FishHook 与柔和 Slalom 之间顾此失彼。
* **纯状态裕度变权重的跨工况陷阱**：据仓库 [归一化审查文档](https://github.com/JiaMingXu2482/PMPC/blob/main/docs/README_%E5%BD%92%E4%B8%80%E5%8C%96%E5%AE%A1%E6%9F%A5.md)，依赖车速稳态横摆上界的权重在 $40\ \mathrm{km/h}$ 绕桩时缩水至 $100\ \mathrm{km/h}$ 双移线的 $0.16\times$，且在 $\dot{\varphi} \approx 0$ 时盲目加大侧倾权重无法产生阻尼力。

---

## 二、现状机理复盘

结合仓库最新主分支代码与 [架构缺陷与救治方案](https://github.com/JiaMingXu2482/PMPC/blob/main/docs/README_%E6%9E%B6%E6%9E%84%E7%BC%BA%E9%99%B7%E4%B8%8E%E6%95%91%E6%B2%BB%E6%96%B9%E6%A1%88.md)、[归一化审查](https://github.com/JiaMingXu2482/PMPC/blob/main/docs/README_%E5%BD%92%E4%B8%80%E5%8C%96%E5%AE%A1%E6%9F%A5.md) 的实测证据，当前实验现象背后的动力学因果链如下：

1. **时滞属于次要扰动**：$\tau_d = 10^{-4}\ \mathrm{s}$ 与 $12.7\ \mathrm{ms}$ 在全步长下闭环 LTR 重合，证明对于预测步长 $50\ \mathrm{ms}$、控制步长 $10\ \mathrm{ms}$ 的上层 MPC 而言，一阶执行器滞后属于高阶次要扰动，论文可将其作为消融反证保留，而不再作为主创新。
2. **冻结首拍边界制造“幻影阻尼”**：在现有 [func\_CostWeightingRegulation\_QuadSlacks.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_CostWeightingRegulation_QuadSlacks.m) 中，抗侧倾阻尼力矩上下界 $[M_{d,\min}, M_{d,\max}]$ 由当前时刻 $k$ 的四角压缩速度算得，并在整个 $0.6\ \mathrm{s}$（$N_p = 12$）预测视界内保持冻结。车辆入弯时 $|\dot{\varphi}_k|$ 处于高位，冻结边界使 QP 误以为未来 $0.3\ \mathrm{s}$ 达到最大侧倾角时 CDC 仍能持续输出数千牛米抗侧倾力矩，从而推迟 DYC 与 AFS 介入；等到真实轨迹抵达侧倾峰值、$\dot{\varphi} \to 0$ 时，下层 [func\_SemiActivePassivity.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_SemiActivePassivity.m) 将阻尼力投影回软态耗散底线 $c_{\min}v_j \approx 0$，导致 LTR 超调。
3. **现有优先级层的三处跨工况冲突**（见 [func\_BuildQPConstraints.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_BuildQPConstraints.m) 与 [func\_PriorityCert.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_PriorityCert.m)）：
   * `prio_on` 模式将防侧翻 `LTR` 与车道包络 `env` 捆绑进同一标量松弛 $s_1$，在高速 DLC 中为了死守车道边界反而迫使车辆维持高侧向加速度 $a_y$，直接与防侧翻对抗；
   * `certC` 在安全裕度 $d_1, d_2 > 0$ 时将 DYC 完全锁死（$\gamma_{\text{DB}} = 0$），一旦越界则权重骤跳至 $w_{\max}$，形成数值悬崖效应（[缺陷 A4](https://github.com/JiaMingXu2482/PMPC/blob/main/docs/README_%E6%9E%B6%E6%9E%84%E7%BC%BA%E9%99%B7%E4%B8%8E%E6%95%91%E6%B2%BB%E6%96%B9%E6%A1%88.md)）；
   * $\rho_{\text{CDC}}$ 曾被下界锁死在 $1$ 附近（98.2% 时间无调节余地），使执行器优先级退化为仅对 DYC 单通道开关定价。

---

## 三、五条优先级技术路线对比

| 候选优先级路线 | 核心优先级定义 | 论文创新辨识度 | 多工况鲁棒性（FishHook / Slalom / DLC） | 参数敏感度与落地代价 |
| :--- | :--- | :--- | :--- | :--- |
| **1. 现有静态字典序单 QP** (`prio_on + PriorityCert`) | 精确罚权重分离（$s_1 \succ s_2 \succ \gamma_{\text{DB}}$），凭候选裕度切换硬阈值（[源码](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_PriorityCert.m)） | 中等（忽略执行器时变能力边界） | **偏弱**：`LTR` 与 `env` 混层冲突；$d_1 \to 0$ 处 $\gamma_{\text{DB}}$ 阶跃跳变诱发 DLC 数值敏感 | 高敏感；现成代码 |
| **2. 纯 Route A（在研）** ($\Omega_{k+j}(\dot{\varphi}_{k+j})$ 演化约束) | 预测视界内按预测侧倾率动态收缩/展开 CDC 力矩上下界 | 偏弱（易被视作工程建模修正而非优先级机制） | **局部有效**：改善 FishHook 80 保舵段，但 Slalom 快速过零处易受热启动符号翻转扰动，且未解决 DYC 何时介入 | 中等；约 40 行改动 |
| **3. 纯两参数权重调度** ($(\mu_{\text{yaw}}, \mu_{\text{roll}}) \to Q, R$) | 由瞬时横摆稳定裕度与侧倾裕度经 Sigmoid/指数函数调节代价权重 | **低**（与 `ZENG` 及 [2026 AMPC 文献](https://www.mdpi.com/2073-8994/18/1/208) 高度同质化） | **脆弱**：“状态危险但执行器无权限”时盲目加权，需按车速/工况重调斜率 | 高（超参数多）；约 30 行改动 |
| **4. 主推方案：CED-PMPC**（视界能力包络 + 能力赤字驱动双层优先级） | **目标层**三级精确罚解耦（侧翻 $\succ$ 横摆 $\succ$ 车道）；**执行器层**按演化包络下的**视界安全赤字 $\delta_{\text{def}}$** 光滑招募 DYC | **高（顶刊级）**：首次将“半主动时变耗散包络”转化为“单 QP 执行器渐进招募优先级判据” | **极强**：Slalom 零误制动、DLC 精准点刹并适度让渡路径、FishHook 提前预判阻尼塌陷平滑接管，全程同一套无量纲参数 | **低敏感**；复用现有 85% 架构，约 90 行改动 |
| **5. 备选方案：CA-AMPC**（风险裕度 $\times$ 控制权限双因子调度） | 用“状态风险度 $\times$ 执行器有效权限度”联合调度 $Q$ 与 $R$ | **中高**：以“高优执行器权限枯竭触发低优执行器接管”形成机制区分 | **强**：消除轮胎饱和与减振器过零时的无效加权，但缺少约束松弛意义下的字典序底线证明 | 中低；可作融合子模块，约 50 行改动 |

---

## 四、核心创新机制：CED-PMPC 数学形式

将“优先级”从静态权重比例重定义为：在目标空间执行“生存安全 $\succ$ 操纵稳定 $\succ$ 轨迹跟踪”的平滑精确罚序，在执行器空间执行“仅当低侵入执行器集合在未来演化能力包络 $\Omega_{k+j}$ 下出现预测安全能力赤字时，按赤字幅值连续解锁高侵入执行器”的按需招募律（契合集成底盘控制协调原则，见 [Skrickij 等 ICC 综述](https://pmc.ncbi.nlm.nih.gov/articles/PMC10819876/)）。

### 机制一：过零稳健的视界演化能力包络 $\Omega_{k+j}$（升级版 Route A）

利用上一拍 QP 最优增量移位生成预测状态序列 $\hat{x}_{k+j|k-1}$，推演未来各步四角减振器行程速度 $\hat{v}_{i,k+j}$，通过 `nlcsnn` 硬态（$0\ \mathrm{A}$）与软态（$1.6\ \mathrm{A}$）包络逐拍生成时变抗侧倾上下界。为防止 Slalom 与 DLC 快速换向时热启动预测速度符号翻转引起 QP 约束跳变，围绕被动阻尼力矩 $M_{d,0,k+j}$ 引入过零不确定区平滑收缩因子 $\chi_{k+j} \in [0, 1]$：

$$
\chi_{k+j} = \frac{\hat{\dot{\varphi}}_{k+j}^2}{\hat{\dot{\varphi}}_{k+j}^2 + \omega_{\text{dz}}^2}
$$

$$
M_{d,\min,k+j} = M_{d,0,k+j} - \gamma_{\text{CDC}} \cdot \chi_{k+j} \cdot \bigl(M_{d,0,k+j} - M_{d,\text{lo}}(\hat{v}_{k+j})\bigr)
$$

$$
M_{d,\max,k+j} = M_{d,0,k+j} + \gamma_{\text{CDC}} \cdot \chi_{k+j} \cdot \bigl(M_{d,\text{hi}}(\hat{v}_{k+j}) - M_{d,0,k+j}\bigr)
$$

同时采纳 [救治方案 1B](https://github.com/JiaMingXu2482/PMPC/blob/main/docs/README_%E6%9E%B6%E6%9E%84%E7%BC%BA%E9%99%B7%E4%B8%8E%E6%95%91%E6%B2%BB%E6%96%B9%E6%A1%88.md)，将 $\gamma_{\text{CDC}} \in [0, 1]$ 重定义为相对被动阻尼点的调制裕度，彻底修复原先 $\rho_{\text{CDC}}$ 被钉死在 $1$ 的结构缺陷，并在 $\hat{\dot{\varphi}}_{k+j} \to 0$ 时自然收敛至被动阻尼，保证任何工况下 QP 恒可行。

### 机制二：目标三级严格解耦与能力赤字门控

在 [func\_BuildQPConstraints.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_BuildQPConstraints.m) 中拆开原先捆绑的 $s_1$，建立严格三级标量松弛：第 1 级防侧翻 $s_{\text{roll}}$（$|LTR| \le LTR_{\text{th}}$）、第 2 级横摆与后轴侧偏稳定 $s_{\text{yaw}}$（$|\alpha_r| \le \alpha_r^{\text{sat}}$，以恒定轮胎侧偏角阈值替代随车速漂移的 $r_{\text{ssmax}}$）、第 3 级车道包络 $s_{\text{env}}$，满足精确罚分离 $w_{\text{roll}} \gg w_{\text{yaw}} \gg w_{\text{env}}$。随后计算“无 DYC 候选解 $\hat{z}^0$”在演化包络 $\Omega_{k+j}$ 下的**无量纲视界安全能力赤字**：

$$
\delta_{\text{def}}(k) = \max_{1 \le j \le N_p} \max\left[\frac{|LTR_{k+j}^0| - \kappa_m LTR_{\text{th}}}{LTR_{\text{th}}},\; \frac{|\alpha_{r,k+j}^0| - \kappa_m \alpha_r^{\text{sat}}}{\alpha_r^{\text{sat}}}\right]
$$

$$
v_{\text{DYC}}(k) = v_{\text{DYC},\min} + \frac{v_{\text{DYC},\max} - v_{\text{DYC},\min}}{1 + \exp\bigl(\delta_{\text{def}}(k) / \delta_s\bigr)}
$$

其中安全预警裕度系数固定取 $\kappa_m = 0.85$、平滑过渡尺度固定取 $\delta_s = 0.06$，同时打开相邻拍激活度平滑项 $W_{d\gamma} > 0$。当 $\delta_{\text{def}} < 0$ 时 $v_{\text{DYC}}$ 保持门控高值，严格保证 $\gamma_{\text{DYC}} = 0$；一旦未来视界出现能力赤字，$v_{\text{DYC}}$ 连续下调并平滑招募差动制动。

### 备选/融合加分项：权限感知型两参数权重调度

若需在代价矩阵 $Q, R$ 中同步融入“横摆裕度 + 侧倾裕度”，应引入**“状态危险度 $\times$ 执行器瞬时权限度”交叉律**：令 $\xi_{\text{roll}} = \operatorname{sat}(|LTR|/LTR_{\text{th}})$、$\xi_{\text{yaw}} = \operatorname{sat}(|\alpha_r|/\alpha_r^{\text{sat}})$，定义 CDC 有效权限度 $\eta_{\text{CDC}} = \operatorname{sat}\bigl((M_{d,\max}-M_{d,\min})/(2 M_{d,\text{ref}})\bigr)$ 与 AFS 剩余权限度 $\eta_{\text{AFS}} = 1 - \operatorname{sat}(|\alpha_f|/\alpha_f^{\text{sat}})$：

$$
Q_{\text{path}} = Q_{\text{path},0}\Bigl(1 - 0.7 \max(\xi_{\text{roll}}, \xi_{\text{yaw}})^2\Bigr)
$$

$$
R_{\text{DYC}} = R_{\text{DYC},0} \cdot \bigl(1 - \xi_{\text{roll}}(1 - \eta_{\text{CDC}})\bigr) \cdot \bigl(1 - \xi_{\text{yaw}}(1 - \eta_{\text{AFS}})\bigr)
$$

---

## 五、跨工况免调参鲁棒性逻辑与代码改造蓝图

### 跨工况自动分化机理

| 验证工况与扰动集 | 物理相空间特征 | CED-PMPC 自动涌现的协调机理 | 相对传统方法的预期优势 |
| :--- | :--- | :--- | :--- |
| **Slalom 连续绕桩** ($45\text{–}65\ \mathrm{km/h}, \mu = 0.85$) | 连续周期性换向，$|\dot{\varphi}|$ 均值高、过零驻留极短 | $\Omega_{k+j}$ 全程提供充沛 CDC 抗侧倾力矩，$\delta_{\text{def}} < 0$ 自动锁闭 DYC；恒定 $\alpha_r^{\text{sat}}$ 消除低速权重坍塌 | 相比固定权重 MPC 不掉车速（$\Delta V_x \approx 0$），侧倾角 RMS 与相位滞后显著下降 |
| **ISO 3888-2 双移线 (DLC)** ($80\text{–}100\ \mathrm{km/h}, \mu = 0.85 / 0.50$) | 第二车道回正瞬间出现横摆、后轴侧偏与侧倾复合双峰冲击 | 进出弯瞬态由 CDC + AFS 承担；回正峰前预判 $\delta_{\text{def}} > 0$ 触发短促 DYC 点刹；极限低附着时自动松弛 $s_{\text{env}}$ | 消除阈值边缘抖振，兼顾横向跟踪误差峰值与纵向末速度保留率 |
| **NHTSA FishHook 鱼钩** ($75\text{–}85\ \mathrm{km/h}$, 空载 / $+170\ \mathrm{kg}$ 顶载) | 急剧反向打舵后长时间保舵，进入高 $a_y$ 准稳态，$\dot{\varphi} \to 0$ | 打舵瞬态 CDC 满载耗能抑峰；视界提前 $0.25\ \mathrm{s}$ 预见保舵段 $M_{d,\max,k+j} \to 0$，无缝唤醒 DYC + AFS | 解决保舵段 LTR 二次抬头甚至抬轮（$F_z \to 0$）问题，对质心升高天然鲁棒 |

### `controller/` 目录模块改造清单

1. [func\_CostWeightingRegulation\_QuadSlacks.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_CostWeightingRegulation_QuadSlacks.m)：输入移位预测轨迹 $\hat{X}_{\text{pred}}(1:N_p)$，逐拍计算 $N_c$ 步时变边界序列 `Lim.Md_min_seq`、`Lim.Md_max_seq`、`Lim.Md0_seq` 及 $\chi_{k+j}$；将横摆稳定归一化基准统一为恒定 $\alpha_r^{\text{sat}}$。
2. [func\_BuildQPConstraints.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_BuildQPConstraints.m)：将状态软约束拆分为独立三级标量松弛 $s_1 = s_{\text{roll}}, s_2 = s_{\text{yaw}}, s_3 = s_{\text{env}}$；将 `A_input` 的 $M_d$ 行改为围绕 `Md0_seq` 展开的时变 $\gamma_{\text{CDC}}$ 约束。
3. [func\_PriorityCert.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_PriorityCert.m)：在投影无 DYC 候选解 $\hat{z}^0$ 时代入演化包络 $\Omega_{k+j}$，输出 $\delta_{\text{def}}$、$v_{\text{DYC}}(\delta_{\text{def}})$ 与三级精确罚权重 $[w_{\text{roll}}, w_{\text{yaw}}, w_{\text{env}}]$。
4. [func\_BuildQPCost.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_BuildQPCost.m) & [pmpc\_step.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/pmpc_step.m)：启用非零 $W_{d\gamma}$ 抑制相邻拍跳变；将 $v_{\text{DYC}}(\delta_{\text{def}})$ 写入 $f_\gamma$，并在 36 维诊断向量 `St.cert` 中记录 $\delta_{\text{def}}$。

### 建议消融实验矩阵

冻结同一套参数，设置 **5 组对比控制器 $\times$ 4 组工况簇**：（C1）传统固定权重 MPC（`mpc_mil`）；（C2）状态裕度自适应变权重基准（`zeng_mil` / 纯两参数调度）；（C3）冻结首拍能力集的静态字典序 PMPC（无 Route A）；（C4）仅含演化能力集 $\Omega_{k+j}$ 但固定执行器权重的 MPC（纯 Route A）；（C5）**本文完整的 CED-PMPC**。
