# Zhao-2026 与 Yang-2024 公式级拆解：机理漏洞、复现指南与靶向对比实验设计

> 逐页提取仓库中 [Zhao-2026（JBSMSE）](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6/Zhao-2026-Adaptive-MPC-coordinated-control-trajectory-tracking-yaw-roll-stability-distributed-drive-EV-JBSMSE.pdf) 与 [Yang-2024（IEEE TVT）](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6/Yang-2024-Anti-roll-trajectory-tracking-adaptive-weight-phase-space-stable-region-TVT.pdf) 的状态方程、稳定域边界、自适应权重函数与下层分配公式，定位可在 [JiaMingXu2482/PMPC](https://github.com/JiaMingXu2482/PMPC) 上直接复现并靶向击败的机理盲区。

## 一、核心公式与参数对照

| 拆解维度 | Zhao-2026（JBSMSE, AFS + DYC + 主动悬架 ASS） | Yang-2024（IEEE TVT, 单执行器 AFS） |
| :--- | :--- | :--- |
| **预测状态与输入输出** | 8 维状态 $x=[V_x,V_y,\phi,\dot{\phi},X,Y,\emptyset,\dot{\emptyset}]^T$；3 维输入 $u=[\delta, M_z, M_x]^T$；5 维输出 $y=[Y,\phi,\dot{\phi},\beta,LTR]^T$ | 6 维状态 $x=[\beta,\phi,\dot{\phi},\gamma,e_y,e_\varphi]^T$；1 维输入 $u=\delta_f$；5 维输出 $y=[\beta,LTR,\gamma,e_y,e_\varphi]^T$ |
| **采样周期与时域** | $T = 0.02\ \mathrm{s}$，$N_p = 10$（预瞄 $0.20\ \mathrm{s}$；底层积分步长 $1\ \mathrm{ms}$） | $T = 0.01\ \mathrm{s}$，$N_p = N_c = 25$（预瞄 $0.25\ \mathrm{s}$，无控制时域压缩） |
| **横摆稳定域判据** | $\beta$-$\dot{\beta}$ 双线性边界 + 拟合椭圆：$a=0.0694\mu^3-0.0284\mu^2+0.0946\mu+0.0002$，$b=0.7426\mu-0.0002$；内边界取 $0.8\times$ 外边界，按模长 $S_i=\sqrt{\beta_i^2+\dot{\beta}_i^2}$ 算 $\kappa$（Eq. 57） | 三维箱体边界（Eq. 34）：$\|\gamma\| \le \mu g/v_x$；$\beta_{r,\min/\max} = l_r\gamma/v_x \mp \tan(\alpha_{r,\max})$ |
| **侧倾 LTR 模型与阈值** | $LTR = 2(K_\phi\emptyset + C_\phi\dot{\emptyset})/(mgd)$；阈值 $LTR_m = 0.65$，极限 $1.0$ | $LTR = -2K_\phi\phi/(MgW) - 2C_\phi\dot{\phi}/(MgW)$；硬边界 $LTR_{\max} = 0.60$ |
| **自适应权重核心律** | $\gamma(\kappa) = \frac{1}{1+e^{-15(\kappa+0.5)}}$（$-1\le\kappa<0$）；$\eta(LTR)=c(e^{\|LTR\|-0.65}-1)$（$c \approx 2.3862$）；$\zeta(LTR)\in\{0,1\}$；$Q_{Y,\phi}=(1-\eta)\gamma Q_0$，$Q_{\dot{\phi},\beta}=(1-\eta)(1-\gamma)Q_0$，$Q_{LTR}=\eta Q_{LTR0}$；$R_\delta=\gamma R_{\delta 0}$，$R_{\Delta M_z}=(1-\gamma)R_{M_{z0}}$，$R_{\Delta M_x}=\zeta R_{M_{x0}}$ | $Q_{e_y}, Q_{e_\varphi}$ 与 $R_0=0.5$ 恒定，仅调 $Q_i$（$Q_{i0}=0.1$）：$e_{k,i} = \|x_{k,i}-x_{k,i,\max}\|$（Eq. 35），$Q_i = Q_{i0} + \frac{e_{k,i}}{\sum e_{k,j}} Q_{i0}$（Eq. 36） |
| **下层分配方程** | 轮矩分配（`fmincon`）：$\min \sum C_i F_{xi}^2/(\mu_i F_{zi})^2$（**忽略 $F_{yi}$**）；悬架均分：$F_{u1}=F_{u3}=-F_{u2}=-F_{u4}=\Delta M_x/(2d)$（**无无源性约束**） | 无下层分配器 |

---

## 二、致命代数漏洞拆解

### Zhao-2026 的四处硬伤（Z1–Z4）

* **Z1. 侧倾逼近极限时横摆权重乘性归零（Eq. 87）**：由图 9 端点 $\eta(1.0)=1$ 可反推常数 $c = 1/(e^{0.35}-1) \approx 2.3862$。由于作者把 $[1-\eta(LTR)]$ 同时乘在了路径权重和横摆稳定权重 $Q_{\dot{\phi}}, Q_\beta$ 前面，当 $|LTR| \to 0.95$（$\eta \approx 0.835$）时，**横摆角速度与侧偏角权重瞬间坍缩逾 83%**，在侧翻边缘放任甩尾。
* **Z2. $|LTR| = 0.65$ 硬开关死区与矩阵奇异（Eq. 86 & 88）**：$|LTR| < 0.65$ 时 $\eta=0$ 且 $\zeta=0$，导致 $R_{\Delta M_x}=0$（若不旁路则会引发 Hessian 奇异）；在 $\mu \le 0.5$ 工况下悬架全程零出力，跨过 $0.65$ 时发生不连续阶跃。
* **Z3–Z4. 理想主动悬架假设在半主动 CDC 上的峰值踏空**：侧倾角峰值处 $\dot{\emptyset}=0$，半主动阻尼器物理可达力归零，虚拟力矩 $M_x$ 被无源性约束截断；且下层 `fmincon` 轮矩分配忽略了侧向力摩擦圆耦合。

### Yang-2024 的三处硬伤（Y1–Y3）

* **Y1. 距离方向悖论与量纲混加（Eq. 35）**：$e_{k,i} = |x_k - x_{k,\max}|$ 在直线行驶（$x_k=0$）时最大，逼近边界时反而趋近 0；且将 `rad`、`rad/s` 与无量纲 `LTR` 直接相加。
* **Y2. 2 倍硬天花板与分母内耗（Eq. 36）**：$e_{k,i}/\sum e_{k,j} \le 1$ 将权重死锁在 $2Q_{i0}$；当横摆、侧偏、侧倾同时告警时，三者平分分母使 $Q_{LTR}$ 从 $2.0\times$ 稀释回落至 $1.33\times$。
* **Y3. 单 AFS 振荡与超时**：$N_p=N_c=25$ 导致实车 Speedgoat 最大求解耗时达 $120.55\ \mathrm{ms}$（超过 $10\ \mathrm{ms}$ 采样周期 12 倍，见原文 Table V）。

---

## 三、PMPC 仓库公平复现与四组“靶向击败”实验

在仓库 [7 状态 SUV + 54 路 CarSim + NLCSNN 闭环](https://github.com/JiaMingXu2482/PMPC) 上共用同一套常量参考基准权重（来自 [func\_CostWeightingRegulation\_QuadSlacks.m](https://www.google.com/search?q=https://github.com/JiaMingXu2482/PMPC/blob/main/controller/func_CostWeightingRegulation_QuadSlacks.m)）与实时 QP 求解器，仅替换上层权重/优先级核：

1. **实验 1：FishHook $80\ \mathrm{km/h}$（空载 & $+170\ \mathrm{kg}$ 顶箱）** $\to$ 命中 **Zhao Z1+Z3** 与 **Yang Y2+Y3**：保舵段 $\dot{\varphi} \to 0$ 时 Zhao 虚拟阻尼踏空且横摆权重归零；CED-PMPC 提前 $0.25\ \mathrm{s}$ 预见能力塌陷并由 $\delta_{\text{def}}$ 平滑唤醒 DYC。
2. **实验 2：低附着高速双移线（$75\text{–}85\ \mathrm{km/h}, \mu = 0.45$）** $\to$ 命中 **Zhao Z2+Z4**：全程 $|LTR| < 0.65$ 使 Zhao 悬架休眠；CED-PMPC 零门槛连续工作并严控后轴侧偏角。
3. **实验 3：中速连续绕桩 Slalom（$50\text{–}55\ \mathrm{km/h}, \mu = 0.85$）** $\to$ 命中 **Zhao Z2**（跨 $0.65$ 阈值阶跃抖振）与 **Yang Y1+Y3**；CED-PMPC 全程 $\delta_{\text{def}} < 0$ 实现零误制动掉速。
4. **实验 4：高附着极限双移线（$95\text{–}100\ \mathrm{km/h}, \mu = 0.85$）** $\to$ 命中 **Yang Y2**（三危并发分母稀释）与 **Zhao Z1**（第二回正峰甩尾）；CED-PMPC 严格三级解耦，主动让渡厘米级车道误差保住横摆与侧翻底线。
