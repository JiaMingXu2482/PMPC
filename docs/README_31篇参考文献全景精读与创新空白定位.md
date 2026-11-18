# PMPC 仓库 31 篇参考文献全景精读与创新空白定位

> 针对提交 `ccb6f58` 新增的 [参考论文/](https://github.com/JiaMingXu2482/PMPC/tree/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87) 目录（6 个主题子目录、共 31 个 PDF 文件 / 30 篇独立文献，含 `Zeng-2025` 双语版与英文原版 2 份；`Krauze-2023` 为 0 页损坏文件待补下），整理全库清单、核心竞品对比与新论文引言立论脉络。

## 一、核心结论

仓库中的 6 个文献文件夹清晰映射出两条学术脉络：第一条是前期围绕“半主动阻尼器响应时滞 + 多时域/变步长 MPC”储备的 23 篇文献，从频域上解释了为何时滞补偿在防侧翻中无效；第二条是直接对标 [项目目标](https://github.com/JiaMingXu2482/PMPC/blob/main/docs/%E9%A1%B9%E7%9B%AE%E7%9B%AE%E6%A0%87.md) 的 8 篇底盘集成与姿态控制核心竞品。

* **纯状态变权重已高度拥挤**：“相平面横摆裕度 + LTR 侧倾指标 $\to$ 自适应变权重”已被 `Yang-2024`（TVT）、`Zeng-2025`（TTE）与 `Zhao-2026`（JBSMSE）密集占据。
* **最核心的生态空白**：最接近的 `Zhao-2026` 假设使用**理想全主动悬架**，完全忽略了半主动 CDC 在侧倾角峰值（$\dot{\varphi} \to 0$）处耗散能力归零的物理硬边界——这正是以**“视界演化能力集 $\Omega_{k+j}(\dot{\varphi}_{k+j})$ + 能力赤字驱动优先级”**突破的最佳切口。
* **两项关键理论衔接**：
  1. `Kim-2023`、`Ji-2025`、`Luan-2025` 针对路面不平度激励的 8–15 Hz 轮跳模态（12–20 ms 滞后导致 40°–90° 相位反转），而转向激发的簧上侧倾模态仅 0.8–1.5 Hz（12.7 ms 滞后不足 7°），直接解释了为何实测 $\tau_d = 10^{-4}$ 与 $12.7\ \mathrm{ms}$ 闭环 LTR 一致。
  2. [DoNascimento-2025（IEEE L-CSS）](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/MPC%E6%97%B6%E5%9F%9F%E7%BB%93%E6%9E%84%E7%90%86%E8%AE%BA/DoNascimento-2025-Constraint-horizon-in-MPC.pdf) 提出的“约束时域与预测时域分离”与 [Behrunani-2024（CDC）](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/MPC%E6%97%B6%E5%9F%9F%E7%BB%93%E6%9E%84%E7%90%86%E8%AE%BA/Behrunani-2024-Recursive-feasibility-guarantees-multi-horizon-MPC.pdf) 的约束收紧递归可行性证明，可直接支撑 Route A 的有限能力约束时域 $N_\Omega < N_p$ 设计。

---

## 二、八篇直接对标文献对比

| 文献与期刊 | 受控执行器组合 | 核心控制与优先级/权重机制 | 验证工况与主要指标 | 相对本论文的明确局限 |
| :--- | :--- | :--- | :--- | :--- |
| [Li-2022](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6/Li-2022-Integrated-longitudinal-lateral-vehicle-stability-control-TITS.pdf) (IEEE TITS) | AFS + 4WIMD 轮矩（无悬架） | 包络分区切换 MPC：$\beta$-$r$ 相平面三区离散切换目标权重与约束 | 极限工况 CarSim + HIL | 仅限平面纵横向（无侧倾）；三区硬切换边界易跳变 |
| [Zhang-2023](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6/Zhang-2023-Cooperative-strategy-trajectory-tracking-stability-4WID-TVT.pdf) (IEEE TVT) | 4WID 转向 + 独立轮矩 DYC（无悬架） | SCKF 修正侧偏刚度 + 标量归一化稳定指标协调 MPC 与 SMC | 极限工况 CarSim + HIL | 无垂向悬架与侧倾自由度；上下层分治融合 |
| [Yang-2024](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6/Yang-2024-Anti-roll-trajectory-tracking-adaptive-weight-phase-space-stable-region-TVT.pdf) (IEEE TVT) | **仅单执行器 AFS**（3-DOF） | $(\beta, r, LTR)$ 三维相空间自适应输出权重分配律（AW） | 多工况仿真 + 实车试验（跟踪误差降 11.4%） | **纯状态距离变权重已发在 TVT**；仅有 AFS 单执行器，饱和后无替代 |
| [Zeng-2025](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6/Zeng-2025-Integrated-motion-control-interaction-safety-stability-path-tracking-FRIDEV-bilingual-EN-ZH.pdf) (IEEE TTE) | 前轮转向 + 前后独立驱动/制动（无悬架） | 变步长 MPC + 改进相平面自适应权重 + 人工势场（`zeng_mil` 原型） | 仿真 + HIL | 无侧倾动力学与半主动悬架，无法感知垂向阻尼耗散限制 |
| [Fan-2025](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6/Fan-2025-Coordinated-control-strategy-AFS-DYC-stability-category-recognition-RiE.pdf) (RiE) | AFS + 差动制动 DYC（无悬架） | K-means++ 在线识别稳定性类别并输出合成权重 + 下层 SMC | CarSim + 实车（峰值横摆率降 12%） | 无预测视界与约束优化，聚类中心依赖训练工况 |
| [Zhao-2026](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6/Zhao-2026-Adaptive-MPC-coordinated-control-trajectory-tracking-yaw-roll-stability-distributed-drive-EV-JBSMSE.pdf) (JBSMSE) | **AFS + DYC + 主动悬架** | 横摆相平面与 LTR 双指标自适应权重 MPC + 轮矩/抗侧倾力矩分配 | 高速避障仿真 + HIL（侧倾角降 17%，横摆误差降 21%） | **核心突破口**：假定全主动悬架可随时出力，移植到半主动 CDC 在 $\dot{\varphi}=0$ 处将彻底落空 |
| [Ricco-2024](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E6%82%AC%E6%9E%B6%E7%B3%BB%E7%BB%9FMPC%E6%8E%A7%E5%88%B6/Ricco-2024-Nonlinear-MPC-yaw-rate-body-motion-semi-active-active-suspensions-VSD.pdf) (VSD) | 半主动/主动悬架（无 AFS/DYC） | 隐式 NMPC 固定权重 vs 自适应权重，通过前后载荷转移调横摆 | 标定模型亚极限与极限工况 | 缺少 AFS 与 DYC 直接力矩协同，隐式 NMPC 计算开销高 |
| [Sun-2025](https://github.com/JiaMingXu2482/PMPC/blob/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/CDC%E4%B8%8EMR%E9%98%BB%E5%B0%BC%E5%99%A8%E5%8D%8A%E4%B8%BB%E5%8A%A8%E6%8E%A7%E5%88%B6/Sun-2025-Optimal-safety-distribution-strategy-full-vehicle-semiactive-suspension-energy-minimization-JoE.pdf) (JoE) | 整车半主动悬架（无 AFS/DYC） | 反步法生成虚拟力 + 能量最小化安全分配 | 极端工况联合仿真 | 反应式瞬时反馈（无预测视界），过零时无法预调横纵向执行器 |

---

## 三、全部 31 个 PDF 文件分类索引

* **[车辆稳定性与轨迹跟踪集成控制](https://github.com/JiaMingXu2482/PMPC/tree/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E8%BD%A6%E8%BE%86%E7%A8%B3%E5%AE%9A%E6%80%A7%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AA%E9%9B%86%E6%88%90%E6%8E%A7%E5%88%B6)（7 个）**：`Fan-2025`、`Li-2022`、`Yang-2024`、`Zeng-2025`（原版与双语版共 2 份）、`Zhang-2023`、`Zhao-2026`。
* **[悬架系统MPC控制](https://github.com/JiaMingXu2482/PMPC/tree/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E6%82%AC%E6%9E%B6%E7%B3%BB%E7%BB%9FMPC%E6%8E%A7%E5%88%B6)（6 个）**：`Hu-2015`、`Kim-2023`（CEP）、`Luan-2025`（中国科学，双时域 Tube-MPC + GRNN）、`Ricco-2024`（VSD）、`Yang-2023`、`Zheng-2025`（滑铁卢大学硕士论文，7-DOF CDC 对比 MPC 与 Skyhook-LQR）。
* **[CDC与MR阻尼器半主动控制](https://github.com/JiaMingXu2482/PMPC/tree/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/CDC%E4%B8%8EMR%E9%98%BB%E5%B0%BC%E5%99%A8%E5%8D%8A%E4%B8%BB%E5%8A%A8%E6%8E%A7%E5%88%B6)（8 个）**：`Jenicek-2021`、`Ji-2024`（Sci. Rep.）、`Ji-2025`（MSSP）、`Kou-2025`（AJC）、`Krauze-2023`（0 页损坏文件）、`Sun-2025`（JoE）、`Xu-2024`（JVET）、`Yoon-2021`。
* **[MPC时域结构理论](https://github.com/JiaMingXu2482/PMPC/tree/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/MPC%E6%97%B6%E5%9F%9F%E7%BB%93%E6%9E%84%E7%90%86%E8%AE%BA)（6 个）**：`Behrunani-2024`（CDC）、`DoNascimento-2025`（IEEE L-CSS / [arXiv:2503.18521](https://arxiv.org/html/2503.18521v1)）、`Kim-2021`、`Kuan-2023`（IEEE IV）、`Song-2026`（TSE, DHA-MPC）、`Tan-2016`。
* **[分层多时域MPC应用](https://github.com/JiaMingXu2482/PMPC/tree/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E5%88%86%E5%B1%82%E5%A4%9A%E6%97%B6%E5%9F%9FMPC%E5%BA%94%E7%94%A8)（3 个）**：`Ishihara-2026`（CCTA）、`Kogel-2023`（IJRNC）、`Xiao-2025`。
* **[专著与教材](https://github.com/JiaMingXu2482/PMPC/tree/main/%E5%8F%82%E8%80%83%E8%AE%BA%E6%96%87/%E4%B8%93%E8%91%97%E4%B8%8E%E6%95%99%E6%9D%90)（1 个）**：`Daniilidis-2025`。
