# S-function → MATLAB Function 迁移方案（面向 NI HIL）

**约束条件：不修改现有算法逻辑。** 凡是能做到的步骤，都以「输出逐位不变」为验收标准；
做不到的步骤（只有换求解器那一步）明确标出，并改用容差 + 闭环指标验收。

初版 2026-09-17，当日多次修订（实测 codegen / 查证 R2018a 能力 / 阶段 A·S·B 完成）。

---

## 目标环境（已确认）

| 项 | 现状 |
|---|---|
| 部署机 | 与 NI 设备相连的主机，**MATLAB R2018a** |
| 开发机 | 本机 **R2024b**（Coder / Embedded Coder / MPC Toolbox 许可齐全） |
| 工作流 | 在 2018a 上先跑初始化 → 生成 **C++** 代码 → Simulink 里输入 `NI In` / 输出 `NI Out` |
| MPC Toolbox on 2018a | **已确认可用** |

> ⚠️ **本文所有 codegen 实测结论都是在 R2024b 上得到的。**
> R2018a 的 MATLAB Coder 在类型推断（尤其 `isfield` 常量折叠、变尺寸推断）上弱得多，
> 这些结果**在 2018a 上必须重新验证**，预计会多出一批修改。

---

## 0. 现状勘察结论

### 0.1 入口结构（迁移前）

| 项 | 现状 |
|---|---|
| 类型 | **Level-1** MATLAB S-function（`switch flag` + `sfuntmpl` 模板） |
| 端口 | 62 入 / 54 出 |
| 触发 | Function-Call Generator，周期 **10 ms**（`Ts_exec`） |
| `mdlUpdate` | **空**（`sys = x;`） |
| 声明的离散状态 | `NumDiscStates = 6`，**从未使用**（实测 `x` 在 `mdlOutputs` 里出现 0 次） |
| 真实状态载体 | 16 个 `global` |

> 全部计算集中在 `mdlOutputs`，块本质上是「输入 + 持久状态 → 输出」的纯函数。
> 这正好对应 MATLAB Function block + 状态端口的语义，**入口形态不需要重新设计**。

### 0.2 天然分界：初始化 vs 每拍

| | 函数数 | 行数 | 需要 codegen 兼容 |
|---|---|---|---|
| 只在 `mdlInitializeSizes` 调用 | 8 | 358 | **否** |
| 每拍在 `mdlOutputs` 调用 | 17 | 489（+被调用者） | **是** |

**所有最棘手的构造（`evalin`/`fileread`/`regexp`/`exist`）全部落在初始化侧。**

### 0.3 初始化产出的数据量

| 变量 | 大小 | 性质 |
|---|---|---|
| `TireF` / `TireR` | 157 KB × 2 | 常量（轮胎查表） |
| `Reftraj` | 28.2 KB | 常量 |
| `VehiclePara`(48 字段) / `Constraints`(46) / `CostWeights`(22) / `MPCParameters`(14) | 18 KB | 常量 |
| **`InitialParams`** | 473 KB | **可变状态，不能当参数** |

常量合计约 **360 KB**。`InitialParams` 的 473 KB 绝大部分是三个 20000 长的诊断缓冲。

### 0.4 QP 规模（实测）

```
决策变量 n = 29        ( = Nu*Nc + Ne + Nr = 3*6 + 8 + 3 )
不等式约束 A_cons      : 96 x 29
独立边界 lb / ub       : 各 29 个
Hessian: 对称, 最小特征值 4.16e-06 > 0  => 正定
cond(H) = 1.43e13   ->  Jacobi 预缩放后 cond = 4833   (代码里已有这一步)
```

### 0.5 求解耗时与迭代分布（迁移前，quadprog，解释执行，触发周期 10 ms）

| QP | 均 ms | p99 | max | 均迭代 | p99 迭代 | max 迭代 |
|---|---|---|---|---|---|---|
| **上层 MPC** | 1.92 | 8.38 | 53.12 | 71.8 | 404 | **2500**（撞上限） |
| 分配 QP (DB) | 0.50 | 1.05 | 5.59 | 1.1 | 4 | 5 |
| 分配 QP (CDC) | 0.45 | 1.02 | 1.71 | 2.6 | 5 | 9 |

**两个分配 QP 根本不是问题。全部实时压力来自上层那一个 QP。**
单拍总耗时的 451/77/74 ms 三个离群值是第 1/2/5 拍的 MATLAB JIT 预热，codegen 后会消失。

### 0.6 变尺寸点

1. `MPCParameters.Nr = 3*(ContrlMode==1)` —— PMPC 比 baseline MPC 多 3 个 γ 变量
2. `A_cons` 行数随模式和分支变化
3. `GAMMA` 长度随 `nd` 在 `Np` 与 `3*Np` 之间切换
4. `func_SystemFurture` 里 `kap` / `dist` 在 if/else 两支尺寸不同

---

## 1. ⚠️ R2018a 决定了求解器必须换（查证结论）

### 1.1 `quadprog` 在 R2018a 上不能 codegen

| 来源 | 内容 |
|---|---|
| MathWorks 员工 Mary Fenelon，2020-04-21 | "**Code generation for quadprog is supported in R2020a**, following on support for fmincon added in R2019b." |
| 同帖原始答复，2018-08-01 | "**No, there is no work-around.** Very little of the Optimization Toolbox can have code generated." |

出处：[MATLAB Answers 413028](https://www.mathworks.com/matlabcentral/answers/413028-is-it-possible-to-do-code-generation-with-quadprog)

### 1.2 官方替代：`mpcqpsolver`（KWIK active-set）

[官方文档](https://www.mathworks.com/help/mpc/ref/mpcqpsolver.html)：

- **Introduced in R2015b** → R2018a 里一定有；MPC Toolbox 许可**已确认可用**
- Extended Capabilities 原文：*"You can use mpcqpsolver as a general-purpose QP solver that
  **supports code generation**"*
- 前提：*"The KWIK algorithm requires that the Hessian matrix, H, be positive definite."*
  → **已实测满足**（§0.4）

### 1.3 接口差异（四条，全部核实）

```matlab
[x,status,iA,lambda] = mpcqpsolver(Linv,f,A,b,Aeq,beq,iA0,options)
```

| 项 | quadprog | mpcqpsolver |
|---|---|---|
| Hessian | 直接传 `H` | 传 **`Linv`**：`L = chol(H,'lower'); Linv = L\eye(n);` |
| 不等式 | `A*x <= b` | **`A*x >= b`**（符号相反） |
| 边界 | 独立的 `lb` / `ub` | **没有** → 必须折进不等式（96 → 154 行） |
| 热启动 | 原始点 `x0` | **逻辑型活动集 `iA`** |

### 1.4 `status` 语义

| status | 含义 |
|---|---|
| `> 0` | x 最优，**status 的值就是迭代次数** |
| **`0`** | **达到最大迭代次数。解 x 可能次优，也可能不可行** |
| `-1` | 不可行 |
| `-2` | 数值错误 |

> **更正**：早前版本写过"active-set 撞上限返回的是可行但次优点，应该接受它"。
> KWIK 官方文档不是这么写的 —— `status=0` 时解**可能不可行**。
> 现有代码 `exitflag ~= 1` 就回退，与 MPC Toolbox 自己的默认行为一致，**保持现状即可**。

---

## 2. 已锁定的三个决策

| # | 决策 | 理由 |
|---|---|---|
| 1 | **实时性走「限迭代」** | 显式 MPC 直接出局（预测模型每拍重新线性化，`H`/`A_cons` 每拍都变，mp-QP 没法离线解）。OSQP 需自行集成 C 库且改变数值结果。**换 KWIK 后此项基本不再需要，见 §3 阶段 S** |
| 2 | **初始化放在 Simulink 外的独立 `.m`** | 参数在开发机 build 时解析并固化进生成代码，目标机不需要 MATLAB 工作区 |
| 3 | **求解器换 `mpcqpsolver`** | R2018a 的硬约束，见 §1 |

### 关于决策 2 的实施要点

1. **MATLAB Function block 不会自动认工作区变量** —— 必须把变量声明为 **Scope = Parameter**
2. **`InitialParams` 不能当参数** —— 它是跨拍状态
3. **工作区依赖是脆的** —— 正式交付时改用**数据字典（.sldd）**

---

## 3. 阶段划分与进度

> 顺序原则：**先啃最硬、风险最高的（换求解器），失败要早暴露。**

### 阶段 A — 冻结基准、建回归脚本 ✅ **已完成 2026-09-17**

**产出**：

| 文件 | 作用 |
|---|---|
| `baseline_0917/erd/` | quadprog 时代的基准 ERD |
| `baseline_0917_kwik/erd/` | **阶段 S 后的默认基准**（MPC 157 拍 / ZENG 174 拍 / PMPC 156 拍） |
| `baseline_0917/src/` | 38 个文件快照（**不要 addpath**，会遮蔽开发版） |
| `baseline_0917/MANIFEST.md` | 每个文件 MD5 + 冻结时参数值 + CarSim 数据集身份 + StopTime |
| `chk_regress.m` | 一条命令出 PASS/FAIL；FAIL 时定位到首次偏离的拍号和通道 |
| `func_Metrics.m` | 16 项闭环指标（阶段 S 的验收判据） |
| `func_ErdDir.m` / `func_CarSimResDir.m` | 路径解析 |

**顺带修的两个真问题**：

1. `run3b` / `sweepZ` / `run_before` / `plot_week` 原来把输出目录硬编码成
   `%TEMP%\claude\<会话UUID>\scratchpad` —— 换个会话或磁盘清理就全失效。已改用 `func_ErdDir()`。
2. 模型有未保存改动（`StopTime` 8→10）。实测 StopTime=8 会把 ZENG 提前掐断在 8.0 s
   （161 拍）而不是站点到达的 8.65 s（174 拍）。已保存为 10。

> ⚠️ **更正一条早先的误判**：2026-09-12 把一次 160→173 拍的变化归因为
> "CarSim 异步写盘导致 ERD 被截断"。事后实测证明真因是 **StopTime 从 8 改成了 10**
> （StopTime=8 → 161 拍，=10 → 174 拍，差 13 拍，与历史那次完全一致）。
> `func_WaitERD` 仍建议保留（等日志标记比靠文件大小判稳可靠），但它并未被证实修复过真实截断。

---

### 阶段 S — 换求解器 `quadprog` → `mpcqpsolver` ✅ **已完成 2026-09-17**

#### 实际做法（与原计划的偏离）

原计划写的是"重构 `func_BuildQPConstraints` 的输出"。实际改成在**求解器边界**折叠：
新增 `func_QPKwik.m` 吸收全部三条接口差异（`Linv` / 符号翻转 / 边界折进），
`func_BuildQPConstraints` **一行未动**。好处：`QPSolver=0` 时与原版逐字节等价。

| 文件 | 改动 |
|---|---|
| `func_QPKwik.m` | **新增**。按 R2018a 的 `mpcqpsolver` API 写（已在 R2024b 上验过） |
| `func_SolveMPCQP.m` | 加 `MPCParameters.QPSolver` 开关；第 9 入参/第 7 出参 `iA` 活动集 |
| `cq32019mpc.m` | `iA` 存进 `InitialParams.prevstate`；可用 `PMPC_QPSOLVER` 覆盖 |
| `run3s.m` | **新增**。按指定求解器跑三个控制器 |

#### 验收：安全类指标全部持平或改善，无一个 `!!`

| 指标 | ① MPC | ② ZENG | ③ PMPC |
|---|---|---|---|
| 越界 % | 0 → 0 | 0 → 0 | 0 → 0 |
| \|LTR\| 峰 | 0.4633 → **0.4628** | 0.4148 → **0.4124** | 0.4548 → **0.4514** |
| ey RMS | 0.2379 → 0.2379 | 0.1963 → 0.1961 | 0.2275 → 0.2275 |
| QP 失败次数 | 1 → **0** | 1 → **0** | 4 → **0** |

其余：ZENG 油门速率 RMS 2.55 → **1.44**（−43.5%）；
PMPC dTb RMS 361 → 579（+60%，但绝对值仍远低于 MPC 1268 / ZENG 1766）。

#### ⚠️ 实时性：结论反转了

| PMPC | quadprog | **KWIK** |
|---|---|---|
| 均迭代 | 71.8 | **2.5** |
| p99 迭代 | 404 | **15** |
| **max 迭代** | **2500（撞上限）** | **28** |
| max 耗时 | 53.12 ms | **7.0 ms** |
| 撞上限 / 失败 | 有 | **0 / 0** |

ZENG 同样：866 拍全部 `status>0`，max 77 次迭代、max 9 ms。

原因：KWIK 的热启动传的是**逻辑型活动集**，相邻两拍活动集几乎不变；
quadprog 的原始点热启动完全没这个效果。

> **这使 §2 决策 1（限迭代）基本不再需要**。解释执行下 max 已经只有 28/77 次，
> `MaxIter` 维持 KWIK 默认的 200 就绰绰有余，codegen 后还会大幅下降。
> 限迭代仍作为最坏情况兜底保留（`status=0` 走失败回退）。

#### 现在的默认与基准

- `MPCParameters.QPSolver` 默认 **1（KWIK）**；`PMPC_QPSOLVER=0` 可切回
- `chk_regress` 默认基准 = `baseline_0917_kwik/erd`（已验证可复现）
- quadprog 时代的基准保留在 `baseline_0917/erd`，`chk_regress('-base',...)` 随时可查

---

### 阶段 B — 抽离初始化 ✅ **已完成 2026-09-17**

| 文件 | 改动 |
|---|---|
| `setup_pmpc.m` | **新增**。原 `mdlInitializeSizes` 的 350 行实体，打包成 16 字段的 `P`（365 KB） |
| `wsget.m` | **抽出**。原为 `cq32019mpc.m` 的局部函数，`setup_pmpc` 调不到 |
| `cq32019mpc.m` | **898 → 560 行**；`mdlInitializeSizes` 从 367 行减到 46 行 |
| `run3b.m` | 每次先 `clear PMPC_P` |

`mdlInitializeSizes` 现在只做两件事：`simsizes` 端口声明 + 从 `PMPC_P` 解包到全局量。
优先用 base 工作区里已有的 `PMPC_P`，没有就现调 `setup_pmpc()`（向后兼容）。

**验收**：`chk_regress` 逐字节 PASS；目标工作流 `setup_pmpc` → `sim` 得到的 ERD 与基准 MD5 完全相同。

> **遗留的坑**：`PMPC_P` 一旦缓存在工作区，再改 `PMPC_MODE` 就**不会生效**（参数已固化）。
> `run3b` 已加 `clear PMPC_P` 防这一手；手工切模式时记得重跑 `setup_pmpc`。
> 这不是 bug —— HIL 上参数本来就是 build 时固化的，只是开发期要注意。

---

### 阶段 C+D — 编译器驱动修改 + 迁入 MATLAB Function block  ✅ **已完成 2026-09-18**

#### 已完成：把每拍逻辑抽成独立函数

| 文件 | 改动 |
|---|---|
| `pmpc_step.m` | **新增**。原 `mdlOutputs` 实体，接口 `[sys,St] = pmpc_step(u, Pm, St)` |
| `cq32019mpc.m` | **560 → 97 行**。`mdlOutputs` 只剩打包 global → 调用 → 写回 |

> 不先建 Simulink 块，而是直接对 `pmpc_step` 跑 `codegen` —— 报错以秒计返回。
> 而且这本就是最终架构：块里只写一行调用。

#### 踩到的两个坑（都是回归护栏拦下的）

1. **提前 `return` 不回写状态**。原代码靠 global 自动持久，有两处早退分支；
   改成出参后末尾的回写被跳过 → `InitialGapflag` 永远加不上去 → 控制器全程不工作
   （三个控制器 MD5 竟然相同，这才暴露出来）。已在每个 `return` 前补回写。
2. **变量名撞车**。body 里 `S` 已被代价权重矩阵占用
   （`[Q,R,S,W,V,...] = func_CostWeighting...`），状态结构体叫 `S` 会被覆盖。改名 `Pm`/`St`。

#### 已修的 codegen 错误

| # | 位置 | 问题 | 修法 |
|---|---|---|---|
| 1 | `func_StateEstimation` | 结构体被读后动态加字段 | 预声明 23+38 个字段 |
| 2 | `pmpc_step` / `func_ReportStatus` | `~isfield` 惰性建字段（6 处） | `func_InitialParams` 预声明 + NaN 哨兵 |
| 3 | `QPA_CDC`/`TireTable`/`Fiala_tire` | `error`/`warning` 里的中文字符串 | 改 ASCII |
| 4 | `func_VehicleParams` | 切线刚度 4 字段动态添加 | 预声明 |
| 5 | `setup_pmpc` | Zeng 界 3 字段动态添加 | 预声明 |
| 6 | `func_RefTraj_LocalPlanning` | `exist(...,'file')` | 固定用本地 `local_wrapToPi` |
| 7 | `func_FindBezierControlPointsND` | `varargin`+`cellfun('isempty')`+`deal` | 改普通可选参数 |

> **第 6 项差点出事**：`local_wrapToPi` 写成无条件 `mod(ang+pi,2*pi)-pi`，
> 而 MATLAB 的 `wrapToPi` 对已在 [-pi,pi] 内的角度**原样返回**。多一次 mod 的
> 浮点 round-trip 改掉末位，闭环 MD5 就不一样了 —— **回归护栏拦下的**。
> 已改成只在越界时才 wrap。

| 8 | `func_FindBezierControlPointsND` | `C1/C2` 标量初值被赋成行向量 | 改 `zeros(1,size(p,2))` |
| 9 | `func_bezierInterp` | 同 #7 的 varargin 模式 | 改普通可选参数 |
| 10 | `func_bezierInterp` | `Q` 靠索引长出来 | 预分配 |
| 11 | `func_DynamicalModel` | `off` 在 `[]` 与 `2×Np` 间冲突 | 定长 + 显式 `off_valid` 标志 |
| 12 | `func_SystemFurture` | **cell 数组**（`A_aug`/`Aprod`/`PSI`/`THETA`/`PHI`） | 改三维数组 + 直接写进预分配大矩阵 |
| 13 | `func_SolveMPCQP`/`QPA_DB`/`ReportStatus` | 遗漏的中文格式串 | 改 ASCII |
| 14 | `func_QPKwik` | `mpcqpsolver` 返回 int32 | `double()` 转换 |
| 15 | `func_QPKwik` | `iA` 初值 `false(0,1)` 与 154×1 冲突 | 改 `persistent`（尺寸由首次赋值定） |
| 16 | `func_SolveMPCQP` | `x_opt = []` 与 `nvars×1` 冲突 | 定长，失败时全零 |
| 17 | `func_SolveMPCQP` | `rho_val = x_opt(end-Nr+1:end)` 变尺寸 | `Nr` 只可能 0/3，写成定长后 3 个 |
| 18 | **全局** | `nvars = Nu*Nc+Ne+Nr` 等维度都是运行时值 | **参数传 `coder.Constant`** |

> **第 18 项是转折点**：把 `Pm` 声明为 `coder.Constant` 后，所有维度折成编译期常量，
> 剩余的变尺寸问题**一次性全部消失**。这本来就是 HIL 的目标形态（参数 build 时固化）。

> **第 12 项（cell → 三维数组）是最大的一处改动**，也是唯一一次改了几十行，
> 但 `cell2mat` 的拼装规则很规整（块行/块列），改完逐字节 PASS。

#### ⚠️ 两个分配 QP 也必须换（原计划漏了）

生成代码里一直有 `quadprog.c`，追查发现 **`func_QPA_DB` / `func_QPA_CDC` 直接调 `quadprog`**，
不在 `QPSolver` 开关管辖内。R2018a 上同样不能 codegen，必须一起换。

- 两者都只有边界约束，H 也正定（DB 有 `lam_d` 项、CDC 有 `rho_c`），满足 KWIK 前提
- **三个调用点不能共用 `func_QPKwik` 的 `persistent`** → 加 `usews` 开关：
  上层 MPC 用活动集热启动，两个分配 QP 冷启动（实测只要 1~3 次迭代，热启动无意义）
- 上层的 quadprog 对照分支用 `coder.target('MATLAB')` 隔离：生成代码时整支剔除，
  仿真里仍可用 `PMPC_QPSOLVER=0` 切回去对照

**换完的影响**（对比阶段 S 后的基准）：ey RMS 三者全 +0.0%，越界全 0，
\|LTR\| 峰 MPC 0.4628→0.4628 / ZENG 0.4124→0.4126 / PMPC 0.4514→0.4529 ——
后两个标了 `!!`，但**都仍优于 quadprog 原始基准**（0.4148 / 0.4548）。

#### ✅ 结果：`pmpc_step` 已能生成 C 代码

```
64 个 .c 文件, 6.1 MB, pmpc_step.lib 链接成功
含 qpkwik.c / mpcqpsolver.c —— KWIK 求解器确实生成成 C
不含 quadprog.c / optim*.c  —— R2018a 不支持的东西已全部排除
```

唯一告警：`func_RefTraj_LocalPlanning:100` 的 `[]` 被当作 `double [0x1]`（无害）。

#### 当前状态

- **回归全程保持逐字节 PASS**（换分配 QP 那一步除外，已按闭环指标验收并重建基准）
- 新基准 `baseline_0918/erd`，`chk_regress` 已指过去，可复现性已验证
- 历史基准保留：`baseline_0917`（quadprog）/ `baseline_0917_kwik`（仅上层 KWIK）

#### 阶段 D 的落地（2026-09-18）

- `build_mf.m` 建块 `Subsystem1/PMPC_MF`，块里只有一行 `sys = pmpc_block(u, PMPC_P);`
- `pmpc_block.m` 是块的实体：`persistent St`，首拍用 `PMPC_P.S0` 初始化，
  然后 `[sys, St] = pmpc_step(uc, PMPC_P.Pm, St);`
- `use_mf(0|1)` 在 S-function 路 / MF block 路之间切换

##### 三个必须记住的坑

1. **`PMPC_P` 必须 `Tunable = false`**（`d.Tunable=false`）。这是块层面的
   `coder.Constant`。模型默认 `DefaultParameterBehavior=Tunable`，参数就不是
   编译期常量，`nvars = Nu*Nc+Ne+Nr` 这类维度退回运行时值，块直接报
   "无法确定输出大小和/或类型"。HIL 上参数本来就是 build 时固化的。
2. **不工作的那条路必须 `Commented='on'`，光挂 Terminator 不够**。两条路都调
   `func_QPKwik`，并联跑会各自推进自己的状态、抢 CarSim 的同一份输入，结果两边都不对。
3. **块里不能用 `global`**。`global SYSLOG` 这种调试记录会让块推不出维度。
   要在块里记东西得走 `coder.extrinsic`（且只在仿真时有效）。

##### 验收：MD5 必然 FAIL，但指标全线吻合

MF block 是**编译执行**，S-function 是**解释执行**。小矩阵乘法等运算的求值顺序
不同，末位舍入就不同。实测：

```
第 1 拍     两条路输出完全相同
第 2 拍     max|diff| = 6.2e-10，而该拍 max|sys| = 6.8e+04   相对 ~1e-14
第 200 拍   max|diff| = 2.8e-05
第 400 拍   max|diff| = 46
全程        max|diff| = 536（该通道量级 5.2e+04，约 1%）
```

从 1e-14 的相对舍入长到 1%，是 QP 活动集翻转 + 闭环反馈放大的结果，**不是逻辑错误**。
调用次数两条路完全一致（778 次），已排除采样率/执行次数的可能。

闭环指标（`chk_regress -cur erd_mf`）：

| | MPC | ZENG | PMPC |
|---|---|---|---|
| 拍数 | 157 → 157 | 174 → 174 | 156 → 156 |
| 越界 % | 0.00 → 0.00 | 0.00 → 0.00 | 0.00 → 0.00 |
| \|LTR\| 峰 | 0.4628 → 0.4628 | 0.4126 → 0.4124 | 0.4529 → 0.4527 |
| 最大偏差项 | 尾段\|SW\|峰 −0.1% | 尾段\|SW\|峰 −0.9% | 差动制动RMS +0.8% |

三个控制器**拍数完全相同**（说明停机站点和轨迹一致），安全类指标全部持平或略好，
无一个 `!!`。符合用户既定的验收口径："迁移过程数值上有变化是正常的，
只要算法能正常运行就可以"。

##### MF 路径自身是逐字节可复现的

重跑一次对 `baseline_mf/erd` 仍是 PASS（`e0895a93f6f5` / `cc306e7ab39a` / `1c8e01e186ac`）。
所以**阶段 E 起 MD5 仍是硬判据，只是换基准**。

##### 旧 S-Function 已删除（2026-09-18）

`Subsystem1` 现在只剩 `x -> PMPC_MF -> stable_ctr`，S-Function 块、两个
Terminator、`use_mf.m` 全部删除，`cq32019mpc.m` 改名 `.retired` 留档。
删除后重跑回归对 `baseline_mf` 仍逐字节 PASS。

> **坑**：`find_system` **默认跳过被注释的块**。`use_mf(0/1)` 把不工作的那条
> 设成 `Commented='on'`，于是按名字删除的循环静默漏掉了它——列出来的块清单
> 里也没有，看着像已经删干净了。要确认一个块在不在，用 `get_param` 直接试，
> 或给 `find_system` 加 `'IncludeCommented','on'`。

#### 剩下的活

1. 端口分组命名（62 入 / 54 出），为 VeriStand 映射做准备

---

### 阶段 E — codegen 验证 + 计时

拆成五块（原来那三条 bullet 严重低估了 E1）。

#### E5 — 端口分组命名 ✅ **已完成 2026-09-18**

`func_PortMap.m` 给出 62 入 / 54 出 / 11 回灌的完整定义，
`func_PortMap('-doc')` 生成 `PORTMAP_HIL.md` + 两个 csv（VeriStand 映射用）。
数据来源都已核对：CarSim `Run_all.par` 的 EXPORT/IMPORT 列表、
`func_StateEstimation.m` 的 `ModelInput(i)`、`pmpc_step.m` 的 `sys` 向量、模型接线。

两个查出来的事实：

1. **输入 62 路里有 12 路控制器根本没用**（5–16：悬架压缩位移 / 地面高度 / 轮心高度）
2. **HIL 上要部署的不止 `PMPC_MF`**。纵向那 2 路（油门、制动压力）来自
   `PID velocity control` 子系统，而它的 3 个输入全部取自 `PMPC_MF` 的输出
   （sys31 Vx / sys32 Vset_pid / sys21 Md_next）。是一条纯前馈链：

   ```
   62 入 -> PMPC_MF -> 54 -> (9 直接 + 3 进 PID -> 2) -> 11 回灌
   ```

#### 打印开关 ✅ **已完成 2026-09-18**

`MPCParameters.Verbose`（`setup_pmpc` 里由 `wsget('PMPC_VERBOSE', 1)` 取，
默认开）。关掉后 `func_ReportStatus` 的每拍 `fprintf`、`func_SolveMPCQP` 的
3 个 warning、两个分配 QP 的失败 warning 全部静默，**失败计数不受影响**。

放在 `MPCParameters` 而不是 `Constraints`，是因为前者属于 `Pm`（块里是
`coder.Constant`），`Verbose=0` 时打印分支在生成代码阶段就被折掉，一行 C 都不留；
`Constraints` 属于状态 `S0`，折不掉。

验收：`PMPC_VERBOSE=1` 和 `=0` 都逐字节 PASS。

> ⚠️ **实测推翻了"关打印能提速"的假设。** RTIME 关打印 1.039/1.065/2.129
> vs 开打印 1.098/1.056/2.116（MPC/ZENG/PMPC），差异在运行间噪声内（±1%）。
> **PMPC 那 2.13 倍实时是真实计算开销**，只能靠限迭代 + 生成代码解决。

#### E1 — 建 HIL 模型并生成 C++ ✅ **已完成 2026-09-18**

现在的 `cg_step.m` / `cg_block.m` 是对 **M 函数**跑 `codegen` 出 lib，
那只验证"代码能生成"。实际 HIL 路径是 **Simulink Coder 从模型生成 C++**。

**要建新模型**：`cq3_2019` 里有 CarSim 的 `vs_sf` 块（车辆模型），
既不能生成代码也不需要——NI 上车辆在对面。新模型是
`NI In (62) -> PMPC_MF + PID velocity control -> NI Out (11 或 54)`。

当前配置与目标的差距：`TargetLang = C`（要 C++）、
`SystemTargetFile = grt.tlc`（要 ert.tlc 或 NI 的目标）。

**已确认并构建成功**：NI VeriStand Model Framework 2020 R4，
`NIVeriStand.tlc` + C++ + `NIVeriStand_vc.tmf`，产物 `pmpc_hil.dll`。

R2018a 的 Coder 比 R2024b 弱得多，如预期冒出一批修改——**7 处，分 4 轮**，
全部是等价重写（`chk_regress` 三个 MD5 逐字节不变）。
**详见 `HIL_R2018a_codegen_notes.md`**，里面记了通用规律：

> 决定数组尺寸的配置标志必须放进 `MPCParameters`（`Pm` = `coder.Constant`），
> 放在 `InitialParams`/`Constraints`（`S0` 跨拍状态）会让分支折不掉、尺寸变可变。
> 这个坑犯了 3 次。另外常量经过运行时 `struct(...)` 构造后就不再是常量。

顺带挖出一个真 bug：`func_QPKwik` 的 `persistent` 被三个调用点共用，
**活动集热启动从来没生效过**。隔离后 PMPC 的 |LTR| 峰改善 0.17%。

#### E2 — 计时与定 `MaxIter`（未开始）

当前上限：上层 MPC 2500，两个分配 QP 各 200。阶段 S 实测最大迭代
28(PMPC) / 77(ZENG)，离上限很远，等于没起约束作用。要做：

1. 统计三个控制器的迭代分布（p50 / p95 / max）
2. 在**生成的代码**上量单次迭代真实耗时（不是解释执行的 tic/toc）
3. 定 `MaxIter` 使最坏情况落进 10 ms 预算并留余量
4. 压到该值后跑 `chk_regress`，确认闭环指标不退化
5. **决定迭代用尽时的策略** —— 目前没有明确回退方案，实时系统里必须回答

#### E3 — 数值一致性（未开始）

生成代码 vs Simulink 仿真。和阶段 D 同理，不可能逐字节，判据是指标表，
基准 `baseline_mf`。

#### E4 — R2018a 复验（未开始，必须在部署机）

目前所有 codegen 结论都是 R2024b 得到的。R2018a 的 Coder 在类型推断上弱得多，
阶段 C 那 18 个错误里有些是靠 2024b 的推断能力过的，预计会再冒出一批修改。

#### 遗留的小项

`pmpc_step.m` 里的 `tic`/`toc` 在实时目标上是真实系统调用。E2 需要它来测量，
E1 定稿前应再加一道开关或改成目标侧计时。

---

## 4. 回归验收标准

```
阶段 A / B（拆函数、抽初始化）—— 硬判据：
  三份 ERD 的 MD5 与基准逐字节相同

阶段 S（换求解器）/ 阶段 D（换 MATLAB Function block）—— MD5 必然失效，改用：
  1. 逐拍数值对比：||x_new - x_old||_inf 的分布、目标函数差、约束违反量
  2. 闭环指标（三控制器 × 16 项）与基准逐项比较
  3. 判据：安全类指标（越界%、|LTR|峰）不得变差
```

现行基准 = `baseline_mf/erd`（MATLAB Function block 路径，`chk_regress` 的默认值）。
MF block 与原 S-function 之间不可能逐字节一致，但 **MF 路径自身是可复现的**，
所以对本基准 MD5 仍是硬判据。

历史基准仅留档，S-Function 块已删除，这几份**都不能再跑出来了**：

| 目录 | 对应状态 |
|---|---|
| `baseline_0918` | S-function 路径，三个 QP 全走 KWIK |
| `baseline_0917_kwik` | S-function 路径，仅上层 KWIK |
| `baseline_0917` | S-function 路径，quadprog 时代 |

必须用 `func_WaitERD` 等 CarSim 写完，且**先确认 StopTime 与基准一致**。

---

## 5. 工作量汇总

| 阶段 | 内容 | 估计 | 实际 | 数值风险 |
|---|---|---|---|---|
| A | 冻结基准 + 回归脚本 | 0.5 天 | ✅ 当天 | 无 |
| S | 换 `mpcqpsolver` | 3~5 天 | ✅ **当天** | 高（唯一改数值的阶段） |
| B | 抽离初始化 | 1~2 天 | ✅ **当天** | 无 |
| C+D | 编译器驱动 + 迁入 block | 3~5 天 | 进行中 | 低 |
| E | codegen 验证 + 计时 | 1 天 | 待做 | 低 |

阶段 S 远快于预期，因为实际做法（求解器边界折叠）比原计划（重构约束构造）风险小得多，
且 KWIK 的活动集热启动效果远超预期。

---

## 6. 不建议的做法

- **不要一次性重写。** 17 个函数同时动，对不上就失去定位能力。
- **不要先做"分模块重构"。** 模块边界已经存在（§0.2）。
- **不要用 `coder.extrinsic` 绕过。** 仿真能跑但生成不了代码 —— 而生成代码是唯一目的。
- **不要在阶段 C+D 顺手"优化"算法。** 任何顺手改动都会让 MD5 失效。
- **不要跳过并排验证**直接换求解器。
- **改文件不要用 `open(p,'w')` 直接覆盖。** 它先截断再写，写失败就剩空文件
  （2026-09-17 本文档被这样清空过一次）。写临时文件再改名。

---

## 7. 待办 / 待确认

- [ ] 在 2018a 机器上跑一遍 codegen 测试，重估 C+D 工作量
- [x] NI 侧接 C++（已确认）
- [ ] 62 入 / 54 出在迁移时按功能分组命名，否则 VeriStand 里映射 116 个端口会很痛苦
- [ ] 决定权重是否需要在线可调（`Simulink.Parameter` + StorageClass）
- [ ] `ReportStatus` 的 20000 长历史缓冲在 HIL 版本里删除
- [ ] 建议早验：拿一个平凡模型走通「C++ 生成 → NI 集成」全流程，别等最后才发现接口形态不对
