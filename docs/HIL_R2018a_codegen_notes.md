# R2018a + NI VeriStand 代码生成踩坑记录

> 2026-09-18 首次构建成功。以后在 MIL 改完算法、重新 `make_deploy` 导出 HIL 包时，
> 若又报错，**先看这里的"通用规律"**，大概率是同一类问题。

构建成功的标志：

```
### Created library : ...\pmpc_hil_niVeriStand_rtw\pmpc_hil.dll
### Successful completion of build procedure for model: pmpc_hil
Build process completed successfully
```

---

## 一、通用规律（最重要，看这一条就够）

### 规律 1：决定数组尺寸的配置标志，必须放进 `MPCParameters`

**不能放在 `InitialParams` / `Constraints` 里。**

| | 归属 | 在块里是什么 | 分支能否折掉 |
|---|---|---|---|
| `MPCParameters`、`CostWeights`… | `PMPC_P.Pm` | **`coder.Constant`**（块参数 `Tunable=false`） | ✅ 编译期折掉 |
| `InitialParams`、`Constraints`、`WarmStart`… | `PMPC_P.S0` → 块内 `persistent` | 跨拍状态 = **运行时值** | ❌ 两支都活着 |

一旦两支都活着，而两支产生的数组行数不同，`A_cons` 之类就变成 variable-size，
一路传到 `mpcqpsolver` → `qpkwik.p`，报：

```
Dimension 1 is fixed on the left-hand side but varies on the right ([59 x 1] ~= [:? x 1]).
```

（`59 = 2*nvars+1 = 2*29+1`，是 qpkwik 内部工作尺寸，**不是**约束行数。
看到这个不要去改 `iA0`/`Am`/`bm` 的行数，要往上游找谁变尺寸。）

**这个坑犯了 4 次**：`sigma_budget`、`fx_couple`、`FishhookMode`、`Zg_Npv`/`Zg_ayfull`。
以后新加任何"开关型"或"长度型"配置，问一句：**它会不会改变某个数组的维度？**
会就放 `MPCParameters`。

第 4 次的表现形式不一样，值得单列 —— 它不是在编译期报错，而是**跑到某一拍才炸**：

```
The working dimension was selected automatically, is variable-length,
and has length 1 at run time. This is not supported.
Manually select the working dimension by supplying the DIM argument.
```

根因：`pmpc_step` 里

```matlab
r_pv = r_pv(1:min(Constraints.Zg_Npv, numel(r_pv)));   % ❌ 切片长度成了运行时值
```

`Zg_Npv` 恒为 12，但放在 `Constraints`(S0) 里 → 切片长度变可变 → `r_pv` / `c_pv` /
`t_pv` / `V_cand` 一路都成变长 → `min(V_cand)` 没法确定沿哪一维。
搬进 `MPCParameters` 即可。`Zg_ayfull`(决定 `c_pv` 走哪条分支)同理。

> **注意这一类不会在编译期暴露**，`sim` 能启动、跑几拍才报。所以 MIL 一定要
> **跑完整条工况**才算验过，别看到模型能编译就以为没事。

### 规律 2：常量经过运行时 `struct(...)` 就不再是常量

```matlab
% pmpc_step.m —— Lim 里绝大多数字段是运行时值
Lim = struct('Fyfmax',Fyfmax, ..., 'fx_couple',MPCParameters.fx_couple, ...);

% func_BuildQPConstraints.m
if Lim.fx_couple        % ❌ R2018a 当运行时值, 折不掉
if MPCParameters.fx_couple ~= 0   % ✅ 直接测源头
```

即使值来自 `Pm`，**存进运行时构造的结构体后 R2018a 就当运行时值**。
R2024b 的常量传播能看穿这层，所以**开发机上永远不会暴露**。

### 规律 4：`PMPC_VERBOSE` 会改变"哪些代码需要编译"

诊断分支都写成 `if Verbose ... end`，而 `Verbose` 来自 `MPCParameters`(编译期常量):

| | 诊断分支 | 后果 |
|---|---|---|
| `PMPC_VERBOSE = 0`(HIL, `hil_init`) | **编译期折掉** | 里面的问题全被掩盖 |
| `PMPC_VERBOSE = 1`(MIL, `mil_init`) | 真的要编译 | 问题立刻暴露 |

实例：`warning` 在 R2018a 的 MATLAB Function 块里**不支持**

```
Function 'warning' is not supported for code generation.
```

HIL build 一路顺过，因为 `Verbose=0` 把 5 处 `warning` 全折没了；
MIL 开着打印才炸出来。已全部换成 `fprintf(2, ...)`(写 stderr，codegen 支持)。

> **所以在 2018a 上验证时，开着 verbose 跑一遍更能提前发现问题。**
> `error(...)` 是支持的，不用动。

### 规律 3：R2018a 的 Coder 比 R2024b 弱得多

同一份代码在 R2024b 上 codegen 通过，不代表 R2018a 能过。差距集中在：

- 结构体字段的常量传播（规律 2）
- 隐式扩展（implicit expansion）完全不支持
- 标量性推断（数组索引结果不认为是标量）
- `cell` + `cell2mat` 的"每个元素都赋过值"检查
- `warning` 不支持（`error`、`fprintf` 支持）
- `tic` / `toc` 不支持
- 变长数组上的 `min`/`max`/`sum`/`diff` 维度歧义（运行时才报）

**开发机能做的**：`cg_step.m` 本地 codegen 能抓住一部分（尺寸冲突类），
但语言特性差异抓不到。别指望本机全过就一定能上 2018a。

---

## 二、逐条清单（按实际暴露顺序）

### 第 1 轮

| 错误 | 位置 | 修法 |
|---|---|---|
| `Function 'tic' is not supported for code generation` | `pmpc_step` | **移除计时**。试过 `coder.target` 隔离和常量条件包裹，MATLAB Function 块一律推不出输出维度（报的还是笼统的"无法确定输出大小"，看不出是 tic 引起的，靠二分才定位）。`uint64(0)` 和 `double 0` 两种预声明都不行 |
| `Expected a scalar. Non-scalars are not supported with logical operators` | `func_RefTraj_LocalPlanning` 的 `local_wrapToPi`：`if ang < -pi \|\| ang > pi` | 改成逐元素循环，每次比较标量。**注意别改成无条件 `mod`**——那会改变末位，实测让闭环结果变化 |
| `cell2mat` 要求每个元素都赋过值 | `RefU_cell = cell(Np,1)` | 换预分配数值数组 + 固定下标 |

顺带把同类的 `Q_cell`/`R_cell`/`S_cell`（`func_CostWeightingRegulation_QuadSlacks`）
和 `H_cell`（`func_BuildQPCost`）也换掉了——它们当时因为 `Error limit reached` 没报出来。

### 第 2 轮

| 错误 | 位置 | 修法 |
|---|---|---|
| `Matrix must be square` | `func_RefTraj_LocalPlanning`：`sqrt(tempDx^2 + tempDy^2)` | 改 `.^2`。`tempDx` 来自数组索引，R2018a 证不出标量就把 `^` 当**矩阵幂**。⚠️ `func_DynamicalModel` 里的 `A^2` 是**真矩阵幂**，不能动 |
| `optimoptions not supported for code generation` | `func_SolveMPCQP` 函数顶部**无条件**调用 | 挪进 `elseif coder.target('MATLAB')` 分支，跟 quadprog 一起被剔除。R2024b 靠死代码消除侥幸过了 |
| `Dimension 1 is fixed but varies ([59 x 1] ~= [:? x 1])` | `qpkwik.p` | 见规律 1 + 规律 2，改 `func_BuildQPConstraints` |
| `大小不匹配 [154 x 1] ~= [8 x 1]` | `func_QPKwik` 的 `persistent iA_ws` | 移进只有 `usews=true` 才实例化的局部函数 `local_warmstart`（见下方"副作用"） |

### 第 3 轮

| 错误 | 位置 | 修法 |
|---|---|---|
| `Size mismatch ([96 x 29] ~= [1 x 29])` | `func_SolveMPCQP`：`As = A_cons .* sc.'` | **隐式扩展 R2018a 不支持**，改 `bsxfun(@times, A_cons, sc.')` |

⚠️ 注意：**标量与数组的运算不算隐式扩展**（一直支持）。只有两边都非标量、
维度不同才是。全路径 30 处 `.*`/`./` 里只有这一处是真的。

### 第 4 轮

| 错误 | 位置 | 修法 |
|---|---|---|
| `Dimension 2 is fixed but varies ([1 x 1] ~= [1 x :?])` | `func_ReportStatus`：`InitialParams.emax.y` | 源头在 `func_error`：`local_X = X_ref(start_idx:end_idx)` 长度运行时决定 → `X_interp` 变尺寸 → `[~,idx]=min(...)` 被推成 `[1 x :?]`（Coder 认为可能为空）→ `X_interp(idx)` 不再是标量 → 传到 `PrjP.ey`。修法：`idx_interp = idx_interp_(1)` 把类型钉成标量 |
| （主动堵掉，未报出） | `pmpc_step`：`if InitialParams.FishhookMode` | 两支的 `Bezier_SK` 尺寸不同（`[]` vs `Np×1`）、`PrjP` 字段集也不同。`FishhookMode` 搬进 `MPCParameters` |

---

## 三、改过的文件汇总

| 文件 | 改了什么 |
|---|---|
| `pmpc_step.m` | 移除 tic/toc；4 处 `FishhookMode` 改测 `MPCParameters` |
| `func_RefTraj_LocalPlanning.m` | `local_wrapToPi` 逐元素；`RefU_cell`→预分配；`^2`→`.^2`；索引钉标量 |
| `func_BuildQPConstraints.m` | `Ncons_*` 直读；标志直测 `MPCParameters`；合并改预分配+固定下标 |
| `func_BuildQPCost.m` | `H_cell` → 预分配分块 |
| `func_CostWeightingRegulation_QuadSlacks.m` | `Q/R/S_cell` → 预分配块对角 |
| `func_SolveMPCQP.m` | `optimoptions` 挪进分支；`.*` → `bsxfun` |
| `func_QPKwik.m` | `persistent` 移进 `local_warmstart` |
| `setup_pmpc.m` | 新增 `MPCParameters.sigma_budget / fx_couple / FishhookMode` |

---

## 四、⚠️ 一个真 bug（顺带挖出来的）

`func_QPKwik` 的 `persistent iA_ws` 原来写在主函数里，被**三个调用点共用**：

- 上层 MPC QP：`m = 154`
- 两个分配 QP：`m = 8`

分配 QP 每拍执行 `if numel(iA_ws) ~= m, iA_ws = false(m,1); end` 把它重置成 8；
下一拍上层 QP 发现尺寸不对又重置——**活动集热启动从来没真正生效过，一直在冷启动**。

以前 `A_cons` 行数是变尺寸的，两边都退化成 `[:? x 1]` 才糊过去；行数一固定就冲突显形。
把 persistent 隔离进局部函数后它第一次真的工作了。

**影响**（相对 `baseline_mf`）：

| | `\|LTR\|` 峰 |
|---|---|
| MPC | 0.46277896 → 0.46277896（不变） |
| ZENG | 0.41244116 → 0.41244686（+14 ppm） |
| PMPC | 0.45267546 → 0.45191932（**−0.17%，改善**） |

其余指标全在 ±0.7% 内，越界率和拍数不变。
文档实测记录"冷启动 108 次迭代，热启动只要 1 次"——对 10 ms 预算是实打实的收益。

### 实测收益（2026-09-18，用 coder.extrinsic 挂记录器量的）

上层 MPC QP 的迭代次数：

| 控制器 | 冷启动 p50/p95/max | 热启动 p50/p95/max | p95 降幅 |
|---|---|---|---|
| MPC | 37 / 117 / 165 | **1 / 5 / 43** | 96% |
| ZENG | 27 / 143 / 196 | **1 / 10 / 77** | 93% |
| PMPC | 74 / 141 / 189 | **1 / 8 / 82** | 94% |

整条 DLC 的累计迭代次数：

| | 冷启动 | 热启动 | 倍数 |
|---|---|---|---|
| MPC | 38482 | 1586 | **24×** |
| ZENG | 41927 | 2538 | **17×** |
| PMPC | 64192 | 2052 | **31×** |

QP 失败次数冷热都是 0。中位数从 27~74 降到 **1** —— 绝大多数拍上一拍的活动集
直接就是这一拍的最优活动集。

**这个 bug 之前让实时性预算白白多花了一个数量级。**

### 已采纳，基准已重建

`baseline_mf/erd` = 热启动结果（`0159e0f2d286` / `3dc99a039126` / `8918d9dba06f`）。
冷启动时代的留档在 `baseline_mf_cold/erd`。

量迭代次数的办法（以后要复现）: 建一个 `qplog.m` 用 global 累加, 在 `func_QPKwik`
里 `coder.extrinsic('qplog')` 后调它。**不能直接用 global** —— MATLAB Function 块
里写 global 会让它推不出输出维度。

⚠️ 注意 `nvars` 随控制器变: PMPC 有 σ/γ 所以 Nr=3 -> 29; MPC/ZENG 的 Nr=0 -> 26。
按 `n==29` 过滤会漏掉两个控制器(第一次就这么漏了)。

---

## 四之二、输入口径从 62 路改成 50 路（2026-09-18 晚）

### 背景

CarSim 的 `I/O Channels: Export` 从 62 路砍到 50 路（去掉控制器从没读过的
`CmpD_*` / `Zgnd_*` / `Z_*`，旧编号 5-16）。当天先用的是**模型层散布**：
50 路进来，在 Simulink 里散布回一个 62 元素向量、5-16 槽填 0，这样
`func_StateEstimation` 一个字不用改。

后来改成**代码层重编号**：`func_StateEstimation` 的 52 处 `ModelInput(n)` 整体
改成 1..50，散布器全部删除。现在

> **CarSim 通道号 = `ModelInput` 下标 = NI In 序号**，三者完全一致。

### 为什么敢改（改之前先证）

不是"应该没问题"，是先跑了一遍审计：

```
生效下标 52 处，用到 50 个不同通道: {1,2,3,4} ∪ {17..62}
落在 5-16 的: 无              <<< 裁剪前提在这里被证明
重编号后恰好是 1..50          <<< 双射, 不重不漏
```

加上 `u` 在**整个项目里只被下标一次**（`pmpc_step.m` 传给 `func_StateEstimation`），
所以改动面是封闭的。

然后是**脚本批量改，不手工数**（52 处手改必出错），映射规则 `n<=4 -> n; n>=17 -> n-12`。
改完再拿 CarSim 真实的 50 路 Export 清单逐路对了一遍变量名。

### 验收

`chk_regress` 三个 ERD 与 `baseline_mf` **逐字节相同**：

| | MD5 |
|---|---|
| MPC  | `0159e0f2d286` |
| ZENG | `3dc99a039126` |
| PMPC | `8918d9dba06f` |

### 连带改动

| 文件 | 改了什么 |
|---|---|
| `func_StateEstimation.m` | 52 处下标重编号 + 50 路通道表写进函数头 |
| `func_PortMap.m` | `P.in` 变 50 行并重新编号；砍掉的 12 路留在 `P.dropped` |
| `build_hil.m` | `Mux(62)` + 常数 0 → `Mux(50)`，`pack50` |
| `build_pmpc_mil.m` | 整个 `Scatter 50 to 62` 子系统删除，CarSim 50 路直接进 PMPC_MF |
| `cq3_2019.slx` | 拆掉散布器，`x(50)` 直接进 PMPC_MF |
| `hil50.m`（新） | 远程机上**就地**把 Mux(62) 换成 Mux(50) |
| `mil50.m` | 退役删除（它的使命是散布，现在不需要了） |

### ⚠️ `hil50.m`：远程机上不要覆盖模型

部署机上那份 `pmpc_hil.slx` 的 50 个 Inport 已经手工换成 NI In 了。
**重新生成就要再换 50 次**，所以给了 `hil50.m`：

- 顺着 `PMPC_MF` 的入口往上找那个 Mux，**不看块名**（NI In 叫什么都行）
- 断言 5-16 槽全来自同一个值为 0 的 Constant，不符就停，不硬改
- 换成 `Mux(50)` 后按 `[1:4 17:62]` 的顺序接回去
- 幂等：已经是 50 口就直接返回

开发机上拿改造前的旧模型实测过：结果与 `build_hil` 重新生成的**逐条接线一致**。

### 将来要把那 12 路加回来

别插回中间（50 路的编号会全乱）。加在 **51.. 之后**，
CarSim 的 Export 列表也追加在末尾。

---

## 五、验证流程（每改一处都走一遍）

```bash
# 1. 数值零变化检查 —— 哈希必须和上一次完全相同
chk_regress('-force')

# 2. 本机 codegen（能抓尺寸类问题, 抓不到语言特性差异）
cg_step

# 3. 重打包
setup_pmpc; make_deploy

# 4. 部署机
hil_init          # 必须, 刷新 PMPC_P
Ctrl+B
```

**判据**：所有 codegen 修复都应该是**等价重写**，`chk_regress` 的三个 MD5
必须与改动前**逐字节相同**。本轮 7 处修复全部做到了。
一旦哈希变了，说明改动动了数值，必须查清楚。

**本机检查变尺寸的办法**：生成 C 后查 `emxArray`

```matlab
cfg = coder.config('lib'); cfg.GenerateCodeOnly = true;
codegen('pmpc_step','-config',cfg,'-d',d,'-args',{u_, coder.Constant(P_), S_},'-nargout',2);
```

然后 `grep -rl emxArray <出码目录>`。QP 路径上出现 `emxArray` 就是变尺寸。
修好后应该只剩 `func_RefTraj_LocalPlanning.c` / `linspace.c`（见下）。

固定尺寸的目标值（当前 HIL 配置 Nu=3 Nc=6 Ne=8 Nr=3 Np=12）：

```
nvars = 29        A_cons = 96×29 (double A_cons[2784])   b_cons = 96×1
Am = 154×29       bm = 154×1     iA0 = 154×1 logical
分配 QP: n=4, m=8
```

---

## 六、残留警告（无害，不用管）

### 1. 7 个 `Treating [] as double [0 x 1]`

`func_RefTraj_LocalPlanning` 里 `Global_x = []` 之后循环追加。
这是**变尺寸数组**，但只在局部规划器内部，**不进 QP 路径**，所以不影响。
生成代码里对应 `func_RefTraj_LocalPlanning.c` / `linspace.c` 的 `emxArray`。

要彻底消除就得改成预分配 + 计数，但那是纯优化，不是阻塞项。

### 2. 大量 `Unable to honor user-specified priorities`

`NIVeriStand In<k>` 与 `Out<k>` 的执行顺序优先级冲突。
数据依赖决定了输入必须先于输出执行，VeriStand 块的默认优先级号与之矛盾。
**不影响正确性**，Simulink 按数据依赖排序。最后会出现 `Warning: Too many errors.`
——那只是警告被截断，不是错误。

### 3. `You selected an unsupported compiler`

VeriStand 自己找到了 MSVC 10.0（Visual Studio 2010）并用它编译成功。

---

## 七、本次构建的环境事实

| | |
|---|---|
| System target file | `C:\VeriStand\2020\ModelInterface\tmw\target\NIVeriStand.tlc` |
| VeriStand | 2020.4.0.0 (2020 R4) |
| Template makefile | `NIVeriStand_vc.tmf` |
| 编译器 | Microsoft Visual C++ 10.0 (VS2010) |
| 产物 | `pmpc_hil.dll` |
| `NUMST` | **3**（三个采样率：0.001 基频 / 0.01 控制器 / 常量） |
| `NCSTATES` | **1**（`PID velocity control` 里的 `Int_I` 连续积分器，所以用 ode1 而非 FixedStepDiscrete） |
| 构建耗时 | 58 s |

⚠️ `pmpc_hil.slx` 在包里保存的是默认目标（本机 R2024b 装不了 NI 目标），
**部署机上要自己设 `NIVeriStand.tlc` + C++ 并存盘**。
重新导包后 `.slx` 会被 `make_deploy` 重新生成，**别覆盖部署机上已配好的那个**，
只拷变化的 `.m` 文件。

---

## 八、下一步（未做）

- **E2 实时性**：量单拍耗时，定 `MaxIter`。
  迭代次数已实测(见第四节): 热启动后 p95 只有 5~10 次、最坏 82 次,
  **当前上限 2500 完全是虚设的**, 离最坏情况差 30 倍。
  定 `MaxIter` 应该盯 p95 而不是 max。分配 QP 实测 max 只有 5 次, 上限 200 同样宽松。
  计时已从 `pmpc_step` 移除，要量就在**外部**循环调 `pmpc_step` 做 tic/toc。
  预算：一拍全部运算 < 10 ms，目标 5~7 ms。
- **热启动 A/B 决策**（见第四节）。
- `Int_I` 换离散积分器 → 可用 `FixedStepDiscrete`。已实测影响：MPC/ZENG 逐字节不变，
  PMPC 全部指标 ±0.0%，安全项无退化。
