# V25.2 → 电流动态 → V22 E0 闭环 MATLAB 包

架构（用户提供的图）：

```
F_des -> inv_mf (V25.2, 100 Hz) -> I_cmd
      -> curr_dyn_mf (上升 10 ms / 下降 19 ms, 10-90%)
      -> I_act -> fwd_mf (V22 E0, 1 kHz) -> F_pred
      -> 与 F_des 比较，形成闭环
```

## 文件清单（全部为文本 .m，无 .mat 依赖，git pull 即可用）

| 文件 | 内容 |
|---|---|
| `fwd_mf.m` | 正模型 wrapper，签名与任务书一致 |
| `fwd_mf_impl.m` | V22 E0 正模型，1 kHz，4 corner（cid 1–4） |
| `fwd_weights_m.m` | V22 E0 权重，CDC v1.0.0 best ep127，SHA `dbfb844f…351b4` |
| `inv_mf.m` | 逆模型 wrapper，签名与任务书一致 |
| `inv_mf_impl_10ms.m` | V25.2 逆模型，100 Hz 求解，1 kHz 调用，4 corner |
| `inv_weights_m.m` | V25.2 权重，best ep179，SHA `4a0567de…29577` |
| `inv_weights_ep120_m.m` | V25.2 权重，ep120（备用，见下） |
| `curr_dyn_mf.m` / `curr_dyn_impl.m` | 电流动态仿真块，一阶滞后，上升 tau 4.55 ms / 下降 tau 8.65 ms |
| `test_closedloop_v252_v22e0.m` | 单 corner 闭环冒烟测试（合成正弦，非验证数据） |

## MATLAB 下载命令（在 MATLAB 命令行里，cd 到你想放的目录后）

如果本包已推到 PMPC 仓库（见聊天里的推送确认）：

```matlab
!git clone https://github.com/JiaMingXu2482/PMPC.git
% 已有 PMPC 目录时，只更新：
!git -C PMPC pull
% 然后 cd 到本包所在目录，例如：
cd PMPC/plot/v252_v22e0_closedloop
test_closedloop_v252_v22e0
```

如果只收到 zip 附件：解压到 MATLAB 工作目录后，`cd` 到该目录，
直接运行 `test_closedloop_v252_v22e0` 即可，不需要任何 .mat 文件。

## 使用方法（Simulink MATLAB Function 块）

- 每 corner 一个块：fwd 块、inv 块、curr_dyn 块各 4 个，cid 固定为
  1=L1、2=L2、3=R1、4=R2，同一 cid 的三个块共用 corner，但各块的
  persistent 状态是独立的（fwd 块的状态、inv 块的逆 GRU 状态、
  inv 块内部的 trial 正模型状态、curr_dyn 的电流状态互不干扰）。
- 求解器：固定步长，fwd / curr_dyn / inv 块均按 1 ms 调用；
  inv 块内部只在 10 ms tick 上求解，tick 间保持输出。
  不要用变步长求解器反复在同一 t 调用这些块：t 不增加时，
  fwd / inv 块按模板行为按 1 ms 推进一次（与 v17/v19 模板一致）。
- 接线（按图）：
  - `inv_mf` 的 `Fd_N` 接 MPC 的期望力（CarSim 符号，压缩 +）。
  - `inv_mf` 的 `I_cmd` 接 `curr_dyn_mf` 的 `I_cmd`。
  - `curr_dyn_mf` 的 `I_act` 接 `fwd_mf` 的 `I_cmd` 输入端
    （fwd 块的输入名叫 I_cmd，但闭环里它实际是 I_act，不要再手动移位）。
  - `fwd_mf` 的 `F` 就是 `F_pred`，与 `F_des` 比较时注意时序（见下）。
- 复位：每次仿真前（或脚本重跑前）执行
  `clear fwd_mf_impl inv_mf_impl_10ms curr_dyn_impl`，
  否则 persistent 状态会从上一次仿真残留。

## 时序与符号（最容易错的三点，必读）

1. **V25.2 输出是下一拍的指令，不是当拍的实际电流。**
   训练定义：`I_act[k] ~= I_cmd[k-1]`，`I_cmd[k] = clip(I_pred[k-1] + delta[k])`。
   因此闭环评价应比较 `F_pred(k+1)` 与 `F_des(k)`，而不是同一 tick 的
   `F_pred(k)` 与 `F_des(k)`。同 tick 比较会多出约 10–20 ms 的表观滞后，
   那是定义，不是模型错误。`curr_dyn_impl` 头部有同样警告：
   不要在 curr_dyn 块之外再手动移位一拍，否则是双重 10 ms 延迟。
2. **正模型力在 t 拍用的是上一拍电流。**
   V22 E0 的 observer 是 `I_prev`，即力在 t 拍用 `I(t-1)`。
   这是模型本身的定义，本包严格沿用，没有再加移位。
   闭环总滞后 = V25.2 一拍定义 + 电流动态约 10/19 ms + 正模型 1 ms，
   这是按图搭建的物理链，不是 bug。
3. **符号：CarSim ↔ 台架。**
   与 v17/v19 模板一致：`x_rig = x_ref - cmpD`，`v_rig = -cmpRD`，
   `F = -F_model`。`cmpD/cmpRD` 为 CarSim 符号（压缩 +），
   `x_rig/v_rig/F_model` 为台架数据符号。如果你直接用台架数据
   （rod_length / velocity / force 列）驱动这些函数，不要用本 wrapper，
   需自行去掉符号转换，否则力符号会反。

## inv_mf 的 Flo / Fhi / F_clipped / F_ach 是什么

V25.2 本身没有力映射，无法像 v17 那样用正模型网格直接算包络。
本包的 `Flo/Fhi/F_ach` 是用 V22 E0 在 inv 块内部做的一个
**10 ms trial**：从 inv 块自己的 trial 正模型历史状态出发，
恒定候选电流（0 A、1.6 A、I_cmd）保持 10 ms、运动保持在 tick 值，
取 10 步平均力作为 trial 力。`Flo = min(F0, F16)`，
`Fhi = max(F0, F16)`，`F_clipped = clip(Fd, Flo, Fhi)`（CarSim 符号）。
这是估计值：未来 10 ms 的真实运动未知，且 trial 历史跟的是 I_cmd
而非经过电流动态的 I_act。**闭环比较请用 fwd_mf 的 F_pred，
不要用 inv 块的 F_ach。**

## 加速度与 warmup（与 Python 验证的差异）

- 本包没有加速度输入，加速度由速度差分 + 一阶滞后导出，
  但 **tau 改为 0.002 s，不是 v19 模板的 0.02 s**，改动有实测依据：
  2026-10-31 在全部 16 个验证文件上用 PyTorch 正模型对比
  （数据集加速度 vs 推导加速度，有效掩码内）：
  tau 0.02 时 pooled 力差 MAE 6.71 N、P95 30.7 N，
  6.62 Hz 文件上 MAE 差 57–79 N、ST 边沿最大 230 N
  （0.02 s 在 6.62 Hz 处相位滞后 40 度）；
  tau 0.002 时 pooled MAE 2.26 N、P95 9.3 N；
  纯差分（tau 0）时 pooled MAE 1.07 N、P95 4.8 N。
  选 0.002 是折中：6.62 Hz 相位滞后仅 4.8 度，同时仍能平滑
  单采样速度尖峰（每 1 ms 步 alpha = 0.39）。
  如果你的 CarSim 速度很干净，可把 `TAU_ACCEL` 改成 0（纯差分），
  与训练最接近。训练用的是数据集加速度，所以即使 tau 0.002，
  MATLAB 力与 Python 验证力在 ST 边沿仍有小残差，这是推导方法差异，
  不是权重错误。
- 首步 warmup 200 步（首样本预滚），与 v17/v19 模板一致。
  训练排除了每文件前 1000 样本，零状态是分布外状态，不预滚会在
  t=0 出现力尖峰。验证评价时请忽略前 1 s（与 Python 掩码一致）。

## ep120 切换方法

默认 `inv_weights_m.m` 是 best（ep179，NRMSE 口径）。
要换成 ep120（MAE 口径，val MAE 0.03750 A vs best 0.03878 A）：
把 `inv_weights_ep120_m.m` 的内容复制到 `inv_weights_m.m`
（函数名保持 `inv_weights_m`），或把 `inv_mf_impl_10ms.m` 里
`net = inv_weights_m();` 改成 `net = inv_weights_ep120_m();`，
然后 `clear inv_mf_impl_10ms` 重跑。**注意：ep120 在自由运行时
并不更优**（见下），换 checkpoint 不能解决闭环问题。

## 重要警告：本包实现的是图，但闭环本身在验证中未通过

本包忠实实现了用户给的架构，但 2026-10-31 的自由运行验证
（`V25_2_CASCADE_VALIDATION`）结论是：

- Teacher Forcing（每步给真实电流当基底）：力 MAE 32.8 N，与正模型
  自身 33.0 N 基本相同，通过。
- Free-running（基底用自己上一拍输出，即本包的部署模式）：
  力 MAE 133.1 N（best）/ 131.1 N（ep120），NRMSE 75.0% / 66.7%，
  ST 阶跃丢失，零电流工况持续输出 0.77 A 以上，大振幅工况饱和到 1.6 A。
  **不适合直接以无反馈形态用于闭环。**

因此，`test_closedloop_v252_v22e0.m` 的 MAE 大是预期结果，
不是脚本 bug。如果 MATLAB 里 F_pred 跟 F_des 跟得很紧，
反而要怀疑接线（例如实测电流漏进了逆模型基底）或状态未复位。
在拿到一个经过自由运行验证通过的逆模型之前，不要把本包当作
部署验证通过的闭环。
