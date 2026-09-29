# NLCSNN 减振器模型 MATLAB 移植

把 MR-NLCSNN 仓库的 `legacy_cpu_repro` 模型（180 epoch 最优权重）
移植为纯 MATLAB 函数，用于 PMPC / Simulink / CarSim 仿真。

## 文件

| 文件 | 说明 |
|---|---|
| `nlcsnn_damper_init.m` | 一次性加载权重，返回 `net` 结构体 |
| `nlcsnn_damper_step.m` | 核心：前向模型，单步仿真，`%#codegen` 兼容 |
| `nlcsnn_force_to_current.m` | 逆模型：期望力 → 电流（二分法），`%#codegen` 兼容 |
| `nlcsnn_weights.mat` | 全部权重 + 归一化参数（1.3 MB） |
| `test_nlcsnn_port.m` | golden-vector 测试脚本 |
| `golden_vectors.mat` | Python 参考实现生成的标准答案 |
| `export_matlab.py` | 生成两个 .mat 的 Python 脚本（可复现，注释含全部单位/公式约定） |
| `verify_against_rig.py` | 用台架实测 CSV 验证 x/v/F 方向（防正负号接反） |
| `SIGN_CONVENTIONS.md` | 方向约定说明：哪些已确定、哪些必须实测、CarSim 映射模板 |

> **接线前先看 `SIGN_CONVENTIONS.md`**，重点是第 4 节：用你手里的台架 CSV
> 跑一遍 `verify_against_rig.py`，一次把 `s_x` / `s_F` 两个符号定死。

## 模型档案（来自 checkpoint `model_cfg` / `norm_cfg`）

- 输入 `u`（归一化 6 维）：`[(x-281.0645)/62.0395, v/1000, a/50000,`
  `I/1.6, dI/dt/142.517, (T-40)/40]`，推理时限幅 ±5
- 63 维 physics NFL → 256 GELU → 256 GELU → 8 维隐状态（一阶 RK4，dt=1 ms）
- 力输出：`F = y * 7662.19` [N]
- 每步推理顺序：先 `y = predict_force(u, h)`，再 RK4 更新 `h`

## 快速验证（MATLAB）

```matlab
cd <本文件夹>
test_nlcsnn_port        % 应显示 PASS，力误差 < 1e-6 N
```

## 控制器输出是力、模型输入是电流怎么办

PMPC 输出期望阻尼力 `F_des`，而真实 CDC 减振器只能接受电流指令。
标准做法是串一个**逆模型**（力 → 电流），形成完整半主动链条：

```
PMPC → F_des → [nlcsnn_force_to_current] → i_cmd → [nlcsnn_damper_step] → F_actual → CarSim
```

`nlcsnn_force_to_current` 在固定隐状态 `h` 下求解电流。算法对
非单调的 `F-i` 曲线也鲁棒（已验证：100 个黄金状态中有 3 个非严格
单调，多在速度换向附近）：

1. 电流粗扫描：`[0, i_max]` 上取 9 点，算前向力；
2. `F_des` 限幅到可达区间 `[min F, max F]`；
3. 找包住 `F_des` 的网格区间；多个时选离 `i_prev` 最近的（保连续）；
4. 区间内局部二分 20 次。`v≈0` 无 authority 时保持上步电流。

- 该 CDC 阀是常闭式（fail-safe hard）：**电流越大阻尼越小**，
  即 `|F|` 随 `i` 单调减小，这与模型学到的方向一致。
- 半被动约束：减振器只能产生与速度反向的力。`F_des` 会先被
  限幅到可达区间 `[F(0), F(i_max)]`，要"主动"力的请求会被饱和掉
  （即 clipped-optimal 行为）。`i_max` 按你的 CDC 硬件电流上限填。

每步每角的调用顺序（`h` 为步初隐状态，`i_prev` 为上步电流）：

```matlab
[i_cmd, F_ach] = nlcsnn_force_to_current(net,x,v,a,F_des,temp,h,i_max,i_prev);
dcurr          = (i_cmd - i_prev)/dt;
[F, h_new]    = nlcsnn_damper_step(net,x,v,a,i_cmd,dcurr,temp,dt,h);
```

`h` 和 `i_prev` 各用一个 Unit Delay 回灌（初值 0）。
`v≈0` 时电流几乎无 authority，逆模型会保持上步电流以防抖动。

## Simulink 接线（每个车轮一个实例）

1. 在模型 Init 回调里：`net = nlcsnn_damper_init();`
   把 `net` 存到 base workspace，传给 MATLAB Function block 做参数。
2. MATLAB Function block 内容：
   ```matlab
   function [F, h_new] = fcn(net, x, v, a, curr, dcurr, temp, dt, h)
   [F, h_new] = nlcsnn_damper_step(net, x, v, a, curr, dcurr, temp, dt, h);
   ```
3. `h_new` 经过一个 Unit Delay（8 维，初值 0）反馈回 `h`。
4. 每个车轮用独立的 block 实例，各自维护隐状态。

输入信号（物理单位）：`x`[mm] 悬架行程、`v`[mm/s] 减振器速度、
`a`[mm/s²] 减振器加速度（`v` 滤波微分）、`curr`[A] 电流指令、
`dcurr`[A/s] 电流微分、`temp`[°C]（无动态信号时取常数 40）、
`dt`[s] 宏步长（10 ms 步内自动做 10 个 1 ms RK4 子步，输入零阶保持）。

## 精度说明

- MATLAB 实现与 PyTorch 原模型在 float64 下逐位一致
  （numpy 参考实现已对 PyTorch 验证：力 ~5e-8 归一化单位，源于 float32/64 差异）。
- 注意：原模型训练/验证用 float32；此处权重以 float64 保存，
  与原 checkpoint 的差异约 4e-4 N，可忽略。
- `nlcsnn_damper_init` 用了 `load`，只在初始化调一次；
  `nlcsnn_damper_step` 本身无文件 I/O、无 persistent，可直接用于代码生成。
