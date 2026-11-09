# J-turn R70 工况调参记录（2026-11-05/06）

## 现象

- 工况：J-turn，R≈70 m；最初 60 km/h，后按 70 km/h 调整。
- 60 km/h 过 R70 只需要 `v²/R = 0.40g`，70 km/h 也只需要约 `0.55g`，均远小于 `mu=0.85`，物理上不需要 DB。
- 原控制器在 60 km/h 下仍平均使用约 1181 N·m、峰值 4066 N·m 的横摆力矩，车速从 60 km/h 掉到 16 km/h 左右。
- 70 km/h 下调参后 DB 基本不介入，但出现约 0.42 m 的稳态横向偏差；这是权重/控制结构问题，不是轮胎极限。

## 原因

1. **实际加载的控制器不是当前 Jturn 目录这一份。**
   `pmpc_mil.slx` 保存的 `PreLoadFcn/InitFcn` 仍写死
   `D:\...\Thesis\Code\PMPC`。PMPC 目录里的 `pmpc_step.m` 是
   `Wb=0`、`R=[0.2;0.2;0.2]`，即完全没惩罚 DB 激活量；
   `pmpc_init.m` 还是 DLC / `mu=0.5` 的旧配置。
   当前 `.autosave` 里已经改成 Jturn 根目录；保存的 slx 早期版本没有保存这些修改，现已修正。

2. **即使使用 Jturn 这一份，权重也严重偏向 DB。**
   - `R_delta = 2000/duDelta² ≈ 4.4e7`
   - `R_Mz = 0.2/duMz² ≈ 2.1e-6`
   转向增量的代价比 Mz 增量大约 10^13 倍。
   - `R` 只惩罚增量；稳态 Mz 可以用零增量一直保持，原目标函数里没有绝对 Mz 惩罚。
   - `gamma_DB` 的惩罚只有 `Wb=100`，而且 gamma 是整个时域一个标量，压不住持续 DB。
   - 结果：直线段 `ey=0.10 m`、`epsi=1.1°` 这种小误差，QP 就给出 3278 N·m 的 Mz。

3. **纵向协调把油门锁死。**
   原 `lon_arbitrate` 在 `ΣTb>50 N·m` 时把 `thr_cap` 锁在激活瞬间的值，DYC 持续期间不恢复；DB 的制动阻力把车速越拖越低。

4. **旧保存版 slx 的目标车速是 100 km/h、StopTime 6.5 s。**
   旧 autosave 是 60 km/h / 10 s；2026-11-06 已按 70 km/h / 10 s 保存。
   若重新打开旧版，会先加速到 100，而 R70 在 100 km/h 需要约 1.14g，必然大量使用 DB。

## 修改

### 当前“性能优先”版（70 km/h 定稿）

- `pmpc_core/pmpc_init.m`
  - `M.R_norm = [20; 200; 0.2]`
  - `M.w_gamma = 0`：完全关闭 gamma_DB 激活量惩罚
  - `M.w_db = 1e-5`：只保留很小的绝对 Mz 惩罚，压掉直道上无用的稳态 DB
- `pmpc_core/pmpc_step.m`
  - `Q_ey`: 50 → **2000**
  - `Q_epsi`: 80 → **10**（优先横向位置，不优先航向）
  - 道路/稳定性松弛 `s1/s2` 固定为 0：`lb=ub=0`，不使用松弛变量，约束变硬。
  - R/Wb/w_db 继续从 `params.M` 读取。
- `pmpc_core/lon_arbitrate.m`
  - `TB_ON=150`、`TB_OFF=50`、`HOLD_T=0.15`、`RATE_UP=1.5`。
  - DYC 激活时也逐步放开油门上限，最低保留 `max(thr_prev,0.5)`，避免把车拖停。
- `pmpc_scenario_setup.m`（原 `pmpc_jturn_setup.m`，已合并）
  - 把模型的 `PreLoadFcn/InitFcn` 根目录改成本目录，而不是写死 PMPC。
  - 写入 `v_target=70 km/h`、`StopTime=10 s`，保存并重开模型。

### 上一版“均衡”参数（60 km/h 历史对照）

- `M.w_gamma=2000`、`M.w_db=5e-5`、`Q_ey=50`、`Q_epsi=80`、软松弛。
- 60 km/h 下速度能保持，但 70 km/h 下 `ey` 会停在约 -0.42 m；
  这版说明“DB 惩罚太重会让控制器接受横向偏差”。

## 验证结果

### 70 km/h / R70，性能优先版，10 s

| 指标 | 结果 |
| --- | ---: |
| 车速范围 | 69.7–70.2 km/h |
| `ey` 弯中稳态 | 约 -0.03 ~ -0.07 m |
| `ey` 最大（出弯瞬态） | 0.21 m |
| 平均 \|Mz\| | 473 N·m |
| 峰值 \|Mz\| | 1122 N·m |
| QP 成功率 | 100% |
| 松弛变量 | 未使用（硬约束，QP 仍 100% 成功） |

出弯 t≈7.5 s 有一个 -0.21 m 的瞬态，t≈8.5 s 回到 0.02 m 以内。
如果要求“全程 max|ey|<0.1”，需要再加 `ey` 积分/曲率前馈；单纯调 `Q_ey` 会在
“瞬态超调”和“持续 DB”之间权衡。

### 60 km/h 历史对照（上一版均衡参数）

| 指标 | 原 Jturn 代码 | 均衡调参后 |
| --- | ---: | ---: |
| 车速范围 | 16.4–60.0 km/h | 59.7–60.3 km/h |
| 平均 \|Mz\| | 1181 N·m | 158 N·m |
| 峰值 \|Mz\| | 4066 N·m | 1365 N·m |
| 最大 \|ey\| | 0.54 m | 0.58 m |
| QP 成功率 | 100% | 100% |

## 使用

在 MATLAB 命令行运行一次：

```matlab
pmpc_scenario_setup(3)
```

它会保存当前模型、修正根目录/目标车速/StopTime，然后关闭并重新打开模型。
重新打开后再运行仿真，确保 MATLAB Function 块重新解析本目录的
`pmpc_step.m`、`pmpc_preview.m` 等文件。

如果只想手工检查，确认：

```matlab
which pmpc_step      % 应指向 ...\Code\Jturn\pmpc_core\pmpc_step.m
params.M.w_gamma     % 0
params.M.w_db        % 1e-05
params.M.R_norm      % [20;200;0.2]
params.road.mu       % 0.85
```

如果只想看当前 Q 权重，`pmpc_step.m` 里是：

```matlab
Q=diag([0,0,100,0,2000,10,0]./ ...
```
