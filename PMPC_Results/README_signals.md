# 结果目录里的信号说明

三个工况的结果目录各有一套文件，命名规则相同：

```
<prefix>.mat       控制器侧日志（logVec 25 行 + 四角电流 Icmd）
<prefix>_sig.mat   车辆侧信号（从 CarSim .vs 提取的 54 个通道，画图用）
<prefix>.vs/.vsb   CarSim 原始输出（915 通道，不进 git）
```

`<prefix>` 命名：

| 工况 | 目录 | prefix |
|---|---|---|
| FishHook | `FishHook_80_mu0.9_cargo170_newplant/` | `fin190_passive`、`fh_c1`、`fh_c3_nc`、`fh_c3_c` |
| DLC | `DLC_mu0.4_interp/` | `v6_c1_{60,70,75}`、`v6_c2_*`、`v6_c3nc_*`、`v6_c3c_*` |
| Slalom | `Slalom_mu0.85_interp/` | `sv_c1_{60,80,90}`、`sv_c2_*`、`sv_c3nc_*`、`sv_c3c_*` |

`c1`=MPC，`c2`=ZENG，`c3nc`=Proposed 无补偿，`c3c`=Proposed 有补偿。

---

## 1. `<prefix>_sig.mat`（车辆侧，54 通道）

由 `tools/extract_signals.py` 从 `<prefix>.vs/.vsb` 提取，单精度、压缩存储。

| 字段 | 尺寸 | 说明 |
|---|---|---|
| `t` | (n,) | 时间 [s]，取自 CarSim `T_Record` |
| `names` | (54,) | 通道名，顺序固定 |
| `units` | (54,) | 单位 |
| `data` | (n,54) | 数据 |
| `source` | 字符串 | 生成它的 `.vs` 文件名 |

### 通道清单（顺序即 `data` 的列）

| 组 | 通道 | 单位 |
|---|---|---|
| 时间/轨迹 | `T_Record`、`Xo`、`Yo` | s, m, m |
| 车身运动 | `Vx`、`Vy`、`AVz`、`Ax`、`Ay`、`Beta` | km/h, km/h, deg/s, g, g, deg |
| 姿态 | `Roll`、`Pitch`、`Yaw`、`LTR` | deg, deg, deg, – |
| 转向/驱动 | `Steer_SW`、`Steer_L1`、`Steer_R1`、`Throttle`、`VxTarget` | deg, deg, deg, –, km/h |
| 轮胎力 | `Fx_L1..R2`、`Fy_L1..R2`、`Fz_L1..R2` | N |
| 轮胎滑移 | `Alpha_L1..R2`（侧偏角）、`Kappa_L1..R2`（纵向滑移率） | deg, – |
| 减振器 | `Fd_L1..R2`（阻尼力）、`CmpRD_L1..R2`（压缩速度）、`CmpD_L1..R2`（压缩位移） | N, mm/s, mm |
| 弹簧 | `FsExt_L1..R2` | N |

四角顺序统一为 `[L1; L2; R1; R2]`。

### 读取

Python：

```python
from scipy.io import loadmat
S = loadmat('FishHook_80_mu0.9_cargo170_newplant/fh_c1_sig.mat')
names = [str(n[0]) for n in S['names'].ravel()]
col = {n: i for i, n in enumerate(names)}
t    = S['t'].ravel()
roll = S['data'][:, col['Roll']]
```

MATLAB：

```matlab
S = load('fh_c1_sig.mat');
iRoll = find(strcmp(S.names, 'Roll'));
plot(S.t, S.data(:,iRoll));
```

### 采样率

| 工况 | 采样率 | 说明 |
|---|---|---|
| FishHook | 100 Hz | 8 s → 800 点 |
| DLC | 100 Hz | 10 s → 850–1000 点 |
| Slalom | **20 Hz** | 12 s → 241 点；CarSim Slalom 数据集的输出频率较低 |

Slalom 的 20 Hz 对 1–2 Hz 的绕桩曲线够用，但看瞬态细节（如 Md 抖动）要用
`<prefix>.mat` 里的 100 Hz `logVec`。

---

## 2. `<prefix>.mat`（控制器侧）

| 字段 | 尺寸 | 说明 |
|---|---|---|
| `D` | (25, n) | `pmpc_log`，100 Hz 控制器日志 |
| `t` | (n,) | 时间 [s] |
| `Icmd` | struct | 四角电流指令（Simulink `To Workspace`，含 `time`/`signals`） |

`D` 的行含义（见 `pmpc_core/pmpc_step.m` 头部注释）：

| 行 | 含义 | 单位 |
|---:|---|---|
| 1 | `ey` 横向偏差 | m |
| 2 | `epsi` 航向偏差 | rad |
| 3 | `phi` 侧倾角 | rad |
| 4 | `p` 侧倾角速度 | rad/s |
| 5 | `LTR`（控制器自算，左−右） | – |
| 6 | `Md_actual` 实测侧倾力矩 | N·m |
| 7 | `Md_cmd` 侧倾力矩指令 | N·m |
| 8 | `Md_min` 本拍可达下界 | N·m |
| 9 | `Md_max` 本拍可达上界 | N·m |
| 10–13 | `Fd_L1..R2` 实测阻尼力 | N |
| 14–17 | `Fdes_L1..R2` 分配出的四角力指令 | N |
| 18 | `Mz_cmd` 差动制动横摆力矩指令 | N·m |
| 19 | `SW` 方向盘指令（FishHook 只含 AFS 增量） | deg |
| 20 | `Vx` | m/s |
| 21 | `r` 横摆角速度 | rad/s |
| 22 | `deltaMd` 本拍 Md 增量 | N·m |
| 23 | `deltaMd_max` 本拍 Md 增量上限 | N·m |
| 24 | `QP_OK`（1=主 QP 成功） | – |
| 25 | `candidate_used`（1=用备用候选计划） | – |

> 注意：`D` 里的 `LTR`（第 5 行）是控制器用四轮垂向载荷自算的，
> 与 `_sig.mat` 里 CarSim 的 `LTR` 通道定义相同但不完全等价（采样率、滤波不同）。
> 论文图建议用 `_sig.mat` 的 CarSim `LTR`。

---

## 3. `.vs` / `.vsb`（原始输出，不进 git）

CarSim 的完整输出，915 通道。三个工况加起来约 440 MB，所以不提交。
需要时用 `tools/extract_signals.py` 重新提取，或直接改 `CHANNELS` 列表取别的通道：

```bash
python tools/extract_signals.py <结果目录> <prefix> [<prefix> ...]
```

不给 prefix 时会扫描目录下所有有 `.vs` 的 `<prefix>.mat`。

---

## 4. 图与脚本

每个目录下：

- `*.pdf` — 论文用图（矢量，只存 PDF，见 `AGENTS.md`）
- `*.csv` — 指标表
- `scripts/*.py` — 出图脚本，读 `<prefix>.mat` + `<prefix>.vs`

当前定稿图：

| 工况 | 图 | 脚本 |
|---|---|---|
| FishHook | `fishhook_190_final.pdf` | `scripts/plot_fishhook_final.py` |
| DLC | `v6_three_controllers_{60,70,75}.pdf`、`v6_summary_ey.pdf` | `scripts/plot_dlc_v6.py` |
| Slalom | `sv_four_controllers_{60,80,90}.pdf`、`sv_summary_ey.pdf` | `scripts/plot_slalom_final.py` |

出图脚本目前仍读 `.vs`（为了拿到全部通道）；若只想用 git 里的数据，
把 `load_vs` 换成读 `_sig.mat` 即可，两者通道名一致。
