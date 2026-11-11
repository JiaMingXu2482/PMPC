# 当前 Simulink 减振器模型

本目录只保留当前接入 Simulink 的减振器实现。

| 部分 | Simulink 调用入口 | 实现与参数 |
| --- | --- | --- |
| 正模型，1 kHz | `fwd_mf.m` | `fwd_mf_impl.m`、`fwd_weights_m.m` |
| 力界，10 ms | `force_bounds_four.m` | `force_bounds_impl_10ms.m`（复用正模型权重） |
| 逆模型，10 ms | `inv_current_impl_10ms.m` | **力界线性插值**（不再用神经网络） |
| 电流动态 | `curr_dyn_mf.m` | `curr_dyn_impl.m` |

## 逆模型：为什么从 NN 换成插值

原逆模型是 V25.2-RI0-CMD 残差网络（2073 参数）。`MR_NLCSNN` 的训练报告已经说明它不可用：

- V23-B 报告：逆 NN 对 `F_des` 的控制权限只有满量程 **5.4%**，82% 样本违反单调性，
  预测电流与上一拍电流相关 **+0.872**、与 `F_des` 相关 **−0.077**——本质是"保持上一拍电流"。
- V24 级联训练被阻断：冻结正模型的可达力包络只覆盖 **57.36%** 的真实力目标（门槛 90%）。
- V25.2 改善但不够：MAE 0.0388 A，相对 persistence 只改善 **12.8%**；SH/SR 恒电流工况反而更差。

闭环实测：NN 逆模型的四角力跟踪误差 **120–190 N**（相对误差 27–79%）。

现在改为**力界线性插值**：

```matlab
I = 1.6 * (Fdes - F_at0) / (F_at16 - F_at0);   % 再钳到 [0, 1.6]
```

`F_at0` / `F_at16` 是 `force_bounds_impl_10ms` 算出的当前速度下 0 A / 1.6 A 对应的
10 ms 平均力。插值单调、端点精确，不依赖网络。

闭环实测：力跟踪误差降到 **40–90 N**（2–3 倍改善）。

## 接线

`pmpc_mil.slx` 的 `逆模型（1000Hz）` 已改为 8 参调用：

```matlab
I_cmd = inv_current_impl_10ms(Fdes, t, Temper, cmpD, cmpRD, cid, F_at0(cid), F_at16(cid));
```

`F_at0` / `F_at16` 来自 `边界模型（100Hz）` 的 out3/out4（4×1），
在 `减振器逆模型` 子系统内按 `cid` 取本角标量。

## 已删除

- `inv_weights_m.m`（V25.2 逆网络权重）
- `inv_mf_impl_10ms.m`（逆 NN 实现）
- `inv_mf.m`（逆 NN 包装）

旧版历史可从 git 恢复；`C:\Users\user\AppData\Local\Temp\jturn_backup\` 另存了一份
`pmpc_mil_NN_backup.slx`、`inv_weights_m.m`、`inv_current_impl_10ms_NN.m`。
