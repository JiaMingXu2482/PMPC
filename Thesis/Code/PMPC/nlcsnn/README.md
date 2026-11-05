# 当前 Simulink 减振器模型

本目录只保留当前手工接入 Simulink 的减振器实现。`startup_pmpc.m` 会将本目录加入 MATLAB 路径。

| 部分 | Simulink 调用入口 | 实现与参数 |
| --- | --- | --- |
| 正模型，1 kHz | `fwd_mf.m` | `fwd_mf_impl.m`、`fwd_weights_m.m` |
| 逆模型，10 ms | `inv_mf.m` | `inv_mf_impl_10ms.m`、`inv_weights_m.m` |
| 电流动态 | `curr_dyn_mf.m` | `curr_dyn_impl.m` |

`pmpc_mil.slx`、`zeng_mil.slx` 和 `pmpc_nodelay_mil.slx` 的减振器正逆模块通过这些函数调用；控制器根据逆模型给出的四角可达力上下界求解期望阻尼力。`mpc_mil.slx` 已按用户要求恢复为 GitHub 上一版，仍含旧 `NLCSNN_Forward_1kHz`，尚未接入本目录的当前模型。旧 v17、v14 和更早版本的减振器实现已从本目录删除，旧版历史仍可从 Git 恢复。

注意：`zeng_mil.slx` 和 `pmpc_nodelay_mil.slx` 已接入当前接口，但 CarSim 数据集需要分别 Send 给对应模型后才能做完整联仿更新。
