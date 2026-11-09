# AFS 附加转角接口说明（最小版本，2026-11-06）

## 口径

- `delta_d`：驾驶员前轮转角，作为控制器输入；
- `u(1)` / `state.u_prev(1)`：AFS 附加转角 `Delta delta`；
- 总前轮转角：`delta = delta_d + u(1)`；
- 下层输出：`SW = sat(i_sw * delta, 720 deg)`；
- 预测模型：`B` 第一列对 `Delta delta` 线性化，驾驶员项
  `-C_f * delta_d` 并入前轴偏置 `offsetPred`。

## 最小参数

只保留一个模式位，不再单独定义 AFS 权限/速率：

```matlab
M.afs_on = true;   % J-turn/DLC；鱼钩工况在 maneuver 分支中设为 false
```

AFS 边界直接用总转角机械限幅：

```matlab
if params.M.afs_on
    deltaMax =  V.delta_sw_max/V.isw - delta_d;
    deltaMin = -V.delta_sw_max/V.isw - delta_d;
else
    deltaMax = 0;
    deltaMin = 0;   % u(1)=Delta delta=0
end
duDelta = V.delta_sw_rate_max/V.isw*params.M.Ts_exec;
```

## 三种工况

| 工况 | `delta_d` | `M.afs_on` | 结果 |
|---|---|---|---|
| J-turn | 0 | true | `u(1)=Delta delta` 由 AFS 控制，`SW=i_sw*Delta delta` |
| DLC | 0 | true | 同上 |
| Fishhook | 鱼钩输入（前轮角） | false | `u(1)=0`，`SW=i_sw*delta_d` |

`delta_d` 时间序列在 `pmpc_init` 的 `maneuver` 分支中生成：
J-turn/DLC 全 0；鱼钩用构造好的鱼钩序列，并通过 Simulink 的
`delta_d` 信号输入当前采样值。控制器每拍只使用标量 `delta_d`。

## Simulink 调用

QP 块：

```matlab
function [SW,Fdes,Tb,Xlocal,Ylocal,logVec,stateNext] = ...
    fcn(meas,delta_d,Flo,Fhi,prediction,state,Reftraj,params)
%#codegen
[SW,Fdes,Tb,stateNext,Xlocal,Ylocal,logVec] = ...
    pmpc_step(meas,Reftraj,params,Flo,Fhi,state,prediction,delta_d);
end
```

Preview 块：

```matlab
function [prediction,v_pred] = fcn(delta_d,meas,state,Reftraj,params)
%#codegen
[v_pred,prediction] = pmpc_preview(meas,Reftraj,params,state,delta_d);
end
```

`delta_d` 必须是前轮角；如果鱼钩信号是方向盘角，先除以 `V.isw`。

## 注意：Fishhook + 硬约束

当前 `build_qp` 仍把 `s1/s2` 固定为 0（硬约束）。Fishhook 中
`Delta delta=0`，驾驶员输入不能被 AFS 修正；如果鱼钩输入本身让
LTR/道路/稳定性约束不可行，QP 会失败，只能走 fallback（保持 Mz/Md）。

- 论文公式本身是 soft slack；Fishhook 建议允许 `s1/s2>=0`。
- 如果坚持 J-turn/DLC 硬约束、Fishhook 软约束，可后续加
  `M.slack_hard` 开关（J-turn/DLC=1，Fishhook=0）。
