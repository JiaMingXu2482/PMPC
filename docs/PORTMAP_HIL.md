# 控制器端口映射 (50 入 / 54 出)

> 由 `func_PortMap('-doc')` 生成, 不要手改。
> 来源: CarSim `Run_all.par` 的 EXPORT/IMPORT 列表 + `func_StateEstimation.m`
> + `pmpc_step.m` 的 `sys` 向量 + 模型接线。

## 拓扑

```
50 入 --> PMPC_MF --> 54 出 --+--> 前 9 路 -------------------+
                              |                               +--> 11 路回灌被控对象
                              +--> 31/32/21 --> PID vel --> 2 路 --+
```

⚠️ HIL 上要部署的不止 `PMPC_MF`, 还有 `PID velocity control` 子系统(纵向油门/制动)。
它的 3 个输入全部取自 `PMPC_MF` 的输出, 所以是一条纯前馈链, 没有额外外部输入。

## 输入 50 路 (CarSim EXPORT 顺序 = func_StateEstimation 的 ModelInput 下标)

| # | 名称 | 单位 | 用到 | 说明 |
|---|---|---|---|---|
| 1 | `CmpRD_L1` | mm/s | 是 | 悬架压缩速度 -> V_l1..r2 (/1000 -> m/s) |
| 2 | `CmpRD_L2` | mm/s | 是 | 悬架压缩速度 -> V_l1..r2 (/1000 -> m/s) |
| 3 | `CmpRD_R1` | mm/s | 是 | 悬架压缩速度 -> V_l1..r2 (/1000 -> m/s) |
| 4 | `CmpRD_R2` | mm/s | 是 | 悬架压缩速度 -> V_l1..r2 (/1000 -> m/s) |
| 5 | `Alpha_L1` | deg | 是 | 轮胎侧偏角 |
| 6 | `Alpha_L2` | deg | 是 | 轮胎侧偏角 |
| 7 | `Alpha_R1` | deg | 是 | 轮胎侧偏角 |
| 8 | `Alpha_R2` | deg | 是 | 轮胎侧偏角 |
| 9 | `Fx_L1` | N | 是 | 轮胎纵向力 |
| 10 | `Fx_L2` | N | 是 | 轮胎纵向力 |
| 11 | `Fx_R1` | N | 是 | 轮胎纵向力 |
| 12 | `Fx_R2` | N | 是 | 轮胎纵向力 |
| 13 | `Fy_L1` | N | 是 | 轮胎侧向力 |
| 14 | `Fy_L2` | N | 是 | 轮胎侧向力 |
| 15 | `Fy_R1` | N | 是 | 轮胎侧向力 |
| 16 | `Fy_R2` | N | 是 | 轮胎侧向力 |
| 17 | `Fz_L1` | N | 是 | 轮胎垂向力 |
| 18 | `Fz_L2` | N | 是 | 轮胎垂向力 |
| 19 | `Fz_R1` | N | 是 | 轮胎垂向力 |
| 20 | `Fz_R2` | N | 是 | 轮胎垂向力 |
| 21 | `Fd_L1` | N | 是 | 减振器力(MR 实际出力) |
| 22 | `Fd_L2` | N | 是 | 减振器力(MR 实际出力) |
| 23 | `Fd_R1` | N | 是 | 减振器力(MR 实际出力) |
| 24 | `Fd_R2` | N | 是 | 减振器力(MR 实际出力) |
| 25 | `AVy_L1` | rpm | 是 | 车轮转速 (*pi/30 -> rad/s) |
| 26 | `AVy_L2` | rpm | 是 | 车轮转速 (*pi/30 -> rad/s) |
| 27 | `AVy_R1` | rpm | 是 | 车轮转速 (*pi/30 -> rad/s) |
| 28 | `AVy_R2` | rpm | 是 | 车轮转速 (*pi/30 -> rad/s) |
| 29 | `My_Dr_L1` | N*m | 是 | 车轮驱动力矩 |
| 30 | `My_Dr_L2` | N*m | 是 | 车轮驱动力矩 |
| 31 | `My_Dr_R1` | N*m | 是 | 车轮驱动力矩 |
| 32 | `My_Dr_R2` | N*m | 是 | 车轮驱动力矩 |
| 33 | `AVx` | deg/s | 是 | 侧倾角速度 |
| 34 | `AVz` | deg/s | 是 | 横摆角速度 |
| 35 | `Ax` | g | 是 | 纵向加速度 (*g -> m/s^2) |
| 36 | `Ay` | g | 是 | 侧向加速度 (*g -> m/s^2) |
| 37 | `Beta` | deg | 是 | 质心侧偏角 |
| 38 | `Roll` | deg | 是 | 侧倾角 |
| 39 | `Yaw` | deg | 是 | 航向角 |
| 40 | `Xo` | m | 是 | 全局 X |
| 41 | `Yo` | m | 是 | 全局 Y |
| 42 | `Vx` | km/h | 是 | 纵向车速 (/3.6 -> m/s) |
| 43 | `Vy` | km/h | 是 | 侧向车速 (/3.6 -> m/s) |
| 44 | `AAx` | rad/s^2 | 是 | 侧倾角加速度 (代码未换算, 直接当 rad/s^2 用) |
| 45 | `AAz` | rad/s^2 | 是 | 横摆角加速度 (代码未换算, 直接当 rad/s^2 用) |
| 46 | `Steer_L1` | deg | 是 | 左前轮转角 |
| 47 | `Steer_R1` | deg | 是 | 右前轮转角 |
| 48 | `Steer_SW` | deg | 是 | 方向盘转角 |
| 49 | `VxTarget` | km/h | 是 | 目标车速 |
| 50 | `LTR` | - | 是 | 横向载荷转移率(CarSim 自算) |

**这 50 路控制器全都用到**, 没有一路是白接的。

### 历史: 砍掉的 12 路

2026-09-18 之前 CarSim 导出 62 路, 下面这 12 路控制器从来没读过, 已从 Export
列表里去掉; 其余 50 路顺序不变, 重新编号成 1..50。

| 旧通道号 | 名称 | 单位 | 说明 |
|---|---|---|---|
| 5 | `CmpD_L1` | mm | 悬架压缩位移 |
| 6 | `CmpD_L2` | mm | 悬架压缩位移 |
| 7 | `CmpD_R1` | mm | 悬架压缩位移 |
| 8 | `CmpD_R2` | mm | 悬架压缩位移 |
| 9 | `Zgnd_L1` | m | 轮下地面高度 |
| 10 | `Zgnd_L2` | m | 轮下地面高度 |
| 11 | `Zgnd_R1` | m | 轮下地面高度 |
| 12 | `Zgnd_R2` | m | 轮下地面高度 |
| 13 | `Z_L1` | m | 轮心高度 |
| 14 | `Z_L2` | m | 轮心高度 |
| 15 | `Z_R1` | m | 轮心高度 |
| 16 | `Z_R2` | m | 轮心高度 |

将来要做路面预瞄的话, 把它们加回 Export 列表, 并在 func_StateEstimation 里
**追加**到 51.. 之后的下标 —— 不要插回中间, 否则 50 路的编号全乱。

## 输出 54 路

| # | 名称 | 单位 | 去向 | 说明 |
|---|---|---|---|---|
| 1 | `Tb_L1` | N*m | CarSim IMP_MYBK_* (Add) | 分配 QP 解出的制动力矩 |
| 2 | `Tb_L2` | N*m | CarSim IMP_MYBK_* (Add) | 分配 QP 解出的制动力矩 |
| 3 | `Tb_R1` | N*m | CarSim IMP_MYBK_* (Add) | 分配 QP 解出的制动力矩 |
| 4 | `Tb_R2` | N*m | CarSim IMP_MYBK_* (Add) | 分配 QP 解出的制动力矩 |
| 5 | `Fd_L1` | N | CarSim IMP_FD_* (Replace) | MR 半主动悬架指令力 |
| 6 | `Fd_L2` | N | CarSim IMP_FD_* (Replace) | MR 半主动悬架指令力 |
| 7 | `Fd_R1` | N | CarSim IMP_FD_* (Replace) | MR 半主动悬架指令力 |
| 8 | `Fd_R2` | N | CarSim IMP_FD_* (Replace) | MR 半主动悬架指令力 |
| 9 | `Steer_Wheel` | deg | CarSim IMP_STEER_SW (Replace) | AFS 方向盘转角(自动驾驶: 直接替换) |
| 10 | `eps1` | - | 仅记录 | 上层 QP 松弛变量 |
| 11 | `eps2` | - | 仅记录 | 上层 QP 松弛变量 |
| 12 | `eps3` | - | 仅记录 | 上层 QP 松弛变量 |
| 13 | `eps4` | - | 仅记录 | 上层 QP 松弛变量 |
| 14 | `rho_AFS` | - | 仅记录 | 优先级变量 (PMPC 模式才非零) |
| 15 | `rho_DB` | - | 仅记录 | 优先级变量 |
| 16 | `rho_CDC` | - | 仅记录 | 优先级变量 |
| 17 | `MFxmax` | N*m | 仅记录 | 纵向力矩包络上界 |
| 18 | `MFx_next` | N*m | 仅记录 | 下一拍纵向力矩 |
| 19 | `MFxmin` | N*m | 仅记录 | 纵向力矩包络下界(= -MFxmax) |
| 20 | `Mdmax` | N*m | 仅记录 | 差动制动力矩上界 |
| 21 | `Md_next` | N*m | -> PID velocity control 的 DB_active | 差动制动力矩指令 |
| 22 | `Mdmin` | N*m | 仅记录 | 差动制动力矩下界 |
| 23 | `Fyfmax` | N | 仅记录 | 前轴侧向力上界 |
| 24 | `Fyf_next` | N | 仅记录 | 下一拍前轴侧向力 |
| 25 | `Fyfmin` | N | 仅记录 | 前轴侧向力下界(= -Fyfmax) |
| 26 | `r_ssmax` | rad/s | 仅记录 | 稳态横摆角速度上界 |
| 27 | `yawrate` | rad/s | 仅记录 | 当前横摆角速度 |
| 28 | `r_ssmin` | rad/s | 仅记录 | 稳态横摆角速度下界(= -r_ssmax) |
| 29 | `LTR_real` | - | 仅记录 | 实测 LTR |
| 30 | `LTR_Npdc` | - | 仅记录 | 预测域末端 LTR |
| 31 | `Vx` | km/h | -> PID velocity control 的 Vx | 当前纵向车速 |
| 32 | `Vset_pid` | km/h | -> PID velocity control 的 Set_Vx | 纵向目标车速(滤波后) |
| 33 | `yr` | m | 仅记录 | 参考路径投影点的 Y |
| 34 | `Vtotal` | m/s | 仅记录 | 合成速度 sqrt(Vx^2+Vy^2) |
| 35 | `LTR` | - | 仅记录 | LTR(与 29 同源) |
| 36 | `alpha_r_avg` | deg | 仅记录 | 后轴平均侧偏角 |
| 37 | `Yaw` | deg | 仅记录 | 航向角 |
| 38 | `beta` | rad | 仅记录 | 质心侧偏角(弧度) |
| 39 | `CafTan` | N/rad | 仅记录 | 前轴在线切线刚度(负) |
| 40 | `CarTan` | N/rad | 仅记录 | 后轴在线切线刚度(负) |
| 41 | `Roll` | rad | 仅记录 | 侧倾角 |
| 42 | `Rollrate` | rad/s | 仅记录 | 侧倾角速度 |
| 43 | `Fx_L1` | N | 仅记录 | 轮胎纵向力(透传) |
| 44 | `Fx_L2` | N | 仅记录 | 轮胎纵向力(透传) |
| 45 | `Fx_R1` | N | 仅记录 | 轮胎纵向力(透传) |
| 46 | `Fx_R2` | N | 仅记录 | 轮胎纵向力(透传) |
| 47 | `Fy_L1` | N | 仅记录 | 轮胎侧向力(透传) |
| 48 | `Fy_L2` | N | 仅记录 | 轮胎侧向力(透传) |
| 49 | `Fy_R1` | N | 仅记录 | 轮胎侧向力(透传) |
| 50 | `Fy_R2` | N | 仅记录 | 轮胎侧向力(透传) |
| 51 | `Fz_L1` | N | 仅记录 | 轮胎垂向力(透传) |
| 52 | `Fz_L2` | N | 仅记录 | 轮胎垂向力(透传) |
| 53 | `Fz_R1` | N | 仅记录 | 轮胎垂向力(透传) |
| 54 | `Fz_R2` | N | 仅记录 | 轮胎垂向力(透传) |

真正驱动执行器的只有 **前 9 路**; 另有 3 路(21/31/32)喂给纵向 PID; 其余 42 路是记录量。
HIL 上若不需要记录, 这 42 路可以不接出去 —— 但建议留着, 出问题时没有它们很难查。

## 回灌被控对象的 11 路

| # | CarSim 通道 | 方式 | 来源 |
|---|---|---|---|
| 1 | `IMP_MYBK_L1` | Add | sys(1)  Tb_L1 |
| 2 | `IMP_MYBK_L2` | Add | sys(2)  Tb_L2 |
| 3 | `IMP_MYBK_R1` | Add | sys(3)  Tb_R1 |
| 4 | `IMP_MYBK_R2` | Add | sys(4)  Tb_R2 |
| 5 | `IMP_FD_L1` | Replace | sys(5)  Fd_L1 |
| 6 | `IMP_FD_L2` | Replace | sys(6)  Fd_L2 |
| 7 | `IMP_FD_R1` | Replace | sys(7)  Fd_R1 |
| 8 | `IMP_FD_R2` | Replace | sys(8)  Fd_R2 |
| 9 | `IMP_STEER_SW` | Replace | sys(9)  Steer_Wheel |
| 10 | `IMP_THROTTLE_ENGINE` | Add | PID velocity control 出1 |
| 11 | `IMP_PCON_BK` | Add | PID velocity control 出2 |

`Replace` = 直接替换 CarSim 内部量(自动驾驶: 方向盘、MR 阻尼力);
`Add` = 叠加到 CarSim 自身的量上(制动力矩、油门、制动压力)。

---

# HIL 模型端口 (50 入 / 20 出)

NI VeriStand 的约定是**一个信号一个块**(`NI In<k>` / `NI Out<k>` 是带掩码的
Inport/Outport, 块名即 VeriStand 通道名), 所以端口数量直接等于块数量。

- 入: 50 路 NI In 直接并成控制器的输入向量
- 出: `pmpc_step` 照样返回 54, 用 Selector 挑 20 路

**NI In 序号 = CarSim 通道号 = ModelInput 下标**, 三者完全一致;
桌面模型(cq3_2019 / pmpc_mil)和 HIL 模型共用同一份算法代码。

## NI In 50 路

| NI In | CarSim 通道号 | 名称 | 单位 |
|---|---|---|---|
| 1 | 1 | `CmpRD_L1` | mm/s |
| 2 | 2 | `CmpRD_L2` | mm/s |
| 3 | 3 | `CmpRD_R1` | mm/s |
| 4 | 4 | `CmpRD_R2` | mm/s |
| 5 | 5 | `Alpha_L1` | deg |
| 6 | 6 | `Alpha_L2` | deg |
| 7 | 7 | `Alpha_R1` | deg |
| 8 | 8 | `Alpha_R2` | deg |
| 9 | 9 | `Fx_L1` | N |
| 10 | 10 | `Fx_L2` | N |
| 11 | 11 | `Fx_R1` | N |
| 12 | 12 | `Fx_R2` | N |
| 13 | 13 | `Fy_L1` | N |
| 14 | 14 | `Fy_L2` | N |
| 15 | 15 | `Fy_R1` | N |
| 16 | 16 | `Fy_R2` | N |
| 17 | 17 | `Fz_L1` | N |
| 18 | 18 | `Fz_L2` | N |
| 19 | 19 | `Fz_R1` | N |
| 20 | 20 | `Fz_R2` | N |
| 21 | 21 | `Fd_L1` | N |
| 22 | 22 | `Fd_L2` | N |
| 23 | 23 | `Fd_R1` | N |
| 24 | 24 | `Fd_R2` | N |
| 25 | 25 | `AVy_L1` | rpm |
| 26 | 26 | `AVy_L2` | rpm |
| 27 | 27 | `AVy_R1` | rpm |
| 28 | 28 | `AVy_R2` | rpm |
| 29 | 29 | `My_Dr_L1` | N*m |
| 30 | 30 | `My_Dr_L2` | N*m |
| 31 | 31 | `My_Dr_R1` | N*m |
| 32 | 32 | `My_Dr_R2` | N*m |
| 33 | 33 | `AVx` | deg/s |
| 34 | 34 | `AVz` | deg/s |
| 35 | 35 | `Ax` | g |
| 36 | 36 | `Ay` | g |
| 37 | 37 | `Beta` | deg |
| 38 | 38 | `Roll` | deg |
| 39 | 39 | `Yaw` | deg |
| 40 | 40 | `Xo` | m |
| 41 | 41 | `Yo` | m |
| 42 | 42 | `Vx` | km/h |
| 43 | 43 | `Vy` | km/h |
| 44 | 44 | `AAx` | rad/s^2 |
| 45 | 45 | `AAz` | rad/s^2 |
| 46 | 46 | `Steer_L1` | deg |
| 47 | 47 | `Steer_R1` | deg |
| 48 | 48 | `Steer_SW` | deg |
| 49 | 49 | `VxTarget` | km/h |
| 50 | 50 | `LTR` | - |

50 路全接, 没有空口。

## NI Out 20 路

| NI Out | 来源 | 名称 | 单位 | 说明 |
|---|---|---|---|---|
| 1 | sys(1) | `Tb_L1` | N*m | 分配 QP 解出的制动力矩 |
| 2 | sys(2) | `Tb_L2` | N*m | 分配 QP 解出的制动力矩 |
| 3 | sys(3) | `Tb_R1` | N*m | 分配 QP 解出的制动力矩 |
| 4 | sys(4) | `Tb_R2` | N*m | 分配 QP 解出的制动力矩 |
| 5 | sys(5) | `Fd_L1` | N | MR 半主动悬架指令力 |
| 6 | sys(6) | `Fd_L2` | N | MR 半主动悬架指令力 |
| 7 | sys(7) | `Fd_R1` | N | MR 半主动悬架指令力 |
| 8 | sys(8) | `Fd_R2` | N | MR 半主动悬架指令力 |
| 9 | sys(9) | `Steer_Wheel` | deg | AFS 方向盘转角(自动驾驶: 直接替换) |
| 10 | sys(14) | `rho_AFS` | - | 优先级变量 (PMPC 模式才非零) |
| 11 | sys(15) | `rho_DB` | - | 优先级变量 |
| 12 | sys(16) | `rho_CDC` | - | 优先级变量 |
| 13 | sys(21) | `Md_next` | N*m | 差动制动力矩指令 |
| 14 | sys(29) | `LTR_real` | - | 实测 LTR |
| 15 | sys(30) | `LTR_Npdc` | - | 预测域末端 LTR |
| 16 | sys(31) | `Vx` | km/h | 当前纵向车速 |
| 17 | sys(32) | `Vset_pid` | km/h | 纵向目标车速(滤波后) |
| 18 | sys(38) | `beta` | rad | 质心侧偏角(弧度) |
| 19 | PID vel 出1 | `Throttle` | - | 油门开度 -> CarSim IMP_THROTTLE_ENGINE |
| 20 | PID vel 出2 | `PconBk` | MPa | 制动主缸压力 -> CarSim IMP_PCON_BK |

前 9 路是执行器指令(必须); 10-18 是诊断量, 挑的原则是"HIL 上出问题时要看什么";
19-20 来自 `PID velocity control` 子系统, 不是 `pmpc_step` 的输出。

> ⚠️ **QP 求解状态目前没有导出。** `exitflag` 只在 `pmpc_step` 内部用于统计,
> 没进 `sys` 向量。实时台架上这是个盲区 —— 参考的 NI 例子里就专门有一路
> `qp_status`。建议把它加成 `sys(55)`: 不影响任何计算, ERD 也不含控制器输出,
> 所以 `baseline_mf` 的 MD5 不会变; 代价是改 `pmpc_step` 的输出宽度和 `Demux4`。
