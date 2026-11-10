# 如何用程序操控 CarSim 2019

本文记录本项目实际使用过的 CarSim 控制方式，供其他 agent 复现。

CarSim 2019 有两条可用路径：

1. **CarSim COM 自动化**：适合跑 standalone 工况（稳态圆周、阶跃、K&C 等）。
2. **Simulink + CarSim S-Function**：适合跑闭环 MPC 工况（DLC、J-turn）。
3. 结果都落在 CarSim 的 `Results\Run_<uuid>\LastRun.vs/.vsb`，用同一套脚本读取。

---

## 1. 环境

- CarSim 2019.0
  - Prog：`D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\`
  - Data：`D:\Program Files\CarSim2019.0\CarSim2019.0_Data\`
- MATLAB R2024b
- Windows

关键可执行文件：

```text
CarSim.exe          CarSim GUI / COM server
VSLauncher.exe      正常启动 GUI 的入口（推荐）
Programs\solvers\carsim_64.dll
Programs\solvers\Solver64.exe
```

> `Solver64.exe` 直接命令行跑需要 CarSim license/CSLM，实际不稳定；
> 本项目所有 standalone 工况都走 **COM**，闭环走 **Simulink S-Function**。

---

## 2. 启动 CarSim：GUI、COM、许可证

### 2.1 正常 GUI

用 `VSLauncher.exe` 启动，而不是直接 `CarSim.exe`：

```powershell
Start-Process "D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\VSLauncher.exe" `
  -WorkingDirectory "D:\Program Files\CarSim2019.0\CarSim2019.0_Prog"
```

等窗口标题变成：

```text
CarSim - [CarSim2019.0_Data]
```

才说明数据库加载完成、license 可用。直接 `CarSim.exe` 可能停在 `CarSim` splash。

### 2.2 COM 自动化

CarSim 注册 COM server（只需一次，管理员权限）：

```powershell
& "D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\CarSim.exe" /RegServer
```

MATLAB 连接：

```matlab
h = actxserver('CarSim.Application');
h.GoHome();
```

**重要坑：**

- 如果正常 GUI 已经在跑，`actxserver` 可能卡住（隐藏的 database/license 选择框）。
- 最稳的流程：
  1. 关掉正常 GUI；
  2. 手动启动 `/Automation` 实例：

```powershell
Start-Process "D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\CarSim.exe" `
  -ArgumentList "/Automation" `
  -WorkingDirectory "D:\Program Files\CarSim2019.0\CarSim2019.0_Data"
```

  3. 等 20–40 s，直到标题变成 `CarSim - [CarSim2019.0_Data]`；
  4. 再 `actxserver('CarSim.Application')`。
- 如果 COM 激活命令卡住，先检查是否有隐藏的 `#32770` 对话框；常见是
  “License Manager” 或 “Select Database”。
- 可运行 `CarSim.exe /SETQUICKLAUNCH` 关闭启动时的 database/license 选择框。
- 跑完 COM 后，用 `VSLauncher.exe` 把正常 GUI 启动回来，供 Simulink 使用。

---

## 3. COM API 基础

MATLAB 里 `h` 是 COM server，常用方法：

```matlab
h.GoHome();
[l,d,c] = h.GetCurrentLibInfo();       % 当前 library/dataset/category
h.Gotolibrary(lib, dataset, category); % 跳转
h.CreateNew();                         % 复制当前 dataset
h.DatasetCategory(newName, newCat);    % 重命名/改 category
h.BlueLink('#BlueLink28', lib, dataset, category); % 设置蓝色数据链接
h.Yellow('*SPEED', '60');              % 设置黄色字段
h.Ring('#RingCtrl0', '1');             % 设置下拉/单选
h.Checkbox('#CheckBox8', '1');         % 设置复选框
h.SetTable('#DiagramOne0', T);         % 设置表格（Configurable Function）
h.Run('', '');                         % 跑当前 dataset，阻塞到结束
id = h.GetCurrentDataSetID();          % 例：Run_31f17245-...
db = h.GetDatabaseFolder();            % CarSim2019.0_Data\
h.ImportParsfile(parFile, 1);          % 导入 expanded par
h.ExportParsfile(parFile, 0);          % 导出 expanded par
```

### 3.1 蓝色链接 ID 是屏幕相关的

`#BlueLink0`、`#BlueLink2`、`#BlueLink28` 等 ID 在不同屏幕上不一样。
不要猜，先枚举：

```matlab
h.Gotolibrary('CarSim Run Control','DLC80_mu0.5_PMPC','PMPC');
for id = 0:40
    try
        [lib,ds,cat,file] = h.GetBlueLink(sprintf('#BlueLink%d',id));
        if ~isempty(ds) && ~strcmp(ds,'NaN')
            fprintf('%d: [%s] [%s] [%s]\n', id, lib, ds, cat);
        end
    catch
    end
end
```

当前 Run Control 屏幕常见链接（本项目的 DLC/J-turn dataset）：

| ID | 内容 |
|---|---|
| `#BlueLink0` | Animator camera |
| `#BlueLink2` | Vehicle Assembly |
| `#BlueLink5` | Throttle |
| `#BlueLink11` | Generic VS Commands（LTR/PMPC） |
| `#BlueLink12` | Models: Simulink |
| `#BlueLink23` | Overlay run |
| `#BlueLink28` | Procedure |

---

## 4. 创建 standalone 工况（圆周 / 阶跃）

以“60 km/h、固定 30° 方向盘、1 km Square mu=0.85”为例。

### 4.1 创建转向 dataset

固定转角：

```matlab
h.Gotolibrary('Control: Steering (Open Loop)','Constant: 0 deg.','Constant');
h.CreateNew();
h.DatasetCategory('Verify Const 30','Constant');
h.Yellow('*SCALAR','30');   % 对 Constant 模式，这个黄色字段就是转角
```

> 实测：设置 `*SCALAR=30` 后，导出的 par 里 `STEER_SW_CONSTANT` 也变成 30。
> 不要用 `Yellow('STEER_SW_CONSTANT',...)`，那个 keyword 不生效。

阶跃转角：

```matlab
h.Gotolibrary('Control: Steering (Open Loop)','Step: 0 to 1 Radian at 0 seconds','Step');
h.CreateNew();
h.DatasetCategory('Verify Step 25','Step');
h.SetTable('#DiagramOne0', [0 0; 1.0 0; 1.2 25; 10 25]);
```

> 转向表格的 COM keyword 是 **`#DiagramOne0`**，不是 `STEER_SW_TABLE`。
> `GetTable('#DiagramOne0')` 可回读验证。

### 4.2 创建 procedure dataset

以 `60 km/h Const. Speed, No Steer Torque` 为模板：

```matlab
h.Gotolibrary('Procedures','60 km/h Const. Speed, No Steer Torque','Driver Controls');
h.CreateNew();
h.DatasetCategory('Verify Circle 60','Driver Controls');

h.Yellow('*SPEED','60');    % 目标车速
h.Yellow('*TSTOP','10');    % 仿真时长
h.Yellow('*SSTOP','1000');  % 里程上限，避免 80 km/h 提前停

% 路面：1 km Square, Mu = 0.85
h.BlueLink('#BlueLink0','Road: 3D Surface (All Properties)', ...
    '1 km Square, Mu = 0.85','Flat Test Pads');

% 转向：换成开环固定/阶跃
h.BlueLink('#BlueLink27','Control: Steering (Open Loop)', ...
    'Verify Const 30','Constant');
```

阶跃 procedure 只需把转向链接换成 `Verify Step 25` / `Step`。

### 4.3 创建 run dataset

从现成的 `DLC80_mu0.5_PMPC` 复制，保留当前 no-damper 车辆：

```matlab
h.Gotolibrary('CarSim Run Control','DLC80_mu0.5_PMPC','PMPC');
h.CreateNew();
h.DatasetCategory('Verify Circle 60 COM','PMPC');

% 换 procedure
h.BlueLink('#BlueLink28','Procedures','Verify Circle 60','Driver Controls');

% 清掉 Simulink / PMPC VS / overlay，变成 standalone
h.BlueLink('#BlueLink12','','','');
h.BlueLink('#BlueLink11','','','');
h.BlueLink('#BlueLink23','','','');

% 速度、时长、里程
h.Yellow('*SPEED','60');
h.Yellow('*TSTOP','10');
h.Yellow('*SSTOP','1000');

% 开启速度闭环：Ring0=1 表示 constant target speed
h.Ring('#RingCtrl0','1');
```

### 4.4 运行并取结果

```matlab
h.Run('','');                 % 阻塞到仿真结束
id = h.GetCurrentDataSetID(); % Run_<uuid>
db = h.GetDatabaseFolder();
src = fullfile(db,'Results',id);

dst = fullfile(root,'verification','carsim_runs','circle_60_30');
mkdir(dst);
copyfile(fullfile(src,'LastRun.vs'), dst);
copyfile(fullfile(src,'LastRun.vsb'), dst);
copyfile(fullfile(src,'Run_all.par'), dst);
copyfile(fullfile(src,'LastRun_echo.par'), dst);
```

**重要：** 同一个 dataset 反复跑会覆盖同一个 `Results\Run_<uuid>\`。
每跑完一个 case 必须立刻复制走，再改参数跑下一个。

---

## 5. 批量跑圆周 / 阶跃

参考脚本：

```text
verification/run_circle_matrix.m
verification/run_step_matrix.m
```

它们循环：

```text
改转向 *SCALAR / 阶跃表
改 procedure *SPEED
改 run *SPEED
h.Run('','')
copy Results\Run_<uuid>\LastRun.vs/.vsb 到 verification/carsim_runs/<case>/
```

13 个圆周 + 4 个阶跃就是这么跑出来的。

---

## 6. 闭环 MPC：Simulink + CarSim S-Function

闭环 DLC / J-turn 不能走 standalone COM，而是：

```text
Simulink pmpc_mil.slx
  → CarSim S-Function（carsim_64.dll）
  → simfile.sim
  → Results\Run_<uuid>\Run_all.par
```

### 6.1 复制模型

原 `pmpc_mil.slx` 的 `PreLoadFcn` 可能写死到 `Jturn` 路径。为了避免它
把旧路径重新加到 MATLAB path 前面，复制一份：

```matlab
copyfile('pmpc_mil.slx','pmpc_mil_verify.slx');
load_system('pmpc_mil_verify');
set_param('pmpc_mil_verify','PreLoadFcn','');
set_param('pmpc_mil_verify','InitFcn','');
save_system('pmpc_mil_verify');
bdclose('pmpc_mil_verify');
```

然后 `verification/run_case.m` 用这份 copy 跑。

### 6.2 simfile

CarSim S-Function 的 mask 参数 `SIMFILE` 指向一个 simfile，例如：

```text
SIMFILE

INPUT D:\...\verification\carsim_runs\DLC80_after\Run_all.par
ECHO  D:\...\LastRun_echo.par
LOGFILE D:\...\LastRun_log.txt
FINAL D:\...\LastRun_end.par
ERDFILE D:\...\LastRun.vs
PROGDIR D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\
DATADIR D:\Program Files\CarSim2019.0\CarSim2019.0_Data\
RESOURCEDIR D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Resources\
PRODUCT_ID CarSim
PRODUCT_VER 2019.0
ANIFILE D:\Program Files\CarSim2019.0\CarSim2019.0_Data\runs\animator.par
VEHICLE_CODE i_i
EXT_MODEL_STEP 0.001
PORTS_IMP 1,11
PORTS_EXP 1,54
END
```

`PROGDIR` / `PRODUCT_ID` 必须写；否则 S-Function 会尝试从空路径加载
`\Programs\solvers\_64.dll`。

### 6.3 运行

`verification/run_case.m` 做了：

```matlab
load_system('pmpc_mil_verify');
set_param('pmpc_mil_verify/CarSim','SIMFILE',simfile);
set_param('pmpc_mil_verify','StopTime',sprintf('%g',stop_time));

[params,Reftraj,state0,meas0] = pmpc_init(path_type); % 1=DLC,3=J-turn
[PMPC_MeasBus,PMPC_StateBus,PMPC_PredictionBus] = pmpc_bus(meas0,state0);

assignin('base','params',params);
assignin('base','Reftraj',Reftraj);
assignin('base','state0',state0);
assignin('base','meas0',meas0);
assignin('base','V_target',V_target);
assignin('base','delta_d',0);

out = sim('pmpc_mil_verify','ReturnWorkspaceOutputs','on');
```

关键点：

- `V_target` 是模型里 `v_target [ km/h ]` 常量块的变量名（大写 V）。
- `delta_d` 是驾驶员前轮转角，DLC/J-turn 设 0。
- S-Function 需要 CarSim GUI/license 在运行。若 `sim` 卡在
  “Running DLC” 但没有 `Use vehicle solver: ...`，通常是 GUI 没加载完
  或 license 被 COM 实例占用。
- 跑前确保正常 GUI 标题是 `CarSim - [CarSim2019.0_Data]`。

---

## 7. 读取 CarSim `.vs` / `.vsb`

CarSim 2019 设置 `OPT_VS_FILETYPE=2` 时输出：

- `.vs`：两个 concatenated JSON 对象（metadata + channel list）
- `.vsb`：24 字节 header + float32 数据

`.vsb` header：

```text
6 × uint32:
  [0] = 1
  [1..4] = 0
  [5] = nch
```

数据区：

```text
从 byte 24 开始，float32 little-endian
按时间行存储：nt × nch
time = XStart + (0:nt-1)*XStep
```

MATLAB 读取函数：

```text
verification/read_carsim_vs.m
```

用法：

```matlab
S = read_carsim_vs('...\LastRun.vs');
t   = S.time;
Vx  = S.data(:,S.col('Vx'))/3.6;   % Vx 单位 km/h
Vy  = S.data(:,S.col('Vy'));       % m/s
r   = S.data(:,S.col('AVz'))*pi/180;   % deg/s -> rad/s
p   = S.data(:,S.col('AV_R'))*pi/180;  % roll rate
phi = S.data(:,S.col('Roll'))*pi/180;  % deg -> rad
Ay  = S.data(:,S.col('Ay'));       % 注意：Ay 单位是 g
LTR = S.data(:,S.col('LTR'));      % 可能缺失
delta = 0.5*(S.data(:,S.col('Steer_L1'))+ ...
             S.data(:,S.col('Steer_R1')))*pi/180;
```

常用通道：

| 通道 | 含义 | 单位 |
|---|---|---|
| `Vx` | 纵向车速 | km/h |
| `Vy` | 侧向车速 | m/s |
| `AVz` | 横摆角速度 | deg/s |
| `AV_R` | 侧倾角速度（Euler） | deg/s |
| `Roll` | 侧倾角 | deg |
| `Ay` | 侧向加速度 | **g** |
| `LTR` | 载荷转移率 | - |
| `Steer_SW` | 方向盘转角 | deg |
| `Steer_L1/R1` | 前轮实际转角 | deg |
| `Fz_*` | 垂向轮胎力 | N |
| `Fy_*` | 侧向轮胎力 | N |
| `Fx_*` | 纵向轮胎力 | N |
| `Alpha_*` | 轮胎侧偏角 | deg |

注意：

- `LTR` 在 standalone run 里可能没有；用 Fz 计算：

```matlab
LTR = (Fz_R1 + Fz_R2 - Fz_L1 - Fz_L2)/(m*g);
```

- `Ay` 是 **g**，不是 m/s²。模型里要乘 `9.80665`。
- `Steer_L1/R1` 是实际前轮角，含 Ackermann/转向柔度；不要用它反推
  `isw`。`isw=18.57` 来自转向系统 dataset 的 `RACK_KIN` 斜率。
- `.vs` 只有 channel metadata；数据在 `.vsb`。不要只读 `.vs`。

---

## 8. 常见坑

1. **COM `actxserver` 卡住**
   - 正常 GUI 在跑时容易卡在隐藏的 database/license 对话框。
   - 关 GUI，手动起 `/Automation`，等标题加载完，再 `actxserver`。

2. **直接 `CarSim.exe` 停在 splash**
   - 用 `VSLauncher.exe` 正常启动。
   - 或 `CarSim.exe /SETQUICKLAUNCH` 后等待。

3. **Simulink `sim` 卡在 “Running ...”**
   - S-Function 在等 CarSim license/GUI。
   - 先启动 `VSLauncher` 并等 `CarSim - [CarSim2019.0_Data]`。
   - 不要同时跑 COM 自动化和 Simulink 闭环。

4. **S-Function 报 `Unable to load solver module \Programs\solvers\_64.dll`**
   - simfile 缺 `PROGDIR` / `DATADIR` / `PRODUCT_ID`。
   - 补全 simfile header。

5. **`Run` 后没有 `.vs`**
   - run 在 T=0 停了；看 `LastRun_log.txt` 尾部。
   - 常见：缺 `IMP_*` 台架 import（K&C 台架）、参数名拼错、road/vehicle 链接错。

6. **同一个 dataset 反复跑，结果被覆盖**
   - 每次 `h.Run` 都写同一个 `Results\Run_<uuid>\`。
   - 跑完立刻复制 `.vs/.vsb` 到自己的 case 目录。

7. **蓝色链接 ID 变了**
   - 不同屏幕 `#BlueLink` 编号不同；每次先枚举再设置。
   - 设置后回读 `GetBlueLink` 确认。

8. **转向表设置不生效**
   - 表格 keyword 是 `#DiagramOne0`，不是 `STEER_SW_TABLE`。
   - 固定转角用 `*SCALAR`，不是 `STEER_SW_CONSTANT`。

9. **`Run_all.par` 顺序**
   - K&C 程序的 `IMP_*` 需要 import 先声明；直接改 `Run_all.par` 把
     `IMPORT` 提前也不一定生效，最好用带 K&C 参考点的 vehicle load。

---

## 9. 最小可复现示例

```matlab
%% 1. 连接
h = actxserver('CarSim.Application');
h.GoHome();

%% 2. 固定 30° 转向 dataset
h.Gotolibrary('Control: Steering (Open Loop)','Constant: 0 deg.','Constant');
h.CreateNew();
h.DatasetCategory('Agent Const 30','Constant');
h.Yellow('*SCALAR','30');

%% 3. 60 km/h 圆周 procedure
h.Gotolibrary('Procedures','60 km/h Const. Speed, No Steer Torque','Driver Controls');
h.CreateNew();
h.DatasetCategory('Agent Circle 60','Driver Controls');
h.Yellow('*SPEED','60');
h.Yellow('*TSTOP','10');
h.Yellow('*SSTOP','1000');
h.BlueLink('#BlueLink0','Road: 3D Surface (All Properties)', ...
    '1 km Square, Mu = 0.85','Flat Test Pads');
h.BlueLink('#BlueLink27','Control: Steering (Open Loop)', ...
    'Agent Const 30','Constant');

%% 4. Run Control
h.Gotolibrary('CarSim Run Control','DLC80_mu0.5_PMPC','PMPC');
h.CreateNew();
h.DatasetCategory('Agent Circle 60 COM','PMPC');
h.BlueLink('#BlueLink28','Procedures','Agent Circle 60','Driver Controls');
h.BlueLink('#BlueLink12','','','');   % 清 Simulink
h.BlueLink('#BlueLink11','','','');   % 清 PMPC VS
h.BlueLink('#BlueLink23','','','');   % 清 overlay
h.Yellow('*SPEED','60');
h.Yellow('*TSTOP','10');
h.Yellow('*SSTOP','1000');
h.Ring('#RingCtrl0','1');             % 速度闭环

%% 5. 跑 + 复制结果
h.Run('','');
id = h.GetCurrentDataSetID();
db = h.GetDatabaseFolder();
src = fullfile(db,'Results',id);
dst = fullfile(pwd,'verification','carsim_runs','agent_circle_60_30');
if ~exist(dst,'dir'), mkdir(dst); end
copyfile(fullfile(src,'LastRun.vs'), dst);
copyfile(fullfile(src,'LastRun.vsb'), dst);
copyfile(fullfile(src,'Run_all.par'), dst);
copyfile(fullfile(src,'LastRun_echo.par'), dst);

%% 6. 读结果
S = read_carsim_vs(fullfile(dst,'LastRun.vs'));
fprintf('Vx=%.2f km/h, Ay=%.3f g, LTR=%.3f\n', ...
    mean(S.data(end-100:end,S.col('Vx'))), ...
    max(abs(S.data(:,S.col('Ay')))), ...
    max(abs(S.data(:,S.col('LTR')))));
```

---

## 10. 文件索引

| 文件 | 作用 |
|---|---|
| `verification/run_circle_matrix.m` | COM 批量跑 13 个圆周 |
| `verification/run_step_matrix.m` | COM 批量跑 4 个阶跃 |
| `verification/run_case.m` | Simulink + S-Function 跑 DLC/J-turn |
| `verification/read_carsim_vs.m` | MATLAB 读 `.vs`/`.vsb` |
| `verification/TUNING_LOG.md` | 本项目整定记录 |
| `verification/README.md` | 预测模型验证流程 |

---

## 11. 一句话

**Standalone 工况用 CarSim COM（`CarSim.Application` + `BlueLink`/`Yellow`/
`SetTable`/`Run`）；闭环 MPC 用 Simulink 的 CarSim S-Function。**
两条路径最终都落到 `Results\Run_<uuid>\LastRun.vs/.vsb`，用
`read_carsim_vs.m` 读取即可。
