function P = func_PortMap(mode)
%FUNC_PORTMAP  控制器 50 入 / 54 出 的通道定义 (HIL 端口命名 + VeriStand 映射)
%   P = func_PortMap()      返回结构
%   func_PortMap('-doc')    额外写出 PORTMAP_HIL.md / portmap_in.csv / portmap_out.csv
%
%   数据来源(都已核对, 不是猜的):
%     50 入 —— CarSim 展开参数文件 Run_all.par 里的 EXPORT 列表(顺序即通道号),
%              用途对照 func_StateEstimation.m 里的 ModelInput(i)
%     54 出 —— pmpc_step.m 末尾那个 sys = [...] 向量
%     11 回灌 —— Run_all.par 的 IMPORT 列表 + 模型接线(Demux4/Demux -> Mux3)
%
%   ⚠️ HIL 上要部署的不止 PMPC_MF: 纵向那 2 路(油门/制动压力)来自
%      "PID velocity control" 子系统, 它的 3 个输入又全部取自 PMPC_MF 的输出
%      (sys31 Vx / sys32 Vset_pid / sys21 Md_next)。是一条纯前馈链:
%           50 入 -> PMPC_MF -> 54 -> (9 直接 + 3 进 PID -> 2) -> 11 回灌
if nargin < 1, mode = ''; end

% ---------------- 62 入的历史全量表 (下面会砍成 50 并重新编号) ----------------
C = {'L1','L2','R1','R2'};
in = cell(0,5);   % {通道号, 名字, 单位, 用途, 是否被控制器使用}
in = local_quad(in, 1,  'CmpRD', 'mm/s',  '悬架压缩速度 -> V_l1..r2 (/1000 -> m/s)', 1, C);
in = local_quad(in, 5,  'CmpD',  'mm',    '悬架压缩位移', 0, C);
in = local_quad(in, 9,  'Zgnd',  'm',     '轮下地面高度', 0, C);
in = local_quad(in, 13, 'Z',     'm',     '轮心高度',     0, C);
in = local_quad(in, 17, 'Alpha', 'deg',   '轮胎侧偏角',   1, C);
in = local_quad(in, 21, 'Fx',    'N',     '轮胎纵向力',   1, C);
in = local_quad(in, 25, 'Fy',    'N',     '轮胎侧向力',   1, C);
in = local_quad(in, 29, 'Fz',    'N',     '轮胎垂向力',   1, C);
in = local_quad(in, 33, 'Fd',    'N',     '减振器力(MR 实际出力)', 1, C);
in = local_quad(in, 37, 'AVy',   'rpm',   '车轮转速 (*pi/30 -> rad/s)', 1, C);
in = local_quad(in, 41, 'My_Dr', 'N*m',   '车轮驱动力矩', 1, C);
sng = { 45,'AVx','deg/s','侧倾角速度',1; 46,'AVz','deg/s','横摆角速度',1;
        47,'Ax','g','纵向加速度 (*g -> m/s^2)',1; 48,'Ay','g','侧向加速度 (*g -> m/s^2)',1;
        49,'Beta','deg','质心侧偏角',1;  50,'Roll','deg','侧倾角',1;
        51,'Yaw','deg','航向角',1;       52,'Xo','m','全局 X',1;
        53,'Yo','m','全局 Y',1;          54,'Vx','km/h','纵向车速 (/3.6 -> m/s)',1;
        55,'Vy','km/h','侧向车速 (/3.6 -> m/s)',1;
        56,'AAx','rad/s^2','侧倾角加速度 (代码未换算, 直接当 rad/s^2 用)',1;
        57,'AAz','rad/s^2','横摆角加速度 (代码未换算, 直接当 rad/s^2 用)',1;
        58,'Steer_L1','deg','左前轮转角',1;  59,'Steer_R1','deg','右前轮转角',1;
        60,'Steer_SW','deg','方向盘转角',1;  61,'VxTarget','km/h','目标车速',1;
        62,'LTR','-','横向载荷转移率(CarSim 自算)',1 };
in = [in; sng];
P.in = in;

% ---------------- 54 出 (pmpc_step 的 sys 向量) ----------------
out = cell(0,5);  % {通道号, 名字, 单位, 去向, 说明}
out = local_quad2(out, 1, 'Tb', 'N*m', 'CarSim IMP_MYBK_* (Add)',   '分配 QP 解出的制动力矩', C);
out = local_quad2(out, 5, 'Fd', 'N',   'CarSim IMP_FD_* (Replace)', 'MR 半主动悬架指令力', C);
sng2 = { 9,'Steer_Wheel','deg','CarSim IMP_STEER_SW (Replace)','AFS 方向盘转角(自动驾驶: 直接替换)';
        10,'eps1','-','仅记录','上层 QP 松弛变量';  11,'eps2','-','仅记录','上层 QP 松弛变量';
        12,'eps3','-','仅记录','上层 QP 松弛变量';  13,'eps4','-','仅记录','上层 QP 松弛变量';
        14,'rho_AFS','-','仅记录','优先级变量 (PMPC 模式才非零)';
        15,'rho_DB','-','仅记录','优先级变量';   16,'rho_CDC','-','仅记录','优先级变量';
        17,'MFxmax','N*m','仅记录','纵向力矩包络上界'; 18,'MFx_next','N*m','仅记录','下一拍纵向力矩';
        19,'MFxmin','N*m','仅记录','纵向力矩包络下界(= -MFxmax)';
        20,'Mdmax','N*m','仅记录','差动制动力矩上界';
        21,'Md_next','N*m','-> PID velocity control 的 DB_active','差动制动力矩指令';
        22,'Mdmin','N*m','仅记录','差动制动力矩下界';
        23,'Fyfmax','N','仅记录','前轴侧向力上界'; 24,'Fyf_next','N','仅记录','下一拍前轴侧向力';
        25,'Fyfmin','N','仅记录','前轴侧向力下界(= -Fyfmax)';
        26,'r_ssmax','rad/s','仅记录','稳态横摆角速度上界';
        27,'yawrate','rad/s','仅记录','当前横摆角速度';
        28,'r_ssmin','rad/s','仅记录','稳态横摆角速度下界(= -r_ssmax)';
        29,'LTR_real','-','仅记录','实测 LTR';  30,'LTR_Npdc','-','仅记录','预测域末端 LTR';
        31,'Vx','km/h','-> PID velocity control 的 Vx','当前纵向车速';
        32,'Vset_pid','km/h','-> PID velocity control 的 Set_Vx','纵向目标车速(滤波后)';
        33,'yr','m','仅记录','参考路径投影点的 Y';
        34,'Vtotal','m/s','仅记录','合成速度 sqrt(Vx^2+Vy^2)';
        35,'LTR','-','仅记录','LTR(与 29 同源)';
        36,'alpha_r_avg','deg','仅记录','后轴平均侧偏角';
        37,'Yaw','deg','仅记录','航向角';   38,'beta','rad','仅记录','质心侧偏角(弧度)';
        39,'CafTan','N/rad','仅记录','前轴在线切线刚度(负)';
        40,'CarTan','N/rad','仅记录','后轴在线切线刚度(负)';
        41,'Roll','rad','仅记录','侧倾角';  42,'Rollrate','rad/s','仅记录','侧倾角速度' };
out = [out; sng2];
out = local_quad2(out, 43, 'Fx', 'N', '仅记录', '轮胎纵向力(透传)', C);
out = local_quad2(out, 47, 'Fy', 'N', '仅记录', '轮胎侧向力(透传)', C);
out = local_quad2(out, 51, 'Fz', 'N', '仅记录', '轮胎垂向力(透传)', C);
P.out = out;

% ---------------- 11 路回灌被控对象 ----------------
P.plant = { 1,'IMP_MYBK_L1','Add',     'sys(1)  Tb_L1';
            2,'IMP_MYBK_L2','Add',     'sys(2)  Tb_L2';
            3,'IMP_MYBK_R1','Add',     'sys(3)  Tb_R1';
            4,'IMP_MYBK_R2','Add',     'sys(4)  Tb_R2';
            5,'IMP_FD_L1','Replace',   'sys(5)  Fd_L1';
            6,'IMP_FD_L2','Replace',   'sys(6)  Fd_L2';
            7,'IMP_FD_R1','Replace',   'sys(7)  Fd_R1';
            8,'IMP_FD_R2','Replace',   'sys(8)  Fd_R2';
            9,'IMP_STEER_SW','Replace','sys(9)  Steer_Wheel';
           10,'IMP_THROTTLE_ENGINE','Add','PID velocity control 出1';
           11,'IMP_PCON_BK','Add',        'PID velocity control 出2' };

% ---------------- HIL 子集: 50 入 / 20 出 ----------------
%  砍掉控制器从来没读过的 12 路(旧编号 5-16: CmpD_* 悬架压缩位移 /
%  Zgnd_* 地面高度 / Z_* 轮心高度), 剩 50 路, 重新编号成 1..50。
%
%  ⚠️ 历史: 2026-09-18 先是"CarSim 出 50 路, 模型里散布回 62 槽(5-16 填 0)",
%  为的是 func_StateEstimation 一个字不用改。同日晚些时候把那 52 处下标整体
%  重编成 1..50(脚本改 + 三个控制器 ERD 逐字节回归验证通过), 散布器全部删除。
%  现在**CarSim 通道号 = 控制器输入下标 = NI In 序号**, 三者完全一致。
raw  = P.in;
keep = true(62,1);  keep(5:16) = false;
idx  = find(keep);

P.dropped = raw(~keep, :);          % 12 路留档: {旧通道号, 名字, 单位, 用途, 0}
P.in = cell(numel(idx),5);          % {通道号(1..50), 名字, 单位, 用途, 是否被控制器使用}
for i = 1:numel(idx)
    P.in(i,:) = [ {i}, raw(idx(i), 2:5) ];
end

%  NI In 序号就是通道号 —— 留这个字段是为了 build_hil / 文档少改点
P.hil_in = cell(numel(idx),4);      % {NI In 序号, CarSim 通道号(=前者), 名字, 单位}
for i = 1:numel(idx)
    P.hil_in(i,:) = { i, i, P.in{i,2}, P.in{i,3} };
end

%  20 路输出: 9 路执行器(必须) + 9 路诊断 + 2 路纵向(来自 PID 子系统)
osel = [1 2 3 4 5 6 7 8 9 14 15 16 21 29 30 31 32 38];
P.hil_out = cell(20,5);             % {NI Out 序号, 来源, 名字, 单位, 说明}
for i = 1:numel(osel)
    k = osel(i);
    P.hil_out(i,:) = { i, sprintf('sys(%d)',k), P.out{k,2}, P.out{k,3}, P.out{k,5} };
end
P.hil_out(19,:) = { 19, 'PID vel 出1', 'Throttle',  '-',   '油门开度 -> CarSim IMP_THROTTLE_ENGINE' };
P.hil_out(20,:) = { 20, 'PID vel 出2', 'PconBk',    'MPa', '制动主缸压力 -> CarSim IMP_PCON_BK' };

assert(size(P.hil_in,1)==50,  'func_PortMap:HilIn',  'HIL 入口应为 50, 实际 %d', size(P.hil_in,1));
assert(size(P.hil_out,1)==20, 'func_PortMap:HilOut', 'HIL 出口应为 20, 实际 %d', size(P.hil_out,1));

assert(size(P.in,1)==50,  'func_PortMap:InCount',  '入口应为 50, 实际 %d', size(P.in,1));
assert(size(P.out,1)==54, 'func_PortMap:OutCount', '出口应为 54, 实际 %d', size(P.out,1));
assert(size(P.dropped,1)==12, 'func_PortMap:Dropped', '砍掉的应为 12 路, 实际 %d', size(P.dropped,1));
assert(all(cell2mat(P.in(:,5))==1), 'func_PortMap:Unused', ...
       '50 路里还有控制器没用到的 —— 说明裁剪没裁干净');

if strcmpi(mode,'-doc'), local_writedoc(P); end
end

% -------------------------------------------------------------------------
function t = local_quad(t, i0, base, unit, desc, used, C)
for k = 1:4
    t(end+1,:) = { i0+k-1, [base '_' C{k}], unit, desc, used }; %#ok<AGROW>
end
end

function t = local_quad2(t, i0, base, unit, dest, desc, C)
for k = 1:4
    t(end+1,:) = { i0+k-1, [base '_' C{k}], unit, dest, desc }; %#ok<AGROW>
end
end

function local_writedoc(P)
%  csv: VeriStand 映射直接导入用
root = func_ProjectRoot();
dataDir = fullfile(root, 'data');
docsDir = fullfile(root, 'docs');
f = fopen(fullfile(dataDir,'portmap_in.csv'),'w');
fprintf(f, 'Index,Name,Unit,Used,Description\n');
for i = 1:size(P.in,1)
    fprintf(f, '%d,%s,%s,%d,"%s"\n', P.in{i,1}, P.in{i,2}, P.in{i,3}, P.in{i,5}, P.in{i,4});
end
fclose(f);
f = fopen(fullfile(dataDir,'portmap_out.csv'),'w');
fprintf(f, 'Index,Name,Unit,Destination,Description\n');
for i = 1:size(P.out,1)
    fprintf(f, '%d,%s,%s,"%s","%s"\n', P.out{i,1}, P.out{i,2}, P.out{i,3}, P.out{i,4}, P.out{i,5});
end
fclose(f);

f = fopen(fullfile(docsDir,'PORTMAP_HIL.md'),'w','n','UTF-8');
fprintf(f, '# 控制器端口映射 (50 入 / 54 出)\n\n');
fprintf(f, '> 由 `func_PortMap(''-doc'')` 生成, 不要手改。\n');
fprintf(f, '> 来源: CarSim `Run_all.par` 的 EXPORT/IMPORT 列表 + `func_StateEstimation.m`\n');
fprintf(f, '> + `pmpc_step.m` 的 `sys` 向量 + 模型接线。\n\n');
fprintf(f, '## 拓扑\n\n```\n');
fprintf(f, '50 入 --> PMPC_MF --> 54 出 --+--> 前 9 路 -------------------+\n');
fprintf(f, '                              |                               +--> 11 路回灌被控对象\n');
fprintf(f, '                              +--> 31/32/21 --> PID vel --> 2 路 --+\n');
fprintf(f, '```\n\n');
fprintf(f, '⚠️ HIL 上要部署的不止 `PMPC_MF`, 还有 `PID velocity control` 子系统(纵向油门/制动)。\n');
fprintf(f, '它的 3 个输入全部取自 `PMPC_MF` 的输出, 所以是一条纯前馈链, 没有额外外部输入。\n\n');
fprintf(f, '## 输入 50 路 (CarSim EXPORT 顺序 = func_StateEstimation 的 ModelInput 下标)\n\n');
fprintf(f, '| # | 名称 | 单位 | 用到 | 说明 |\n|---|---|---|---|---|\n');
for i = 1:size(P.in,1)
    if P.in{i,5}, u = '是'; else, u = '**否**'; end
    fprintf(f, '| %d | `%s` | %s | %s | %s |\n', P.in{i,1}, P.in{i,2}, P.in{i,3}, u, P.in{i,4});
end
fprintf(f, '\n**这 50 路控制器全都用到**, 没有一路是白接的。\n\n');
fprintf(f, '### 历史: 砍掉的 12 路\n\n');
fprintf(f, '2026-09-18 之前 CarSim 导出 62 路, 下面这 12 路控制器从来没读过, 已从 Export\n');
fprintf(f, '列表里去掉; 其余 50 路顺序不变, 重新编号成 1..50。\n\n');
fprintf(f, '| 旧通道号 | 名称 | 单位 | 说明 |\n|---|---|---|---|\n');
for i = 1:size(P.dropped,1)
    fprintf(f, '| %d | `%s` | %s | %s |\n', P.dropped{i,1}, P.dropped{i,2}, P.dropped{i,3}, P.dropped{i,4});
end
fprintf(f, '\n将来要做路面预瞄的话, 把它们加回 Export 列表, 并在 func_StateEstimation 里\n');
fprintf(f, '**追加**到 51.. 之后的下标 —— 不要插回中间, 否则 50 路的编号全乱。\n\n');
fprintf(f, '## 输出 54 路\n\n');
fprintf(f, '| # | 名称 | 单位 | 去向 | 说明 |\n|---|---|---|---|---|\n');
for i = 1:size(P.out,1)
    fprintf(f, '| %d | `%s` | %s | %s | %s |\n', P.out{i,1}, P.out{i,2}, P.out{i,3}, P.out{i,4}, P.out{i,5});
end
fprintf(f, '\n真正驱动执行器的只有 **前 9 路**; 另有 3 路(21/31/32)喂给纵向 PID; 其余 42 路是记录量。\n');
fprintf(f, 'HIL 上若不需要记录, 这 42 路可以不接出去 —— 但建议留着, 出问题时没有它们很难查。\n\n');
fprintf(f, '## 回灌被控对象的 11 路\n\n| # | CarSim 通道 | 方式 | 来源 |\n|---|---|---|---|\n');
for i = 1:size(P.plant,1)
    fprintf(f, '| %d | `%s` | %s | %s |\n', P.plant{i,1}, P.plant{i,2}, P.plant{i,3}, P.plant{i,4});
end
fprintf(f, '\n`Replace` = 直接替换 CarSim 内部量(自动驾驶: 方向盘、MR 阻尼力);\n');
fprintf(f, '`Add` = 叠加到 CarSim 自身的量上(制动力矩、油门、制动压力)。\n\n');

% ---------------- HIL 子集 ----------------
fprintf(f, '---\n\n# HIL 模型端口 (50 入 / 20 出)\n\n');
fprintf(f, 'NI VeriStand 的约定是**一个信号一个块**(`NI In<k>` / `NI Out<k>` 是带掩码的\n');
fprintf(f, 'Inport/Outport, 块名即 VeriStand 通道名), 所以端口数量直接等于块数量。\n\n');
fprintf(f, '- 入: 50 路 NI In 直接并成控制器的输入向量\n');
fprintf(f, '- 出: `pmpc_step` 照样返回 54, 用 Selector 挑 20 路\n\n');
fprintf(f, '**NI In 序号 = CarSim 通道号 = ModelInput 下标**, 三者完全一致;\n');
fprintf(f, '桌面模型(cq3_2019 / pmpc_mil)和 HIL 模型共用同一份算法代码。\n\n');
fprintf(f, '## NI In 50 路\n\n| NI In | CarSim 通道号 | 名称 | 单位 |\n|---|---|---|---|\n');
for i = 1:size(P.hil_in,1)
    fprintf(f, '| %d | %d | `%s` | %s |\n', P.hil_in{i,1}, P.hil_in{i,2}, P.hil_in{i,3}, P.hil_in{i,4});
end
fprintf(f, '\n50 路全接, 没有空口。\n\n');
fprintf(f, '## NI Out 20 路\n\n| NI Out | 来源 | 名称 | 单位 | 说明 |\n|---|---|---|---|---|\n');
for i = 1:size(P.hil_out,1)
    fprintf(f, '| %d | %s | `%s` | %s | %s |\n', P.hil_out{i,1}, P.hil_out{i,2}, ...
            P.hil_out{i,3}, P.hil_out{i,4}, P.hil_out{i,5});
end
fprintf(f, '\n前 9 路是执行器指令(必须); 10-18 是诊断量, 挑的原则是"HIL 上出问题时要看什么";\n');
fprintf(f, '19-20 来自 `PID velocity control` 子系统, 不是 `pmpc_step` 的输出。\n\n');
fprintf(f, '> ⚠️ **QP 求解状态目前没有导出。** `exitflag` 只在 `pmpc_step` 内部用于统计,\n');
fprintf(f, '> 没进 `sys` 向量。实时台架上这是个盲区 —— 参考的 NI 例子里就专门有一路\n');
fprintf(f, '> `qp_status`。建议把它加成 `sys(55)`: 不影响任何计算, ERD 也不含控制器输出,\n');
fprintf(f, '> 所以 `baseline_mf` 的 MD5 不会变; 代价是改 `pmpc_step` 的输出宽度和 `Demux4`。\n');
fclose(f);

f = fopen(fullfile(dataDir,'portmap_hil_in.csv'),'w');
fprintf(f, 'NI_In,CarSimChannel,Name,Unit\n');
for i = 1:size(P.hil_in,1)
    fprintf(f, '%d,%d,%s,%s\n', P.hil_in{i,1}, P.hil_in{i,2}, P.hil_in{i,3}, P.hil_in{i,4});
end
fclose(f);
f = fopen(fullfile(dataDir,'portmap_hil_out.csv'),'w');
fprintf(f, 'NI_Out,Source,Name,Unit,Description\n');
for i = 1:size(P.hil_out,1)
    fprintf(f, '%d,%s,%s,%s,"%s"\n', P.hil_out{i,1}, P.hil_out{i,2}, P.hil_out{i,3}, ...
            P.hil_out{i,4}, P.hil_out{i,5});
end
fclose(f);
fprintf('  已写出 PORTMAP_HIL.md + 4 个 csv (50 入 / 54 出, NI 侧 50/20)\n');
end
