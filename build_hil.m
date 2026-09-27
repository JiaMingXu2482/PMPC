function build_hil(doExport)
%BUILD_HIL  建 HIL 部署模型 pmpc_hil (阶段 E1)
%   build_hil()      建模型并存盘
%   build_hil(1)     额外导出 pmpc_hil_R2018a.slx (给部署机的 MATLAB R2018a)
%
%   结构:
%       In_* x50 (标量 Inport) --> Mux(50) --> PMPC_MF --> Selector(18)
%                                                              |
%                                                              v
%                                                         Demux(18)
%                                                           |      |
%                                              Out_* x18 <--+      +--> PID vel --> Out_* x2
%
%   为什么输入是 50 个**独立标量端口**而不是一根 50 的总线:
%     NI VeriStand 的约定是一个信号一个块(NI In<k> 是带掩码的 Inport, 块名即通道名),
%     所以在 2018a 上要把每个 Inport 逐个换成 NI In。拆开才换得动。
%
%   ⚠️ 2026-09-18 之前这里是 Mux(62) + 一个常数 0: CarSim/NI 只给 50 路而控制器
%     吃 62 路, 中间要散布。后来把 func_StateEstimation 的 52 处下标整体重编成
%     1..50(脚本改 + 三个控制器 ERD 逐字节回归验证通过), 散布器删掉, 现在是
%     **NI In 序号 = CarSim 通道号 = ModelInput 下标**, 一路直通。
%     远程机上那份已经换好 NI In 的 pmpc_hil.slx 不用重新生成 —— 跑一次 hil50 即可。
%
%   端口名为什么带 In_/Out_ 前缀:
%     Simulink 要求同层块名唯一, 而 Fd_L1..R2 (测量值 vs 指令值) 和 Vx 都重名。
%     加前缀是强制的, 顺带也更清楚。
%
%   ⚠️ 前置: 必须先跑 setup_pmpc —— PMPC_MF 的参数 PMPC_P 是 Scope=Parameter,
%      编译时要从 base 工作区解析。
if nargin < 1, doExport = 0; end

MDL = 'pmpc_hil';
SRC = 'cq3_2019';
PM  = func_PortMap();

if bdIsLoaded(MDL), close_system(MDL, 0); end
if exist([MDL '.slx'],'file'), delete([MDL '.slx']); end
load_system(SRC);
new_system(MDL);
load_system(MDL);

nIn  = size(PM.hil_in,1);      % 50
nOut = size(PM.hil_out,1);     % 20
osel = [1 2 3 4 5 6 7 8 9 14 15 16 21 29 30 31 32 38];   % 取出来接 NI Out 的 sys 下标
nSel = numel(osel);            % 18 (另外 2 路来自 PID)

% ---------------- 50 个标量 Inport ----------------
X0 = 40;  DY = 45;
for i = 1:nIn
    nm = ['In_' PM.hil_in{i,3}];
    b  = [MDL '/' nm];
    add_block('simulink/Sources/In1', b, 'Position', [X0, 40+(i-1)*DY, X0+70, 40+(i-1)*DY+24]);
    set_param(b, 'Port', num2str(i));
end

% ---------------- Mux(50): 50 个标量并成控制器的输入向量 ----------------
%  ⚠️ 2026-09-18 之前这里是 Mux(62) + 一个常数 0 驱动的 5-16 槽:
%     CarSim 砍成 50 路导出时, 为了不动算法代码, 在模型层把 50 路散布回 62 槽。
%     后来把 func_StateEstimation 的下标整体重编成 1..50(脚本改 + 逐字节回归验过),
%     散布器就没用了 —— 现在是干干净净的 50 进 50。
MUX = [MDL '/pack50'];
add_block('simulink/Signal Routing/Mux', MUX, 'Inputs', num2str(nIn), ...
          'Position', [X0+220, 40, X0+235, 40+nIn*DY]);
%  Inport 的出口在 40+(i-1)*DY+12, Mux 入口排成一样的高度 -> 50 根线全是水平直线
func_PortAlign(MUX, 'in', 52, DY);
for i = 1:nIn
    add_line(MDL, ['In_' PM.hil_in{i,3} '/1'], sprintf('pack50/%d', i), 'autorouting','off');
end

% ---------------- 控制器 ----------------
add_block([SRC '/Subsystem1/PMPC_MF'], [MDL '/PMPC_MF'], ...
          'Position', [X0+340, 400, X0+460, 480]);
%  ⚠️ 必须显式给 10 ms, 不能让它继承。
%  原模型里它挂在 Function-Call 子系统下, 由 Function-Call Generator 按 10 ms 触发;
%  这里没有触发器, 不设的话它会继承基频(甚至连续时间), 而带 persistent 的
%  MATLAB Function 块在连续采样时间下直接报
%  Stateflow:Runtime:IllegalPersistentVarInContinuousTimeChart。
%  MATLAB Function 块是个 SubSystem, 没有 SampleTime 参数 —— 要设在里面
%  那个 Stateflow chart 对象上 (ChartUpdate + SampleTime)。
rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart','Path',[MDL '/PMPC_MF']);
assert(~isempty(ch), 'build_hil:NoChart', '没找到 PMPC_MF 的 chart 对象');
ch.ChartUpdate = 'DISCRETE';
ch.SampleTime  = '0.01';
%  ⚠️ R2018a 兼容: 必须把输出向量按**一维**解释。
%  默认 VectorOutputs1D=0 (输出是 54x1 二维列向量), 导出到 R2021b 之前的版本时
%  Simulink 会警告"此模块将失去功能" —— 老版本不支持禁用这个属性。
%  设成 1 之后输出是 54 元素一维向量, 下游 Selector 照常工作。
%  只改 HIL 模型这一份; 桌面模型 cq3_2019 的 PMPC_MF 不动(它已逐字节验收过)。
ch.VectorOutputs1D = 1;
add_line(MDL, 'pack50/1', 'PMPC_MF/1', 'autorouting','on');

% ---------------- 从 54 里挑 18 路 ----------------
SEL = [MDL '/pick18'];
add_block('simulink/Signal Routing/Selector', SEL, ...
          'Position', [X0+520, 410, X0+570, 470]);
set_param(SEL, 'NumberOfDimensions','1', 'IndexMode','One-based', ...
          'IndexOptionArray', {'Index vector (dialog)'}, ...
          'IndexParamArray', {mat2str(osel)}, ...
          'InputPortWidth', '54');
add_line(MDL, 'PMPC_MF/1', 'pick18/1', 'autorouting','on');

DMX = [MDL '/split18'];
add_block('simulink/Signal Routing/Demux', DMX, 'Outputs', num2str(nSel), ...
          'Position', [X0+630, 400, X0+645, 400+nSel*DY]);
add_line(MDL, 'pick18/1', 'split18/1', 'autorouting','on');

% ---------------- 18 个标量 Outport ----------------
XO = X0 + 780;
for i = 1:nSel
    nm = ['Out_' PM.hil_out{i,3}];
    b  = [MDL '/' nm];
    add_block('simulink/Sinks/Out1', b, 'Position', [XO, 400+(i-1)*DY, XO+70, 400+(i-1)*DY+24]);
    set_param(b, 'Port', num2str(i));
    add_line(MDL, sprintf('split18/%d', i), [nm '/1'], 'autorouting','on');
end

% ---------------- 纵向 PID -> 另外 2 路 ----------------
%  PID 的 3 个输入全部取自控制器自己的输出, 在 18 路里的位置:
%    Vx        = sys(31) -> 第 16 路
%    Set_Vx    = sys(32) -> 第 17 路
%    DB_active = sys(21) -> 第 13 路
PID = [MDL '/PID velocity control'];
add_block([SRC '/PID velocity control'], PID, 'Position', [XO-120, 400+nSel*DY+60, XO-20, 400+nSel*DY+160]);
pidSrc = [16 17 13];
for k = 1:3
    add_line(MDL, sprintf('split18/%d', pidSrc(k)), sprintf('PID velocity control/%d', k), 'autorouting','on');
end
DM2 = [MDL '/split_pid'];
add_block('simulink/Signal Routing/Demux', DM2, 'Outputs','2', ...
          'Position', [XO+40, 400+nSel*DY+80, XO+55, 400+nSel*DY+140]);
add_line(MDL, 'PID velocity control/1', 'split_pid/1', 'autorouting','on');
for k = 1:2
    nm = ['Out_' PM.hil_out{nSel+k,3}];
    b  = [MDL '/' nm];
    add_block('simulink/Sinks/Out1', b, 'Position', ...
              [XO+140, 400+nSel*DY+70+(k-1)*DY, XO+210, 400+nSel*DY+94+(k-1)*DY]);
    set_param(b, 'Port', num2str(nSel+k));
    add_line(MDL, sprintf('split_pid/%d', k), [nm '/1'], 'autorouting','on');
end

% ---------------- 求解器与代码生成配置 ----------------
%  基频 0.001 = VeriStand 的 1000 Hz; 控制器 PMPC_MF 单独设 0.01 = 10 ms 控制周期。
%  这和桌面模型的结构一致(CarSim EXT_MODEL_STEP 0.001 + Function-Call 每 10 ms 触发)。
%
%  ⚠️ 求解器用 ode1 而不是 FixedStepDiscrete, 因为 "PID velocity control" 里的
%     Int_I 是**连续** Integrator, 有连续状态, FixedStepDiscrete 直接拒绝:
%        Simulink:Engine:InvalidSolver
%     ode1 是定步长显式欧拉, 可以生成代码也能上实时目标, 且行为与桌面模型
%     (FixedStepAuto -> ode3 @1ms) 最接近 —— 换离散积分器会改变纵向 PID 的行为。
%     若要对齐 NI 例子的 FixedStepDiscrete, 把 Int_I 换成 Discrete-Time Integrator
%     (Forward Euler, T=0.001), 但那是一处**会改变数值**的修改, 需要单独验收。
set_param(MDL, 'SolverType','Fixed-step', 'Solver','ode1', ...
               'FixedStep','0.001', 'StartTime','0', 'StopTime','10');
set_param(MDL, 'TargetLang','C++');
%  ⚠️ MAT-file logging 必须关。它是 Simulink 的默认值(on), 而 TargetLang=C++ 对应的
%     Code interface packaging 是 "C++ class" —— 属于**可重入代码**, 两者不兼容:
%        Error: The option 'MAT-file logging' is not compatible with reusable code;
%        consider deselecting 'MAT-file logging', setting the option
%        'Code interface packaging' to 'Nonreusable function', or setting
%        'Multi-instance code error diagnostic' to 'None' or 'Warning'
%     NI VeriStand 目标同样要求可重入(一个模型可以多实例加载), 所以无论 grt 还是
%     NIVeriStand.tlc 都得关掉它。而且往实时目标上写 .mat 本来就没意义。
%     三个建议里只有"关 MAT-file logging"是对的 —— 另两个是把接口改成不可重入
%     或者把诊断降级, 都会让 NI 那边加载不了。
set_param(MDL, 'MatFileLogging','off');
%  NI 目标只在部署机上有, 本机设不了(本机停留在 grt.tlc)。
%  **部署机上必须先切目标再 build**, 否则生成的是普通 grt 产物, VeriStand 加载不了:
%    set_param(MDL,'SystemTargetFile','NIVeriStand.tlc');   % 先切这个
%    set_param(MDL,'TemplateMakefile','NIVeriStand_vc.tmf');
%    set_param(MDL,'MatFileLogging','off');                 % 切目标会重置一批选项, 再关一次
%  切过之后生成目录是 <模型名>_niVeriStand_rtw, 产物是 .dll;
%  如果看到 <模型名>_grt_rtw, 就是目标没切成。

save_system(MDL);
fprintf('  已建 %s: %d 入 / %d 出\n', MDL, nIn, nOut);

if doExport
    f = [MDL '_R2018a.slx'];
    if exist(f,'file'), delete(f); end
    Simulink.exportToVersion(MDL, f, 'R2018A');
    fprintf('  已导出 %s (给部署机的 R2018a)\n', f);
end
end
