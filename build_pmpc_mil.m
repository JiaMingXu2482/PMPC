function build_pmpc_mil()
%BUILD_PMPC_MIL  以 pmpc_hil.slx 为底, 把 NI In/Out 换成 CarSim, 建 pmpc_mil.slx
%
%   为什么要这个模型:
%     cq3_2019(原 MIL) 和 pmpc_hil(HIL) 的**控制器部分是完全共享的** ——
%     同一个 PMPC_MF 块、同一份 pmpc_block.m / pmpc_step.m、同一个 PID 子系统。但两者在**时钟方式和求解器**上不一样:
%
%       cq3_2019 : PMPC_MF 挂 Function-Call 子系统, 由 fcgen 每 10 ms 触发
%                  求解器 FixedStepAuto -> ode3 (有连续状态)
%       pmpc_hil : PMPC_MF 自带 ChartUpdate=DISCRETE, SampleTime=0.01
%                  求解器 ode1
%
%     所以 cq3_2019 跑通**不能**保证 pmpc_hil 那套架构在闭环里也对。
%     pmpc_mil 就是拿 pmpc_hil 的架构接上 CarSim, 上 NI 之前先在闭环里验一遍。
%
%   顶层拓扑(主干全落在 y=350 一条直线上):
%
%     PMPC_MF -54-> pick18 -18-> split18 -+-9-> pack11 -11-> CarSim
%        ^                                |                     |
%        |              10..18 -> 9 个 Terminator               |
%        |                                |                     |
%        |   split_pid <- PID velocity control <-+ (16,17,13)   |
%        |        |                                             |
%        |        +-> pack11 的 10/11                           |
%        +----------- 50 路回灌(绕到图下方) <--------------------+
%
%   ⚠️ 2026-09-18 之前这里还有一个 "Scatter 50 to 62" 子系统: CarSim 出 50 路而
%   控制器吃 62 路, 中间要把 50 路散布回 62 槽(5-16 填 0)。后来把
%   func_StateEstimation 的下标整体重编成 1..50(脚本改 + 逐字节回归验过),
%   CarSim 的 50 路就直接进 PMPC_MF 了, 散布器整个删掉。
%
%   与 cq3_2019 的分工: cq3_2019 仍是算法开发和论文数据的基准(所有 baseline 出自它);
%   pmpc_mil 只用来验证 HIL 架构。两个都留。

SRC = 'pmpc_hil';  MDL = 'pmpc_mil';

local_carsimlib();              % CarSim 的 Solver_SF 库路径(点过 Send to Simulink 的
                                % 会话里本来就有; 独立起的 MATLAB 里没有)
bdclose('all');
if exist([MDL '.slx'],'file'), delete([MDL '.slx']); end
load_system(SRC);
save_system(SRC, MDL);          % 内存里的模型改名成 pmpc_mil, 磁盘上 pmpc_hil.slx 不动
load_system('cq3_2019');        % 为了把 CarSim 块复制过来

% ---------- 1. 删掉所有 NI 端口 ----------
for bt = {'Inport','Outport'}
    b = find_system(MDL,'SearchDepth',1,'IncludeCommented','on','BlockType',bt{1});
    for i = 1:numel(b), local_kill(b{i}); end
end

% ---------- 2. 去掉 pmpc_hil 那个把 50 个标量并起来的 Mux ----------
%  pmpc_hil 里 50 个 NI In 要先并成一个向量才能进 PMPC_MF; 这里 CarSim 本来就
%  给一根 50 宽的信号, 直接接就行。
local_kill([MDL '/pack50']);

% ---------- 3. 顶层: 被控对象 + 执行器打包 ----------
add_block('cq3_2019/CarSim S-Function1', [MDL '/CarSim'], 'Position',[970 230 1090 330]);
add_block('simulink/Signal Routing/Mux', [MDL '/pack11'], 'Inputs','11','Position',[890 170 905 390]);

%  控制器主干摆成一条直线, 端口都落在 y=350
set_param([MDL '/PMPC_MF'], 'Position',[200 290 340 410]);
set_param([MDL '/pick18'],  'Position',[400 320 480 380]);
set_param([MDL '/split18'], 'Position',[540 100 545 300]);
func_PortAlign([MDL '/split18'], 'out', 180, 20);    % 18 路行距 20 -> 180..520, 块中心正好 350
func_PortAlign([MDL '/pack11'],  'in',  180, 20);    % 1..9 与 split18 的 1..9 等高 -> 直线
set_param([MDL '/PID velocity control'], 'Position',[650 600 780 700]);
set_param([MDL '/split_pid'], 'Position',[830 620 835 680]);
local_center([MDL '/CarSim'], local_porty([MDL '/pack11'],'out'));

%  10..18 是只记录不回灌的诊断量, CarSim 的 ERD 里已经有。Demux 出口不能悬空,
%  所以必须挂 Terminator。
yt = local_porty([MDL '/split18'],'out');
for i = 10:18
    add_block('simulink/Sinks/Terminator', [MDL '/' sprintf('term%d',i)], ...
              'Position',[610 yt(i)-8 630 yt(i)+8], 'ShowName','off');  % 名字挤成一团, 不显示
end

% ---------- 4. 顶层接线 ----------
%  ⚠️ 下面这几条**不要**再连, 它们是从 pmpc_hil 继承来的, 删 NI 端口时没被动到:
%       PMPC_MF -> pick18 -> split18 -> (13/16/17) PID -> split_pid
%     重复连会报"目标端口已有信号连接"。
for i = 1:9
    add_line(MDL, sprintf('split18/%d',i), sprintf('pack11/%d',i), 'autorouting','off');
end
for i = 10:18
    add_line(MDL, sprintf('split18/%d',i), sprintf('term%d/1',i),  'autorouting','off');
end
%  (PID 的三个输入 sys31 Vx / sys32 Vset_pid / sys21 Md_next —— 在 pick18 挑出的
%   18 路里排第 16 / 17 / 13 —— 连线继承自 pmpc_hil, 这里不用重连)
%  但**布线**要重来: 这三根是分支(同一个出口还挂着 Terminator), 自动布线会把它们
%  甩到画布下方一千多像素的地方去。手工给拐点, 各走一条竖直通道下来。
%  通道 x 的顺序 590 > 575 > 560 是排过的: 这么放三根之间互不相交。
PID  = [MDL '/PID velocity control'];
ysp  = local_porty([MDL '/split18'],'out');
lane = [590 575 560];        % PID 入 1 / 2 / 3 各自的竖直通道
psrc = [ 16  17  13];        % PID 入 1 / 2 / 3 分别来自 split18 的第几路
for k = 1:3
    lp = get_param(PID,'PortHandles');
    delete_line(get_param(lp.Inport(k),'Line'));    % 只删这一支, 去 Terminator 的那支还在
    dst = sprintf('PID velocity control/%d', k);
    h   = add_line(MDL, sprintf('split18/%d',psrc(k)), dst, 'autorouting','smart');
    p   = get_param(h,'Points');
    if min(p(:,1)) < 520 || max(p(:,2)) > 760       % smart 又绕出去了, 退回手工拐点
        delete_line(h);
        h  = add_line(MDL, sprintf('split18/%d',psrc(k)), dst, 'autorouting','off');
        lp = get_param(PID,'PortHandles');
        qd = get_param(lp.Inport(k),'Position');
        set_param(h, 'Points', [lane(k) ysp(psrc(k)); lane(k) qd(2); qd(1) qd(2)]);
    end
end

h10 = add_line(MDL, 'split_pid/1','pack11/10','autorouting','off');
h11 = add_line(MDL, 'split_pid/2','pack11/11','autorouting','off');
local_elbow(h10, local_portxy([MDL '/split_pid'],'out',1), local_portxy([MDL '/pack11'],'in',10), 850);
local_elbow(h11, local_portxy([MDL '/split_pid'],'out',2), local_portxy([MDL '/pack11'],'in',11), 865);
add_line(MDL, 'pack11/1',   'CarSim/1', 'autorouting','off');

%  50 路回灌: 从右边绕到图的下方再回左边, 手工给拐点, 不让自动布线乱画
hl = add_line(MDL, 'CarSim/1', 'PMPC_MF/1', 'autorouting','on');
q1 = local_portxy([MDL '/CarSim'],  'out', 1);
q2 = local_portxy([MDL '/PMPC_MF'], 'in',  1);
yb = 780;
set_param(hl, 'Points', [q1; q1(1)+45 q1(2); q1(1)+45 yb; q2(1)-50 yb; q2(1)-50 q2(2); q2]);

% ---------- 5. 观感 ----------
set_param([MDL '/CarSim'],  'BackgroundColor','lightBlue');
set_param([MDL '/PMPC_MF'], 'BackgroundColor','orange');
set_param([MDL '/PID velocity control'],'BackgroundColor','lightBlue');
set_param(MDL, 'ShowLineDimensions','on');   % 线上标信号宽度, 一眼看出 50/54/18/11
local_note(MDL, 'hdr',  [ 60  40], 12, ...
    ['pmpc_mil —— 用 pmpc_hil 的架构接 CarSim 跑闭环' char(10) ...
     'PMPC_MF: ChartUpdate=DISCRETE, SampleTime=0.01;  求解器 ode1 / 0.001' char(10) ...
     '与 pmpc_hil 唯一的区别是被控对象: 这里是 CarSim, 那边是 50 个 NI In + 20 个 NI Out']);
local_note(MDL, 'noteT',[600 555], 10, ['10..18 路是诊断量' char(10) 'ERD 里已有, 不回灌']);
local_note(MDL, 'noteF',[120 800], 10, 'CarSim 的 50 路量测直接回灌控制器(顺序 = CarSim Export 列表)');

% ---------- 6. 求解器: 与 pmpc_hil 一致 ----------
set_param(MDL, 'SolverType','Fixed-step', 'Solver','ode1', ...
               'FixedStep','0.001', 'StartTime','0', 'StopTime','10');

save_system(MDL);
fprintf('  已建 %s.slx (以 pmpc_hil 为底, CarSim 接入)\n', MDL);
fprintf('  求解器 %s / step %s, PMPC_MF SampleTime 0.01\n', ...
        get_param(MDL,'Solver'), get_param(MDL,'FixedStep'));
local_report(MDL);
end

% =========================================================================
% 布局小工具
% =========================================================================

function local_elbow(h, q1, q2, lane)
%LOCAL_ELBOW  把一根线画成"横 - 竖 - 横", 竖段落在 x=lane
set_param(h, 'Points', [q1; lane q1(2); lane q2(2); q2]);
end

function local_center(blk, y)
p = get_param(blk,'Position');
h = p(4) - p(2);
t = round(y - h/2);
set_param(blk,'Position',[p(1) t p(3) t+h]);
end

function y = local_porty(blk, side)
ph = get_param(blk,'PortHandles');
if strcmp(side,'in'), hs = ph.Inport; else, hs = ph.Outport; end
y = zeros(numel(hs),1);
for k = 1:numel(hs)
    q = get_param(hs(k),'Position');
    y(k) = q(2);
end
end

function q = local_portxy(blk, side, k)
ph = get_param(blk,'PortHandles');
if strcmp(side,'in'), hs = ph.Inport; else, hs = ph.Outport; end
q = get_param(hs(k),'Position');
q = q(:).';
end

function local_note(mdl, tag, xy, fs, txt)
%  注解纯属好看, 各版本 Position 的接受形式不太一样, 失败了不该拖垮建模型
try
    a = Simulink.Annotation([mdl '/' tag]);
    a.Text                = txt;
    a.FontSize            = fs;
    a.HorizontalAlignment = 'left';
    a.Position            = [xy(1) xy(2) xy(1)+520 xy(2)+60];
catch ME
    fprintf(2, '  (注解 %s 没加上: %s)\n', tag, ME.message);
end
end

function local_carsimlib()
%LOCAL_CARSIMLIB  把 CarSim 的 Solver_SF 库挂上路径
%
%   CarSim 点 Send to Simulink 时会顺手把这个目录加进 MATLAB 路径, 所以在平时
%   用的那个会话里 build 不会有事。但用 matlab -batch 另起一个干净会话时没有,
%   add_block 会报"找不到名为 'Solver_SF' 的库", 然后默默塞一个未解析的链接块。
%   路径从 simfile.sim 的 PROGDIR 推 —— 不写死机器路径。
if ~isempty(which('Solver_SF')), return; end
progdir = '';
if exist('simfile.sim','file') == 2
    txt = fileread('simfile.sim');
    tk  = regexp(txt, '(?m)^PROGDIR\s+(.+?)\s*$', 'tokens', 'once');
    if ~isempty(tk), progdir = strtrim(tk{1}); end
end
if isempty(progdir), return; end
for sub = {fullfile('Programs','solvers','Matlab84+'), fullfile('Programs','solvers')}
    p = fullfile(progdir, sub{1});
    if exist(p,'dir') == 7, addpath(p); end
end
end

function local_kill(blk)
%LOCAL_KILL  先断干净这个块自己的连线再删块 —— delete_block 单独用不保险
%
%   ⚠️ 只删"挂在本块端口上的那一段", **绝对不能**顺着 LineParent 往上找主干再删。
%   pmpc_hil 里 split18 的 13/16/17 路是**一分为二**的: 一支去 Out 端口, 一支去
%   PID velocity control。删 Out 端口时如果删的是主干, 会把去 PID 的那一支一起带走,
%   模型编译时才报"PID 的输入没接" —— 第一版就是这么把 PID 弄断的。
%   目的端口上拿到的是分支段, 删它只断这一支; 源端口上拿到的是主干, 删它会连所有
%   分支一起没 —— 但源端口的宿主块本来就要删掉, 正是想要的。
ph = get_param(blk,'PortHandles');
hs = [ph.Inport, ph.Outport, ph.Enable, ph.Trigger, ph.State, ph.LConn, ph.RConn, ph.Ifaction];
for k = 1:numel(hs)
    for guard = 1:500
        l = get_param(hs(k),'Line');
        if l <= 0, break; end
        delete_line(l);
    end
end
delete_block(blk);
end

function local_report(mdl)
%  自检: 端口对齐差了几个像素(0 = 全是水平直线)
ys = local_porty([mdl '/split18'],'out');
yp = local_porty([mdl '/pack11'], 'in');
fprintf('  端口对齐残差(px): split18->pack11 %g\n', max(abs(ys(1:9) - yp(1:9))));
fprintf('  顶层块 %d 个\n', numel(find_system(mdl,'SearchDepth',1,'Type','block')));
end
