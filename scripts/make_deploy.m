function make_deploy()
%MAKE_DEPLOY  打包部署机(R2018a + NI VeriStand)需要的全部文件到 HIL_deploy\
%
%   跑之前必须先 setup_pmpc(PMPC_MF 的参数要从工作区解析)和 build_hil。
%   本函数会自己调 build_hil, 所以直接:
%       setup_pmpc; make_deploy
%
%   包里放什么:
%     1. 运行期 .m —— 用 matlab.codetools.requiredFilesAndProducts 从
%        setup_pmpc / pmpc_block 两个入口做依赖分析, 不手工列
%     2. 数据 .mat —— 依赖分析找不到 load 的数据文件, 手工列 + 扫描校验
%     3. pmpc_hil.slx —— 导出成 R2018a 格式。**文件名必须等于模型名**,
%        否则 Simulink 打开时名字对不上
%     4. 端口表 + hil_init.m + README
%
%   不放什么: 所有桌面侧工具(chk_regress / run3b / run_ds / build_* /
%   func_Metrics / func_ReadERD / plot_week / cg_* ...), 部署机用不着。

root = func_ProjectRoot();
D = fullfile(root, 'HIL_deploy');
%  清空而不是删目录 —— OneDrive/资源管理器会占住目录句柄, rmdir 直接失败,
%  但删里面的文件是可以的。
if exist(D,'dir')
    old = dir(fullfile(D,'*'));
    for i = 1:numel(old)
        if ~old(i).isdir, delete(fullfile(D, old(i).name)); end
    end
else
    mkdir(D);
end

% ---------- 1. 依赖分析 ----------
fprintf('  依赖分析中...\n');
flist = matlab.codetools.requiredFilesAndProducts( ...
    {fullfile(root,'setup_pmpc.m'), fullfile(root,'controller','pmpc_block.m')});
flist = flist(:);
n = 0;
for i = 1:numel(flist)
    [~,nm,ext] = fileparts(flist{i});
    if ~strcmpi(ext,'.m'), continue; end
    copyfile(flist{i}, fullfile(D,[nm ext]));
    n = n + 1;
end
fprintf('  运行期 .m : %d 个\n', n);

% ---------- 2. 数据文件 ----------
%  DLC.mat 已不在清单里: func_WayPoints 原来从它里面取 cfit 对象求参考路径,
%  而 cfit 是 Curve Fitting Toolbox 的类, R2024b 存的在 R2018a 上加载不了。
%  现已换成等价的显式 tanh 公式(逐位相同), 不再需要这个文件。
mats = {'TireCarpet_265_75R16.mat', 'ZengSaddleDB_SUV.mat'};
%  校验: 扫一遍被打包的 .m, 看有没有漏掉的 load。
%  注意不要剥注释 —— MATLAB 的 '%d' 之类格式串里带 %, 按注释剥会把字符串吃掉
%  (第一版就是这么漏了文件)。宁可多报不可少报。
d = dir(fullfile(D,'*.m'));
found = {};
for i = 1:numel(d)
    t = local_read(fullfile(D, d(i).name));
    tok = regexp(t, '(?:coder\.)?load\s*\(\s*[''"]([^''"]+)[''"]', 'tokens');
    for k = 1:numel(tok)
        f = tok{k}{1};
        if isempty(regexp(f,'\.mat$','once')), f = [f '.mat']; end %#ok<AGROW>
        found{end+1} = f; %#ok<AGROW>
    end
end
found = unique(found);
miss = setdiff(found, mats);
if ~isempty(miss)
    warning('make_deploy:ExtraMat', '扫描发现清单外的数据文件: %s', strjoin(miss,', '));
    mats = [mats, miss(:).'];
end
for i = 1:numel(mats)
    src = fullfile(root, 'data', mats{i});
    if exist(src,'file') ~= 2
        error('make_deploy:NoMat', '找不到数据文件 %s', mats{i});
    end
    copyfile(src, fullfile(D, mats{i}));
end
fprintf('  数据 .mat : %d 个 (%s)\n', numel(mats), strjoin(mats,', '));

% ---------- 3. 模型 ----------
build_hil();                       % 重建, 保证包里的是最新的
%  文件名必须等于模型名 pmpc_hil, 不能叫 pmpc_hil_R2018a.slx
Simulink.exportToVersion('pmpc_hil', fullfile(D,'pmpc_hil.slx'), 'R2018A');
fprintf('  模型      : pmpc_hil.slx (R2018a 格式)\n');

%  给远程机就地改造用: 那边的 pmpc_hil.slx 已经手工换好 50 个 NI In,
%  重新生成要再换 50 次, 所以带上这个脚本, 跑一次就把 Mux(62) 换成 Mux(50)。
copyfile(fullfile(root,'scripts','hil50.m'), fullfile(D,'hil50.m'));

% ---------- 4. 端口表 ----------
func_PortMap('-doc');
copyfile(fullfile(root,'docs','PORTMAP_HIL.md'), fullfile(D,'PORTMAP_HIL.md'));
copyfile(fullfile(root,'docs','HIL_R2018a_codegen_notes.md'), fullfile(D,'HIL_R2018a_codegen_notes.md'));
copyfile(fullfile(root,'data','portmap_hil_in.csv'), fullfile(D,'portmap_hil_in.csv'));
copyfile(fullfile(root,'data','portmap_hil_out.csv'), fullfile(D,'portmap_hil_out.csv'));

% ---------- 5. hil_init + README ----------
local_write_init(D);
local_write_readme(D);

% ---------- 6. 剥掉注释, 换成英文头 ----------
%  部署包里的注释一律英文: 注释全 ASCII 就根本不存在编码问题(见下一步),
%  远程机上看代码也不会乱码。中文的设计说明留在开发目录里, 不进包。
d = dir(fullfile(D,'*.m'));
for i = 1:numel(d)
    local_englishify(fullfile(D, d(i).name));
end
fprintf('  注释      : %d 个 .m 已剥离中文注释并换成英文头\n', numel(d));

% ---------- 7. 全部 .m 转成 GBK ----------
%  ⚠️ 这一步不能省。部署机是**中文 Windows 上的 R2018a**:
%    MATLAB 从 R2020a 起才默认按 UTF-8 读 .m, 之前一律按系统 ANSI 代码页
%    (中文 Windows = GBK/CP936)。于是:
%      - UTF-8 的中文注释 -> 乱码
%      - 带 UTF-8 BOM 的文件 -> 直接报错, 且报在**行 1 列 1**:
%          "文本字符无效。请检查不受支持的符号、不可见的字符或非 ASCII 字符的粘贴。"
%    (实测 func_VehicleParams.m 就是这样挂的。)
d = dir(fullfile(D,'*.m'));
nconv = 0;
for i = 1:numel(d)
    nconv = nconv + func_ToGbk(fullfile(D, d(i).name));
end
fprintf('  编码      : %d/%d 个 .m 从 UTF-8 转成 GBK(并去 BOM)\n', nconv, numel(d));

d = dir(fullfile(D,'*'));
d = d(~[d.isdir]);
fprintf('  >>> %s: %d 个文件, %.0f KB <<<\n', D, numel(d), sum([d.bytes])/1024);
end

% -------------------------------------------------------------------------
function local_englishify(p)
%LOCAL_ENGLISHIFY  剥掉一个 .m 的全部注释, 换成生成的英文头
[~, nm] = fileparts(p);
f = fopen(p,'r'); b = fread(f, inf, '*uint8').'; fclose(f);
if numel(b) >= 3 && isequal(b(1:3), uint8([239 187 191])), b = b(4:end); end
t = native2unicode(b, 'UTF-8');
if ~isequal(uint8(unicode2native(t,'UTF-8')), b)
    t = native2unicode(b, 'GBK');          % 本来就是 GBK 的那几个
end

code = local_strip_comments(t);
hdr  = local_header(nm);
%  头注释放在 function 声明行**之后** —— MATLAB 的 help 只认紧跟声明的那一段,
%  放前面 `help pmpc_step` 就看不到。
L = regexp(code, '\n', 'split');
%  ⚠️ 不能用 \b 做单词边界 —— MATLAB 的 regexp 里 \b 是**退格符**,
%  不是单词边界(MATLAB 用 \< \>)。写成 \b 会永远匹配不上, 而且一声不吭。
fi = find(~cellfun(@isempty, regexp(L, '^\s*function\s', 'once')), 1);
if isempty(fi)
    out = [hdr sprintf('\n') code sprintf('\n')];
else
    %  ⚠️ 函数声明可能跨行(func_QPA_CDC / func_SolveMPCQP 都是 '...' 续行),
    %  头注释必须插在**整个声明之后**。插进续行中间会让 Simulink 报
    %  "无法确定模块 PMPC_MF 的输出大小和/或类型" —— 看着像 Tunable 的老问题,
    %  其实是函数签名被劈开了。
    fe = fi;
    while fe < numel(L) && ~isempty(regexp(strtrim(L{fe}), '\.\.\.$', 'once'))
        fe = fe + 1;
    end
    out = strjoin([L(1:fe), {hdr}, L(fe+1:end)], sprintf('\n'));
    out = [out sprintf('\n')];
end

%  剥完之后, **字符串字面量之外**不许再有非 ASCII —— 那说明剥漏了注释。
%  字符串里的中文是代码的一部分(错误消息之类), 不改, 交给后面的 GBK 转换。
bad = local_nonascii_outside_string(code);
if bad > 0
    error('make_deploy:NonAscii', ...
          '%s 剥掉注释后在字符串之外还有非 ASCII 字符(第 %d 个字符处), 说明有注释没剥干净', ...
          p, bad);
end
%  ⚠️ 必须走 unicode2native 编码后再写。直接 fwrite(f, uint8(out)) 会把
%  码点 > 255 的字符(字符串字面量里的中文)截断成单字节垃圾(实测写出 0xFF),
%  而且不报任何错 —— 下一步的 GBK 转换会因为解不开而炸掉。
f = fopen(p,'w'); fwrite(f, unicode2native(out, 'UTF-8')); fclose(f);
end

function out = local_strip_comments(t)
%LOCAL_STRIP_COMMENTS  去掉 MATLAB 源码里的注释
%  必须能认出字符串: 本代码库里有 9 个文件的字符串字面量含 '%'
%  (sprintf('%d') 之类), 用正则删注释会把它们截断 —— 开发期踩过一次,
%  sprintf 遇到非法转义静默截断, 毁掉过一个文件。
lines = regexp(t, '\r\n|\n|\r', 'split');
keep = cell(numel(lines),1);
k = 0;
prevBlank = true;                          % 开头不留空行
for i = 1:numel(lines)
    raw = lines{i};
    wasBlank = isempty(strtrim(raw));
    c = deblank(local_cut_comment(raw));
    if isempty(strtrim(c))
        if wasBlank && ~prevBlank           % 原本就是空行 -> 保留一个, 用来分块
            k = k + 1; keep{k} = '';
            prevBlank = true;
        end
        continue                            % 整行注释 -> 丢掉
    end
    k = k + 1; keep{k} = c;
    prevBlank = false;
end
keep = keep(1:k);
while ~isempty(keep) && isempty(keep{end}), keep(end) = []; end
out = strjoin(keep.', sprintf('\n'));
end

function code = local_cut_comment(s)
%  逐字符扫; 状态: 0=代码 1=单引号字符串 2=双引号字符串
n = numel(s);  st = 0;  i = 1;  code = s;
while i <= n
    c = s(i);
    switch st
        case 0
            if c == '%'
                %  %#codegen / %#ok<...> 是**编译指示**不是注释, 必须留着。
                %  %#codegen 丢了不会立刻报错, 但 codegen 兼容性检查就不做了。
                if i < n && s(i+1) == '#'
                    return                      % 整行原样保留
                end
                code = s(1:i-1);  return
            elseif c == ''''
                %  转置还是字符串开头? 看**紧挨着**的前一个字符 —— 不跳空格,
                %  否则 [a 'str'] 里的引号会被当成转置。
                if i > 1 && local_is_word_end(s(i-1))
                    % 转置, 状态不变
                else
                    st = 1;
                end
            elseif c == '"'
                st = 2;
            end
        case 1
            if c == ''''
                if i < n && s(i+1) == '''', i = i + 1; else, st = 0; end
            end
        case 2
            if c == '"'
                if i < n && s(i+1) == '"', i = i + 1; else, st = 0; end
            end
    end
    i = i + 1;
end
end

function pos = local_nonascii_outside_string(t)
%  返回第一个出现在字符串字面量之外的非 ASCII 字符位置; 没有则返回 0
pos = 0;  st = 0;  i = 1;  n = numel(t);
while i <= n
    c = t(i);
    if c == sprintf('\n'), st = 0; i = i + 1; continue; end
    switch st
        case 0
            if double(c) > 127, pos = i; return
            elseif c == ''''
                if ~(i > 1 && local_is_word_end(t(i-1))), st = 1; end
            elseif c == '"', st = 2;
            end
        case 1
            if c == ''''
                if i < n && t(i+1) == '''', i = i + 1; else, st = 0; end
            end
        case 2
            if c == '"'
                if i < n && t(i+1) == '"', i = i + 1; else, st = 0; end
            end
    end
    i = i + 1;
end
end

function tf = local_is_word_end(c)
tf = isletter(c) || any(c == '0123456789_)]}.''');
end

function h = local_header(nm)
%LOCAL_HEADER  生成英文文件头。开发目录里的中文说明不进包。
persistent M
if isempty(M)
    M = containers.Map();
    M('setup_pmpc')      = 'Build every controller parameter and initial state; writes PMPC_P to the base workspace.';
    M('hil_init')        = 'One-shot initialization on the deployment machine. Run this before opening the model.';
    M('hil50')           = 'In-place conversion of an older pmpc_hil: replaces the 62-input scatter Mux with a plain 50-input Mux. Only needed once, on a model whose Inports were already replaced by NI In blocks.';
    M('pmpc_block')      = 'Body of the PMPC_MF MATLAB Function block. Cross-step state lives in a persistent.';
    M('pmpc_step')       = 'One control step: state estimation, reference, upper-level MPC, lower-level allocation.';
    M('wsget')           = 'Read one run-configuration value (CarSim dataset > base workspace > default).';
    M('func_AFS')        = 'Active front steering command from the upper-level MPC solution.';
    M('func_BuildQPConstraints') = 'Assemble the inequality constraints of the upper-level MPC QP.';
    M('func_BuildQPCost')        = 'Assemble H and f of the upper-level MPC QP.';
    M('func_CarSimResDir')       = 'Resolve the CarSim results directory from simfile.sim. Desktop only; unused on the HIL target.';
    M('func_CostWeightingRegulation_QuadSlacks') = 'Cost weights and quadratic slack penalties, including the priority variables.';
    M('func_DynamicalModel')     = 'Linear time-varying prediction model: augmented A/B/D over the horizon.';
    M('func_Envelope')           = 'Stability and rollover envelope bounds used as MPC constraints.';
    M('func_FindBezierControlPointsND') = 'Bezier control points for local path smoothing.';
    M('func_Fz_Calpha')          = 'Cornering stiffness as a function of vertical load.';
    M('func_InitialParams')      = 'Initial values of all cross-step controller state.';
    M('func_LTRDiagnosis')       = 'Load transfer ratio (LTR) computation and diagnosis.';
    M('func_MRDamper')           = 'MR damper force model and its inverse.';
    M('func_QPA_CDC')            = 'Lower-level allocation QP for the MR semi-active suspension.';
    M('func_QPA_DB')             = 'Lower-level allocation QP for differential braking.';
    M('func_QPKwik')             = 'Wrapper around mpcqpsolver (KWIK active-set QP), written against the R2018a API.';
    M('func_RLSFilter_Calpha_f') = 'Recursive least squares estimate of front-axle cornering stiffness.';
    M('func_RLSFilter_Calpha_r') = 'Recursive least squares estimate of rear-axle cornering stiffness.';
    M('func_RefTraj_LocalPlanning') = 'Local reference path planning and projection onto the path.';
    M('func_ReportStatus')       = 'QP status counters and error tracking; optional per-step printing.';
    M('func_RunMode')            = 'Pick the controller from the CarSim dataset name. Desktop only.';
    M('func_SolveMPCQP')         = 'Solve the upper-level MPC QP.';
    M('func_StateEstimation')    = 'Unpack the 50 plant measurements into the state structs (index = CarSim export channel).';
    M('func_SystemFurture')      = 'Build the prediction matrices PSI / THETA / PHI.';
    M('func_TireTable')          = 'Measured tire carpet lookup.';
    M('func_VehicleParams')      = 'Vehicle parameters.';
    M('func_WarmStart_shiftHorizon') = 'Shift the previous QP solution one step for warm starting.';
    M('func_WayPoints')          = 'Generate the reference path waypoints.';
    M('func_ZengRho')            = 'Zeng 2025 stability indicator rho (comparison method).';
    M('func_bezierInterp')       = 'Bezier interpolation.';
    M('func_tire_init_Calpha')   = 'Initialize the cornering stiffness estimate.';
end

%  个别文件的关键提示 —— 这些是在部署机上真会咬人的, 值得留在包里
persistent N
if isempty(N)
    N = containers.Map();
    N('hil_init') = {
        'Do NOT call setup_pmpc directly on the deployment machine.'
        'setup_pmpc picks the controller from the CarSim dataset name; with no'
        'CarSim present it silently falls back to the baseline MPC, not PMPC.'};
    N('hil50') = {
        'Run this ONLY on a pmpc_hil built before 2026-09-18, i.e. one whose'
        'Mux feeding PMPC_MF has 62 inputs. It finds the NI In blocks by following'
        'the wiring, not by name, so renamed blocks are fine. It is idempotent:'
        'on an already-converted model it reports and does nothing.'};
    N('setup_pmpc') = {
        'PMPC_MODE / PMPC_ZENGRHO / PMPC_VERBOSE in the base workspace override'
        'the dataset-based selection. On the deployment machine they MUST be set'
        '(hil_init does this). PMPC_VERBOSE = 0 is required before code generation.'};
    N('pmpc_block') = {
        'PMPC_P is declared Scope = Parameter and Tunable = false in the block.'
        'Tunable = false is the block-level coder.Constant: without it the array'
        'sizes derived from PMPC_P stop being compile-time constants and the block'
        'reports that it cannot determine the output size.'};
    N('func_QPKwik') = {
        'mpcqpsolver uses the convention A*x >= b, not A*x <= b.'
        'The active set is warm started through a persistent variable, so two'
        'copies of the controller running in parallel would corrupt each other.'};
    N('pmpc_step') = {
        'The two early-return paths must write the cross-step state back into St.'
        'They used to rely on globals; after the refactor a missing write-back'
        'left the controller inert with no visible error.'};
    N('func_ReportStatus') = {
        'verbose = 0 keeps the counters but prints nothing. Required for codegen:'
        'the flag comes from MPCParameters, which is a compile-time constant, so'
        'the whole printing branch is folded away when it is 0.'};
end

if isKey(M, nm), role = M(nm); else, role = 'Runtime function of the PMPC coordinated chassis controller.'; end
L = { sprintf('%%%s  %s', upper(nm), role)
      '%'
      '%   Part of the PMPC HIL deployment package (MATLAB R2018a + NI VeriStand).'
      '%   Comments were stripped when this package was generated; the full design'
      '%   notes live in the development copy of the project.' };
if isKey(N, nm)
    L{end+1} = '%';
    ex = N(nm);
    for i = 1:numel(ex)
        L{end+1} = ['%   ' ex{i}]; %#ok<AGROW>
    end
end
h = strjoin(L.', sprintf('\n'));
end

%  (原来的 local_to_gbk 已抽成独立的 func_ToGbk.m, 与 make_mil 共用)

function t = local_read(p)
t = '';
for enc = {'UTF-8','GBK','ISO-8859-1'}
    try
        f = fopen(p,'r','n',enc{1});
        t = fread(f, inf, '*char').';
        fclose(f);
        return
    catch
        try, fclose(f); catch, end
    end
end
end

function local_write_init(D)
%  hil_init 是包里第一个被运行的文件, 内容全英文 —— 部署机上的控制台输出
%  不会因为代码页问题乱码。
L = {
'function hil_init()'
'%HIL_INIT  One-shot initialization on the deployment machine.'
'%'
'%   Run this before opening the model or generating code.'
'%'
'%   Do NOT call setup_pmpc directly. It picks the controller from the CarSim'
'%   dataset name; with no CarSim present it silently falls back to the'
'%   baseline MPC instead of PMPC, and nothing tells you.'
'assignin(''base'', ''PMPC_MODE'',    1);'
'assignin(''base'', ''PMPC_ZENGRHO'', 0);'
'assignin(''base'', ''PMPC_VERBOSE'', 0);'
'evalin(''base'', ''setup_pmpc;'');'
''
'P = evalin(''base'',''PMPC_P'');'
'fprintf(''\n===== check before generating code =====\n'');'
'if P.Pm.ContrlMode == 1'
'    fprintf(''  controller   PMPC  (Nr=%d)      [ok]\n'', P.Pm.MPCParameters.Nr);'
'else'
'    fprintf(2, ''  controller   NOT PMPC!  ContrlMode=%d\n'', P.Pm.ContrlMode);'
'end'
'if P.Pm.MPCParameters.Verbose == 0'
'    fprintf(''  per-step log off                 [ok]\n'');'
'else'
'    fprintf(2, ''  per-step log STILL ON - fprintf will end up in the code\n'');'
'end'
'fprintf(''  PMPC_P       Pm %d fields / S0 %d fields\n'', ...'
'        numel(fieldnames(P.Pm)), numel(fieldnames(P.S0)));'
'fprintf(''\n  next: open pmpc_hil, swap in NI In/Out, set NIVeriStand.tlc + C++\n'');'
'fprintf(''  see README_deploy.md\n\n'');'
'end'
};
f = fopen(fullfile(D,'hil_init.m'),'w');
fwrite(f, unicode2native(strjoin(L.', sprintf('\n')), 'UTF-8'));
fclose(f);
end

function local_write_readme(D)
f = fopen(fullfile(D,'README_deploy.md'),'w','n','UTF-8');
fprintf(f, '# HIL 部署包 (R2018a + NI VeriStand)\n\n');
fprintf(f, '整个文件夹拷到部署机任意目录即可, 没有对路径的依赖。\n\n');
fprintf(f, '## 步骤\n\n');
fprintf(f, '### 1. 初始化(必须第一步)\n\n');
fprintf(f, '在 MATLAB R2018a 里 `cd` 到本目录, 然后:\n\n');
fprintf(f, '```\nhil_init\n```\n\n');
fprintf(f, '它会把控制器设成 ③ 本文 PMPC、关掉每拍打印、跑 `setup_pmpc` 生成工作区变量\n');
fprintf(f, '`PMPC_P`, 最后打一份核对清单。**确认打印的是"③ 本文 PMPC"再往下走。**\n\n');
fprintf(f, '> ⚠️ 不要直接跑 `setup_pmpc`。它靠读 CarSim 的 `simfile.sim` 认数据集名来选\n');
fprintf(f, '> 控制器, 部署机没有 CarSim, 读不到就**静默退回 baseline MPC**, 不是本文方法。\n\n');
fprintf(f, '### 2. 打开模型\n\n');
fprintf(f, '```\nopen_system(''pmpc_hil'')\n```\n\n');
fprintf(f, '结构:\n\n```\n');
fprintf(f, 'In_* x50 (标量 Inport) --> Mux(50) --> PMPC_MF --> Selector(18) --> Demux\n');
fprintf(f, '                                                                          |\n');
fprintf(f, '                                                     Out_* x18 <----------+\n');
fprintf(f, '                                                                        +--> PID vel --> Out_* x2\n');
fprintf(f, '```\n\n');
fprintf(f, '> ⚠️ **如果你手上已经有一份换好 NI In 的旧 pmpc_hil.slx**(2026-09-18 之前\n');
fprintf(f, '> 生成的, 喂给 PMPC_MF 的 Mux 是 62 口 + 一个常数 0), **不要用本包里这份覆盖它**,\n');
fprintf(f, '> 否则 50 个 NI In 要重换一遍。把本包的 .m 拷过去, 然后在那份旧模型上跑一次:\n\n');
fprintf(f, '```\nhil50\n```\n\n');
fprintf(f, '它顺着连线找 NI In(**不看块名**), 把 Mux(62)+常数 0 换成 Mux(50), 上游一个不动。\n');
fprintf(f, '重复跑没事, 已经是 50 口就直接返回。\n\n');
fprintf(f, '### 3. 换 NI In / NI Out\n\n');
fprintf(f, '50 个 Inport 和 20 个 Outport 都是**独立标量端口**, 逐个换成 NI VeriStand\n');
fprintf(f, '模块库里的 `NI In` / `NI Out` 即可。对应关系见 `PORTMAP_HIL.md`,\n');
fprintf(f, '或直接用 `portmap_hil_in.csv` / `portmap_hil_out.csv`。\n\n');
fprintf(f, '**端口号即映射顺序** —— `In_*` 的 Port 属性是 1..50, `Out_*` 是 1..20,\n');
fprintf(f, '和两个 csv 的行号一一对应。换的时候保持端口号不变就不会错位。\n\n');
fprintf(f, '端口名带 `In_` / `Out_` 前缀是因为 Simulink 要求同层块名唯一, 而\n');
fprintf(f, '`Fd_L1..R2`(减振器实测力 vs MR 指令力)和 `Vx` 在输入输出里都重名。\n\n');
fprintf(f, '### 4. 配代码生成\n\n');
fprintf(f, '| 项 | 值 |\n|---|---|\n');
fprintf(f, '| System target file | `NIVeriStand.tlc` |\n');
fprintf(f, '| Language | `C++` |\n');
fprintf(f, '| Template makefile | `NIVeriStand_vc.tmf` |\n');
fprintf(f, '| Make command | `make_rtw` |\n\n');
fprintf(f, '本机(R2024b)没装 NI 目标, 所以包里的模型是 `ode1` + `TargetLang=C++`,\n');
fprintf(f, '系统目标文件还是默认的, 需要你在部署机上改。\n\n');
fprintf(f, '### 5. 生成代码\n\n');
fprintf(f, 'Ctrl+B 或 `rtwbuild(''pmpc_hil'')`。\n\n');
fprintf(f, '---\n\n## 已知问题\n\n');
fprintf(f, '### 求解器: 目前是 ode1, 不是 FixedStepDiscrete\n\n');
fprintf(f, '`PID velocity control` 子系统里的 `Int_I` 是**连续** Integrator, 有连续状态,\n');
fprintf(f, '`FixedStepDiscrete` 会直接拒绝:\n\n');
fprintf(f, '```\nSimulink:Engine:InvalidSolver\n');
fprintf(f, '"FixedStepDiscrete" 求解器不能用于仿真模块图, 因为它包含连续状态\n```\n\n');
fprintf(f, '`ode1`(定步长显式欧拉)能生成代码也能上实时目标, 且行为与桌面模型\n');
fprintf(f, '(ode3 @1ms)最接近。若要对齐纯离散, 把 `Int_I` 换成 Discrete-Time Integrator\n');
fprintf(f, '(Forward Euler, T=0.001) —— 那会改变纵向 PID 的数值, 需要单独验收。\n\n');
fprintf(f, '### R2018a 的类型推断比 R2024b 弱\n\n');
fprintf(f, '所有 codegen 结论都是在 R2024b 上得到的。若 `PMPC_MF` 报\n');
fprintf(f, '**"无法确定输出大小和/或类型"**, 先查块里数据对象 `PMPC_P` 的\n');
fprintf(f, '**`Tunable` 是不是 `false`** —— 这是块层面的 `coder.Constant`,\n');
fprintf(f, '不设的话 `nvars = Nu*Nc+Ne+Nr` 这类维度会退回运行时值, 数组尺寸就推不出来。\n');
fprintf(f, '(开发阶段踩过一次。)\n\n');
fprintf(f, '其他类型推断报错请把信息发回来。\n\n');
fprintf(f, '---\n\n## 时序\n\n');
fprintf(f, '| | |\n|---|---|\n');
fprintf(f, '| 模型基频 `FixedStep` | 0.001 s (= VeriStand 1000 Hz) |\n');
fprintf(f, '| `PMPC_MF` 采样时间 | 0.01 s (= 10 ms 控制周期) |\n\n');
fprintf(f, '`PMPC_MF` 的采样时间**必须显式设**, 不能继承: 它内部用 `persistent` 保存跨拍\n');
fprintf(f, '状态, 而带 `persistent` 的 MATLAB Function 块在连续采样时间下非法\n');
fprintf(f, '(`Stateflow:Runtime:IllegalPersistentVarInContinuousTimeChart`)。\n');
fprintf(f, '它是 chart 对象的属性(`ChartUpdate`/`SampleTime`), 不是块参数。\n\n');
fprintf(f, '实时性预算: 一拍全部运算(含两个分配 QP)必须 < 10 ms。开发机上解释执行是\n');
fprintf(f, '22~29 ms/拍, 生成 C++ 后应该快一个量级, 但**生成代码后要实测**。\n');
fclose(f);
end
