function make_mil()
%MAKE_MIL  打包 R2018a 机器上跑 MIL(CarSim + Simulink)需要的全部文件到 MIL_deploy\
%
%   与 make_deploy(HIL 包)的区别:
%     HIL 包 = pmpc_hil.slx (NI In/Out, 没有被控对象) + 运行期 .m
%     MIL 包 = cq3_2019.slx (含 CarSim S-Function) + 运行期 .m + 跑仿真的工具
%
%   用法: setup_pmpc; make_mil
%
%   ⚠️ 注释**不剥**, 只转 GBK。MIL 包是拿去验证/排查的, 中文说明有用;
%      GBK 转换保证 R2018a(中文 Windows 按 ANSI 代码页读 .m)不乱码。
%      HIL 包剥注释是另一回事 —— 那是要交付给 NI 侧的干净产物。

root = func_ProjectRoot();
D = fullfile(root, 'MIL_deploy');
if exist(D,'dir')
    for sub = {'', 'baseline_ref', 'baseline_ref_mil', 'carsim'}
        dd = fullfile(D, sub{1});
        if ~exist(dd,'dir'), continue; end
        old = dir(fullfile(dd,'*'));
        for i = 1:numel(old)
            if ~old(i).isdir, delete(fullfile(dd, old(i).name)); end
        end
    end
else
    mkdir(D);
end

% ---------- 1. 运行期 .m (依赖分析) ----------
fprintf('  依赖分析中...\n');
flist = matlab.codetools.requiredFilesAndProducts( ...
    {fullfile(root,'setup_pmpc.m'), fullfile(root,'controller','pmpc_block.m')});
n = 0;
for i = 1:numel(flist)
    [~,nm,ext] = fileparts(flist{i});
    if ~strcmpi(ext,'.m'), continue; end
    copyfile(flist{i}, fullfile(D,[nm ext]));  n = n + 1;
end

% ---------- 2. 跑仿真/比对的工具 ----------
tools = {'run_ds.m','run3b.m','chk_regress.m','func_Metrics.m', ...
         'func_ReadERD.m','func_WaitERD.m','func_ErdDir.m','mil_init.m','chk_ver.m'};
for i = 1:numel(tools)
    src = which(tools{i});
    if isempty(src), error('make_mil:NoTool', '找不到工具 %s', tools{i}); end
    copyfile(src, fullfile(D,tools{i}));
end
fprintf('  .m 文件  : 运行期 %d 个 + 工具 %d 个\n', n, numel(tools));

% ---------- 3. 数据 ----------
mats = {'TireCarpet_265_75R16.mat','ZengSaddleDB_SUV.mat'};
for i = 1:numel(mats)
    copyfile(fullfile(root,'data',mats{i}), fullfile(D,mats{i}));
end

% ---------- 4. 模型 ----------
%  两个模型都带上:
%    cq3_2019 —— 原 MIL, 所有 baseline 与论文数据出自它(Function-Call 触发 + ode3)
%    pmpc_mil —— 以 pmpc_hil 为底接 CarSim, 验证**HIL 那套架构**在闭环里对不对
%                (PMPC_MF 自带 SampleTime 0.01 + ode1, 与 HIL 完全一致)
load_system('cq3_2019');
load_system('pmpc_mil');
%  ⚠️ 导出前 PMPC_MF 的 VectorOutputs1D 必须是 1, 否则 R2018a 上这个块会失效
rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart','Path','cq3_2019/Subsystem1/PMPC_MF');
if ~isempty(ch) && ch.VectorOutputs1D ~= 1
    error('make_mil:Vec1D', 'cq3_2019 的 PMPC_MF VectorOutputs1D 不是 1, 导出到 R2018a 会失效');
end
Simulink.exportToVersion('cq3_2019', fullfile(D,'cq3_2019.slx'), 'R2018A');
Simulink.exportToVersion('pmpc_mil', fullfile(D,'pmpc_mil.slx'), 'R2018A');
fprintf('  模型     : cq3_2019.slx + pmpc_mil.slx (R2018a 格式)\n');

% ---------- 5. 参考基准(用来核对 2018a 上跑出来的对不对) ----------
bd = fullfile(D,'baseline_ref');
if ~exist(bd,'dir'), mkdir(bd); end
for pair = { {'baseline_mf','baseline_ref'}, {'baseline_mil','baseline_ref_mil'} }
    srcd = fullfile(root, 'simulation_results', 'archive', pair{1}{1}, 'erd');
    dstd = fullfile(D, pair{1}{2});
    if ~exist(dstd,'dir'), mkdir(dstd); end
    for t = {'MPC','ZENG','PMPC'}
        copyfile(fullfile(srcd,[t{1} '.vs']),  fullfile(dstd,[t{1} '.vs']));
        copyfile(fullfile(srcd,[t{1} '.vsb']), fullfile(dstd,[t{1} '.vsb']));
    end
end

% ---------- 6. CarSim 的 50 路 Export 数据集 ----------
%  模型是按 50 路接的, 但**数据集在 CarSim 装机目录里, 不在包里**。
%  那台机器的 CarSim 如果还是 62 路导出, 一跑就报端口宽度错。
%  这里把改好的 Export 数据集(2.3 KB 独立文件, 不链接其他数据集)带上,
%  外加一份纯文本通道清单, 万一 GUID 对不上可以照着手工勾。
cd_ = fullfile(D,'carsim');
if ~exist(cd_,'dir'), mkdir(cd_); end
EXP = ['D:' filesep 'Program Files' filesep 'CarSim2019.0' filesep 'CarSim2019.0_Data' ...
       filesep 'IO_Channels' filesep 'O_Channels' filesep ...
       'Export_389fd0a1-377d-40d5-88be-3701fde6c772.par'];
if exist(EXP,'file') == 2
    copyfile(EXP, cd_);
    %  纯文本清单: 万一 GUID 对不上, 照着在 CarSim 界面里手工勾
    txt = fileread(EXP);
    ch  = regexp(txt, '(?m)^EXPORT\s+(\S+)', 'tokens');
    g = fopen(fullfile(cd_,'export_50_channels.txt'),'w');
    fprintf(g, 'CarSim I/O Channels: Export  共 %d 路\n', numel(ch));
    fprintf(g, '(原 62 路去掉 CmpD_L1/L2/R1/R2, Zgnd_L1/L2/R1/R2, Z_L1/L2/R1/R2 共 12 路)\n\n');
    for i = 1:numel(ch), fprintf(g, '%2d  %s\n', i, ch{i}{1}); end
    fclose(g);
    fprintf('  CarSim   : carsim\\ (50 路 Export 数据集 + 通道清单)\n');
else
    warning('make_mil:NoExportPar', '找不到 Export 数据集, carsim\\ 目录会是空的');
end

% ---------- 7. README ----------
local_write_readme(D);

% ---------- 8. 全部 .m 转 GBK(不剥注释) ----------
d = dir(fullfile(D,'*.m'));
nc = 0;
for i = 1:numel(d)
    nc = nc + func_ToGbk(fullfile(D, d(i).name));
end
fprintf('  编码     : %d/%d 个 .m 转成 GBK(并去 BOM)\n', nc, numel(d));

d = dir(fullfile(D,'**','*'));
d = d(~[d.isdir]);
fprintf('  >>> %s: %d 个文件, %.1f MB <<<\n', D, numel(d), sum([d.bytes])/1024/1024);
end

% -------------------------------------------------------------------------
function local_write_readme(D)
f = fopen(fullfile(D,'README_MIL.md'),'w','n','UTF-8');
fprintf(f, '# MIL 包 (R2018a + CarSim)\n\n');
fprintf(f, '在装了 R2018a 的机器上跑 CarSim + Simulink 闭环, 先确认算法正常, 再做 NI。\n');
fprintf(f, '整个文件夹拷到任意目录即可。\n\n');
fprintf(f, '## 步骤\n\n');
fprintf(f, '### 1. CarSim 里选数据集并 Send to Simulink\n\n');
fprintf(f, '\n');
fprintf(f, '**第一次在新机器上必须先改路径**, 否则 Send 无从谈起, 报:\n');
fprintf(f, '\n');
fprintf(f, '```\n');
fprintf(f, 'Error reported by S-function ''vs_sf'' in ''cq3_2019/CarSim S-Function1'':\n');
fprintf(f, 'Error: Unable to find solver DLL path from sim file.\n');
fprintf(f, '```\n');
fprintf(f, '\n');
fprintf(f, '`vs_sf` 要从 `simfile.sim` 里读 `PROGDIR` 才能找到求解器 DLL, 而那个文件\n');
fprintf(f, '**不在本包里** —— 它是 CarSim 点 Send to Simulink 时现写的, 必须在本机生成。\n');
fprintf(f, '\n');
fprintf(f, '在 CarSim 里打开 `Models: Simulink` 数据集(三个 Run Control 共用的那个),\n');
fprintf(f, '把两个路径改成本机的:\n');
fprintf(f, '\n');
fprintf(f, '| 字段 | 改成 |\n');
fprintf(f, '|---|---|\n');
fprintf(f, '| Working directory | 本包所在目录, 例如 `D:\\XJM\\MIL_deploy` |\n');
fprintf(f, '| Simulink Model | 同上目录下的 `cq3_2019.slx` |\n');
fprintf(f, '\n');
fprintf(f, '改完回 Run Control 点 **Send to Simulink**, `simfile.sim` 就会写进本包目录。\n');
fprintf(f, '**不能直接 `sim(cq3_2019)`** —— 至少要先 Send 一次。\n');
fprintf(f, '\n');
fprintf(f, '`DLC80_mu0.5_MPC` / `_ZENG` / `_PMPC` 三选一, 点 **Send to Simulink**。\n');
fprintf(f, 'CarSim 会把 `simfile.sim` 写进 Simulink 模型所在目录, 并打开 MATLAB。\n\n');
fprintf(f, '> ⚠️ 前提: 那台机器的 CarSim 里要有这三个数据集, 且 I/O Channels 的\n');
fprintf(f, '> **Export 是 50 路**(原 62 路去掉 `CmpD_*` / `Zgnd_*` / `Z_*` 共 12 路)。\n');
fprintf(f, '> 模型按 50 路接的, 数量对不上会直接报端口宽度错。见下一节。\n\n');
fprintf(f, '### 1b. 把 CarSim 的 Export 改成 50 路\n');
fprintf(f, '\n');
fprintf(f, '`carsim\\` 里带了开发机上改好的那个数据集(2.3 KB, 不链接其他数据集):\n');
fprintf(f, '\n');
fprintf(f, '```\n');
fprintf(f, 'Export_389fd0a1-377d-40d5-88be-3701fde6c772.par   <- 50 路的 Export 数据集\n');
fprintf(f, 'export_50_channels.txt                            <- 纯文本通道清单\n');
fprintf(f, '```\n');
fprintf(f, '\n');
fprintf(f, '**办法 A(GUID 对得上时最省事)**: 把那个 `.par` 拷到那台机器的\n');
fprintf(f, '`CarSim2019.0_Data\\IO_Channels\\O_Channels\\` 覆盖同名文件。\n');
fprintf(f, '\n');
fprintf(f, '> !! 必须按这个顺序, 否则改动会被 CarSim 覆盖掉:\n');
fprintf(f, '> 1. **完全退出 CarSim**(它退出时会把内存里的数据集回写到磁盘)\n');
fprintf(f, '> 2. 拷文件\n');
fprintf(f, '> 3. 删掉 `CarSim2019.0_Data\\Configuration\\index.cache`\n');
fprintf(f, '> 4. 重启 CarSim(数据集索引是启动时扫描建立的, 只删 cache 不够)\n');
fprintf(f, '> 5. 三个数据集各点一次 **Send to Simulink**\n');
fprintf(f, '\n');
fprintf(f, '**办法 B(GUID 对不上, 或嫌上面麻烦)**: 在 CarSim 界面里手工改。\n');
fprintf(f, '打开 `I/O Channels: Export`, 在右边 "Variables Activated for Export" 里双击取消这 12 个:\n');
fprintf(f, '\n');
fprintf(f, '```\n');
fprintf(f, 'CmpD_L1  CmpD_L2  CmpD_R1  CmpD_R2\n');
fprintf(f, 'Zgnd_L1  Zgnd_L2  Zgnd_R1  Zgnd_R2\n');
fprintf(f, 'Z_L1     Z_L2     Z_R1     Z_R2\n');
fprintf(f, '```\n');
fprintf(f, '\n');
fprintf(f, '剩下 50 路的顺序要和 `export_50_channels.txt` 完全一致 —— **顺序决定下标**,\n');
fprintf(f, '错一位整个状态估计就全错了。改完各点一次 Send to Simulink。\n');
fprintf(f, '\n');
fprintf(f, '> ⚠️ 没有"办法 C"了: 控制器现在**按 50 路编号**(func_StateEstimation 的\n');
fprintf(f, '> ModelInput 下标 1..50 直接对应 CarSim Export 的 1..50), 62 路导出会直接报\n');
fprintf(f, '> 端口宽度错。那台机器的 Export 必须是 50 路。\n');
fprintf(f, '\n');
fprintf(f, '### 2. 初始化\n\n```\ncd 到本目录\nmil_init\n```\n\n');
fprintf(f, '控制器由数据集名自动决定(名字含 ZENG -> 对比方法; 含 PMPC -> 本文方法; 其余 -> baseline)。\n');
fprintf(f, '要强制指定就 `mil_init(1)` = PMPC。\n\n');
fprintf(f, '\n');
fprintf(f, '## 两个模型, 用哪个\n');
fprintf(f, '\n');
fprintf(f, '| 模型 | 用途 | 控制器时钟 | 求解器 |\n');
fprintf(f, '|---|---|---|---|\n');
fprintf(f, '| `cq3_2019` | 原 MIL, 所有 baseline 与论文数据出自它 | Function-Call 触发 | ode3 |\n');
fprintf(f, '| `pmpc_mil` | **验证 HIL 架构**, 上 NI 之前跑这个 | PMPC_MF 自带 0.01 | ode1 |\n');
fprintf(f, '\n');
fprintf(f, '`pmpc_mil` 是以 `pmpc_hil.slx` 为底把 NI In/Out 换成 CarSim 做的 ——\n');
fprintf(f, '控制器块、时钟、求解器与 HIL **完全一致**, 所以它跑通才真正说明 HIL 那套能用。\n');
fprintf(f, '`cq3_2019` 的架构与 HIL 不同(触发方式和求解器都不一样), 跑通不保证 HIL 也对。\n');
fprintf(f, '\n');
fprintf(f, '在 CarSim 的 `Models: Simulink` 数据集里把 Simulink Model 指向哪个, 就跑哪个。\n');
fprintf(f, '\n');
fprintf(f, '对照基准分开放:\n');
fprintf(f, '\n');
fprintf(f, '```\n');
fprintf(f, 'baseline_ref/      <- cq3_2019 的结果\n');
fprintf(f, 'baseline_ref_mil/  <- pmpc_mil 的结果\n');
fprintf(f, '```\n');
fprintf(f, '\n');
fprintf(f, '开发机上实测的 RTIME(实时时间/仿真时间, <1 才跟得上实时):\n');
fprintf(f, '\n');
fprintf(f, '| | cq3_2019 | pmpc_mil |\n');
fprintf(f, '|---|---|---|\n');
fprintf(f, '| MPC | 1.04 | **0.193** |\n');
fprintf(f, '| ZENG | 1.06 | **0.210** |\n');
fprintf(f, '| PMPC | **2.13** | **0.256** |\n');
fprintf(f, '\n');
fprintf(f, '差 4~8 倍, 因为 `cq3_2019` 有 7 个 Scope + To Workspace 每 1 ms 记录, 且 ode3 是 3 级。\n');
fprintf(f, '也就是说 "PMPC 跟不上实时" 主要是桌面模型的记录开销, 不是算法本身。\n');
fprintf(f, '\n');
fprintf(f, '### 3. 跑\n\n```\nsim(''cq3_2019'')        % 跑当前数据集\nrun_ds(''PMPC'')        % 或者切数据集再跑(需要三个都 Send 过)\nrun3b                 % 三个控制器依次跑\n```\n\n');
fprintf(f, '## 怎么判断"没问题"\n\n');
fprintf(f, '`baseline_ref\\` 里是开发机(R2024b)上跑的三个控制器 ERD, 拿来对照。\n\n');
fprintf(f, '```\nchk_regress(''-base'', ''baseline_ref'')\n```\n\n');
fprintf(f, '> ⚠️ **不要指望 MD5 逐字节一致。** R2018a 与 R2024b 的数学库版本不同,\n');
fprintf(f, '> 末位舍入必然有差异, 再被 QP 活动集翻转和闭环反馈放大。\n');
fprintf(f, '> 判据看**指标表**: 拍数应当相同, 越界%% 为 0, |LTR|峰 与基准接近,\n');
fprintf(f, '> 没有标 `!!` 的安全类退化。这和当初 S-function -> MATLAB Function block\n');
fprintf(f, '> 的验收口径是一样的。\n\n');
fprintf(f, '参考值(开发机):\n\n');
fprintf(f, '| | 拍数 | 越界%% | \\|LTR\\|峰 |\n|---|---|---|---|\n');
fprintf(f, '| MPC | 157 | 0.00 | 0.4628 |\n');
fprintf(f, '| ZENG | 174 | 0.00 | 0.4124 |\n');
fprintf(f, '| PMPC | 156 | 0.00 | 0.4519 |\n\n');
fprintf(f, '---\n\n## 结构\n\n```\n');
fprintf(f, 'CarSim(50 路) --> Subsystem1/x(50)\n');
fprintf(f, '                      |\n');
fprintf(f, '                      +--> sel_1_4 (1:4) ---+\n');
fprintf(f, '                                    CarSim 50 路 --> PMPC_MF --> 54 路\n');
fprintf(f, '                      +--> sel_5_50 (5:50) -+\n```\n\n');
fprintf(f, '原 62 路里的 5-16 控制器从来没读过, 已从 CarSim 的 Export 里去掉;\n');
fprintf(f, '5-16 填 0 —— `pmpc_step` / `func_StateEstimation` 的下标一个字都不用改。\n');
fprintf(f, 'func_StateEstimation 的 52 处下标也已整体重编成 1..50(逐字节回归验过),\n');
fprintf(f, '所以现在 **CarSim 通道号 = ModelInput 下标 = NI In 序号**, 中间没有任何散布。\n\n');
fprintf(f, '## 已知事项\n\n');
fprintf(f, '- `MPC solve ... ms` 永远显示 0 —— 计时已从 `pmpc_step` 移除\n');
fprintf(f, '  (`tic` 不支持代码生成)。要量耗时在**外部**循环调 `pmpc_step` 做 tic/toc。\n');
fprintf(f, '- 跑仿真时 **CarSim Browser 必须开着**, 求解器要靠它取许可证。\n');
fprintf(f, '- 改过 CarSim 数据集之后必须重新 **Send to Simulink**, 否则展开结果\n');
fprintf(f, '  (`Results\\Run_<GUID>\\Run_all.par`)是旧的。`run_ds` 会检查并报错。\n');
fprintf(f, '- 代码生成相关的踩坑见 HIL 包里的 `HIL_R2018a_codegen_notes.md`。\n');
fclose(f);
end
