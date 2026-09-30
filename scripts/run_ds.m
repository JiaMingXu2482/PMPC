function info = run_ds(tag, varargin)
%RUN_DS  按 CarSim 数据集跑仿真; 控制器和模型由 tag 显式决定
%   run_ds('PMPC')          切到名字以 _PMPC 结尾的数据集并跑一次
%   run_ds('PMPC','-nosim') 只切指向, 不跑
%   run_ds('PMPC','-force') 展开结果过期也照跑(只发警告)
%   run_ds('PMPC','-ds','DLC80_mu0.5')  限定数据集名前缀(默认取当前指向的前缀)
%   info = run_ds(...)      返回 .name .guid .resdir .nb
%
%   为什么能不开 GUI 切:
%     仿真读的是 Results\Run_<GUID>\Run_all.par (CarSim 在 Send to Simulink 时
%     展开出来的), 而选哪个 Run_<GUID> 只由项目里的 simfile.sim 指向决定。
%     三个数据集都 Send 过一次以后, 换指向就等于换数据集。
%
%   ⚠️ 展开结果会过期: 在 CarSim 里改过数据集之后必须重新 Send to Simulink,
%      否则 Run_all.par 还是旧的(改了车速却跑出老车速, 且毫无提示)。
%      本函数比较两者时间戳, 过期直接报错, 不让它悄悄跑错。
%
%   run3b 就是对 MPC/ZENG/PMPC 三个 tag 依次调用本函数。

dosim = true;  prefix = '';  force = false;
model = func_ControllerModelForTag(tag);
i = 1;
while i <= numel(varargin)
    switch lower(varargin{i})
        case '-nosim', dosim = false;
        case '-force', force = true;
        case '-ds',    prefix = varargin{i+1}; i = i + 1;
    end
    i = i + 1;
end

SF = fullfile(func_ProjectRoot(), 'simfile.sim');
assert(exist(SF,'file')==2, 'run_ds:NoSimfile', ...
      ['找不到 %s —— 它是 CarSim "Send to Simulink" 写出的句柄文件, 不是临时文件。\n' ...
       '在 CarSim 里点一次 Send to Simulink 即可重新生成。'], SF);
sf = fileread(SF);
wd = regexp(sf,'WORK_DIR\)\$\s+([^\r\n]+)','tokens','once');
wd = strtrim(wd{1});
cur = regexp(sf,'ROOT_FILE_NAME\)\$\s+Run_(\S+)','tokens','once');
curGuid = cur{1};

RUNS = fullfile(wd,'Runs');
RES  = fullfile(wd,'Results');

%  当前指向的数据集名, 用来推默认前缀(如 DLC80_mu0.5_ZENG -> DLC80_mu0.5)
curName = local_dsname(fullfile(RUNS,['Run_' curGuid '.par']));
if isempty(prefix)
    k = find(curName=='_', 1, 'last');
    if isempty(k), prefix = curName; else, prefix = curName(1:k-1); end
end
want = [prefix '_' tag];

%  扫描: 只读每个 .par 的文件头(FullDataName 在第 2 行), 373 个文件全读要 100 MB
d = dir(fullfile(RUNS,'Run_*.par'));
guid = ''; name = '';
for i = 1:numel(d)
    nm = local_dsname(fullfile(RUNS,d(i).name));
    if strcmpi(nm, want)
        name = nm;
        guid = regexprep(d(i).name,'^Run_|\.par$','');
        break
    end
end
assert(~isempty(guid), 'run_ds:NotFound', '没找到数据集 %s', want);

%  ---- 过期检查 ----
%  不能只看时间戳: CarSim 离开数据集界面时会把它重存一遍, 时间戳变了内容却没变
%  (实测 PMPC 就这样被误判成过期)。改成比**值**:
%    1) 源数据集里每个参数赋值, 在展开结果里必须一致
%       (取首次出现 —— 展开结果里 Run Control 自身那份排在最前)
%    2) 展开之后有没有子数据集被改过(127 个链接逐个比 mtime)
srcPar = fullfile(RUNS, ['Run_' guid '.par']);
allPar = fullfile(RES, ['Run_' guid], 'Run_all.par');
assert(exist(allPar,'file')==2, 'run_ds:NeverSent', ...
      '数据集 %s 从未 Send to Simulink 过(没有 %s)', name, allPar);
[bad, chg] = local_stale(srcPar, allPar, wd);
if ~isempty(bad) || ~isempty(chg)
    msg = sprintf('数据集 %s 的展开结果已过期:\n', name);
    for i = 1:size(bad,1)
        msg = [msg sprintf('  参数 %-14s 源=%-14s 展开=%s\n', bad{i,1}, bad{i,2}, bad{i,3})]; %#ok<AGROW>
    end
    for i = 1:numel(chg)
        msg = [msg sprintf('  子数据集已改动: %s\n', chg{i})]; %#ok<AGROW>
    end
    msg = [msg '在 CarSim 里选中它点一次 Send to Simulink 再来' ...
               '(确认影响不到结果可以用 run_ds(..,''-force'') 强跑)。'];
    if force
        warning('run_ds:StaleForced', '%s', msg);
    else
        error('run_ds:Stale', '%s', msg);
    end
end

%  ---- 换指向 ----
if ~strcmp(guid, curGuid)
    fid = fopen(SF,'rb'); raw = fread(fid,inf,'*uint8'); fclose(fid);
    raw = strrep(char(raw(:).'), curGuid, guid);
    fid = fopen(SF,'wb'); fwrite(fid, uint8(raw)); fclose(fid);
end
fprintf('  数据集 -> %s  (Run_%s)\n', name, guid(1:8));

info = struct('name',name,'guid',guid,'resdir',fullfile(RES,['Run_' guid]), ...
              'model',model, 'nb',0);
func_SimModel(fullfile(info.resdir,'Run_all.par'), info.model);
if ~dosim, return; end

%  ---- 跑 ----
%  必须清掉, 否则工作区残留会盖掉 func_RunMode 的自动识别(见 wsget 优先级)
evalin('base','clear PMPC_MODE PMPC_ZENGRHO PMPC_P MPC_P ZENG_P');
dl = dir(fullfile(info.resdir,'LastRun_log.txt'));
if isempty(dl), ls0 = 0; else, ls0 = dl.datenum; end
%  CarSim Browser 没开的话, vs_sf 会弹模态对话框把整个批量作业挂死(见
%  func_CarSimRunning 的说明)。宁可在这里快速失败, 给一句能读懂的话。
assert(func_CarSimRunning(), 'run_ds:NoCarSim', ...
      ['CarSim Browser (或 CSLM.exe) 没在运行 —— 求解器取不到许可。\n' ...
       '直接跑的话 vs_sf 会弹一个模态对话框, 在 matlab -batch 里没人点, ' ...
       '作业会一直挂着。\n先打开 CarSim 再跑。']);
switch info.model
    case 'mpc_mil'
        mil_init_MPC;
    case 'zeng_mil'
        mil_init_ZENG;
    case 'pmpc_mil'
        mil_init_PMPC;
end
func_CarSimLib();     % 独立会话(matlab -batch)里没有 CarSim 的库路径, 自己挂上
evalc(sprintf('sim(''%s'');', info.model));
info.nb = func_WaitERD(info.resdir, ls0);
fprintf('  %s 完成 (%d 字节, %d 拍, 模型 %s)\n', ...
        name, info.nb, round(info.nb/3668), info.model);
end

% -------------------------------------------------------------------------
function nm = local_dsname(par)
%  只读文件头取数据集名 —— FullDataName 在第 2 行
nm = '';
fid = fopen(par,'r');
if fid < 0, return; end
head = fread(fid, 400, '*char').';
fclose(fid);
t = regexp(head,'FullDataName CarSim Run Control`([^`]+)`','tokens','once');
if ~isempty(t), nm = t{1}; end
end

% -------------------------------------------------------------------------
function [bad, chg] = local_stale(srcPar, allPar, wd)
%  bad: {参数, 源值, 展开值} ;  chg: 展开之后被改过的子数据集
bad = cell(0,3);  chg = {};
%  SPECIAL_PARSFILE 是**画图叠加**的引用, 不参与仿真, 必须跳过。
%  CarSim 在 Run Control 上挂一条 overlay 曲线就会把它写进数据集, 源文件
%  mtime 随之变新, 而展开结果里没有它 —— 于是"画了一张对比图"就会让这里
%  误报过期。2026-09-19 实际踩到: 用户叠了 MPC/ZENG 两条曲线出图, 随后
%  PMPC 那一档就跑不动了。数据集里的自述行说得很清楚:
%    SPECIAL_PARSFILE Runs\Run_ed104361-....par
%    #BlueLink23 CarSim Run Control`DLC80_mu0.5_MPC` PMPC` , Overlay run or ERD file`...
%                                                            ^^^^^^^^^^^^^^^^^^^^^^^
skip = {'ENTER_PARSFILE','EXIT_PARSFILE','LOG_ENTRY','SET_MACRO','LOG_START','PARSFILE', ...
        'SPECIAL_PARSFILE'};
src = strsplit(fileread(srcPar), sprintf('\n'));
all = strsplit(fileread(allPar), sprintf('\n'));

%  展开结果: 每个参数取首次出现
pe = containers.Map('KeyType','char','ValueType','char');
links = {};
for i = 1:numel(all)
    ln = strtrim(all{i});
    if strncmp(ln,'ENTER_PARSFILE',14)
        links{end+1} = strtrim(ln(15:end)); %#ok<AGROW>
        continue
    end
    t = regexp(ln,'^([A-Z][A-Z0-9_]*)\s+(.*)$','tokens','once');
    if ~isempty(t) && ~any(strcmp(t{1},skip)) && ~isKey(pe,t{1})
        pe(t{1}) = strtrim(t{2});
    end
end

seen = {};
for i = 1:numel(src)
    t = regexp(strtrim(src{i}),'^([A-Z][A-Z0-9_]*)\s+(.*)$','tokens','once');
    if isempty(t) || any(strcmp(t{1},skip)) || any(strcmp(t{1},seen)), continue; end
    seen{end+1} = t{1}; %#ok<AGROW>
    v = strtrim(t{2});
    if ~isKey(pe,t{1})
        bad(end+1,:) = {t{1}, v, '<缺>'}; %#ok<AGROW>
    elseif ~strcmp(pe(t{1}), v)
        bad(end+1,:) = {t{1}, v, pe(t{1})}; %#ok<AGROW>
    end
end

%  子数据集改动: 只看展开之后被动过的; Run Control 自身由上面的参数比对负责
da = dir(allPar);
[~, rcName] = fileparts(srcPar);
for i = 1:numel(links)
    f = fullfile(wd, links{i});
    d = dir(f);
    if isempty(d), continue; end
    [~, nm] = fileparts(f);
    if strcmp(nm, rcName), continue; end
    if d.datenum > da.datenum
        chg{end+1} = links{i}; %#ok<AGROW>
    end
end
end
