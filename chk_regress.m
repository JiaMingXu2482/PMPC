function ok = chk_regress(varargin)
%CHK_REGRESS  回归验收: 跑三个控制器, 与基准比对
%   chk_regress                 跑仿真后比对
%   chk_regress('-nosim')       跳过仿真, 直接比对 erd_out 里已有的结果
%   chk_regress('-cur',DIR)     比对指定目录的结果(隐含 -nosim)
%   chk_regress('-base',DIR)    指定基准目录 (默认 baseline_mf\erd)
%   其余选项(如 '-force')原样透传给 run3b -> run_ds
%   ok = chk_regress(...)       返回 true/false
%
%   判据 (见 PLAN_HIL迁移方案.md §4):
%     阶段 A/B(拆函数) —— 硬判据: 三份 ERD 的 MD5 与基准逐字节相同
%     阶段 S(换求解器) / 阶段 D(换 MATLAB Function block)
%                    —— MD5 必然失效, 看指标表, 且要求
%                       安全类指标(越界%%、|LTR|峰) 不得变差
%
%   基准 = baseline_mf\erd  (MATLAB Function block 路径, 2026-09-18 起唯一路径)
%   ⚠️ 2026-09-18 重建过一次: func_QPKwik 的 persistent 原来被三个调用点共用,
%   分配 QP(m=8) 每拍把上层 QP(m=154) 的活动集重置掉, **热启动从来没生效过**。
%   隔离 persistent 后热启动真正工作, 结果随之变化(PMPC 的 |LTR|峰 -0.17%,
%   ZENG +14ppm, MPC 逐字节不变), 故重建基准。
%   冷启动时代的基准留档在 baseline_mf_cold\erd, 需要时用 -base 指定。
%   MF block 是编译执行, 原 S-function 是解释执行, 末位舍入不同 ->
%   两者之间不可能逐字节一致; 但 MF 路径**自身是可复现的**,
%   所以对本基准 MD5 仍是硬判据。
%   历史基准(均为 S-function 路径, 已不可再跑, 仅留档):
%     baseline_0918      三个 QP 全走 KWIK
%     baseline_0917_kwik 仅上层 KWIK
%     baseline_0917      quadprog 时代
%
%   MD5 不一致时会定位到: 哪个控制器 / 首次偏离在第几拍 / 哪个通道。
root = fileparts(mfilename('fullpath'));
base = fullfile(root,'baseline_mf','erd');   % 阶段 D 后: MATLAB Function block
%  历史基准见上方说明, 需要时用 -base 指定。
dosim = true;
cur = '';
pass = {};      % 不认识的选项透传给 run3b/run_ds (如 '-force')
hasBase = false;
i = 1;
while i <= numel(varargin)
    switch lower(varargin{i})
        case '-nosim', dosim = false;
        case '-base',  base = varargin{i+1}; i = i+1; hasBase = true;
        case '-cur',   cur  = varargin{i+1}; i = i+1; dosim = false;
        otherwise,     pass{end+1} = varargin{i}; %#ok<AGROW>
    end
    i = i + 1;
end
if ~isempty(cur), out = cur; else, out = func_ErdDir(); end

%  ---- 基准要跟着 Simulink 模型走 ----
%  跑哪个模型由 CarSim 的 "Models: Simulink" 决定(见 func_SimModel), 而
%  baseline_mf 是 cq3_2019 跑出来的、baseline_mil 是 pmpc_mil 跑出来的。
%  拿错基准会得到一堆莫名其妙的 FAIL, 让人以为算法坏了。没显式给 -base 时
%  按模型自动选, 并**明确打印**选了哪个 —— 自动但不隐身。
if ~hasBase
    mdl = local_curmodel();
    switch mdl
        case 'cq3_2019', base = fullfile(root,'baseline_mf','erd');
        case 'pmpc_mil', base = fullfile(root,'baseline_mil','erd');
        otherwise
            warning('chk_regress:UnknownModel', ...
                ['CarSim 指向的模型是 %s, 没有对应的基准目录, 仍用默认 baseline_mf。\n' ...
                 '若结果大面积不符, 先确认基准选对了, 再怀疑算法。'], mdl);
    end
    fprintf('  模型 %-10s -> 基准 %s\n', mdl, base);
end
if ~exist(base,'dir'), error('chk_regress:NoBaseline','基准目录不存在: %s', base); end

if dosim
    fprintf('== 跑仿真 ==\n');
    run3b(out, pass{:});
end

tags = {'MPC','ZENG','PMPC'};
fprintf('\n== MD5 比对 ==\n');
ok = true;
for k = 1:3
    fb = fullfile(base,[tags{k} '.vsb']);
    fo = fullfile(out ,[tags{k} '.vsb']);
    if ~exist(fo,'file'), fprintf('  %-5s 缺少当前结果\n',tags{k}); ok=false; continue; end
    hb = md5_(fb); ho = md5_(fo);
    if strcmp(hb,ho)
        fprintf('  %-5s PASS  %s\n', tags{k}, hb(1:12));
    else
        ok = false;
        fprintf('  %-5s FAIL  基准 %s  当前 %s\n', tags{k}, hb(1:12), ho(1:12));
        locate_(fullfile(base,tags{k}), fullfile(out,tags{k}));
    end
end

fprintf('\n== 指标对比 (基准 -> 当前) ==\n');
%  车道口径 2026-09-26 起为车身角点(见 func_Metrics), 原"质心 |ey|>0.5325 越界"已废弃
rows = { 'ey RMS 质心 (m)','ey_rms','%.4f',1;  '|ey| 峰 质心 (m)','ey_pk','%.4f',1
         '角点峰 (m)','corner_pk','%.4f',1; '角点进余量 (%)','mgn_pct','%.2f',1
         '角点出车道 (%)','out_pct','%.2f',1; '|LTR| 峰','ltr_pk','%.4f',1
         '侧倾峰 (deg)','roll_pk','%.3f',1;'|r|>mu*g/Vx (%)','r_over','%.1f',1
         '尾段|SW|峰 (deg)','sw_tail','%.1f',1;'SW 速率RMS','dsw_rms','%.1f',1
         'dTb RMS','dTb_rms','%.0f',1;    '差动制动RMS','db_rms','%.0f',1
         '油门速率RMS','dthr_rms','%.2f',1;'平均车速 (km/h)','vx_mean','%.2f',0
         '拍数','N','%d',0 };
SAFE = {'corner_pk','mgn_pct','out_pct','ltr_pk'}; % 安全类, 不得变差
for k = 1:3
    fo = fullfile(out,[tags{k} '.vsb']);
    if ~exist(fo,'file'), continue; end
    B = func_Metrics(fullfile(base,tags{k}));
    C = func_Metrics(fullfile(out ,tags{k}));
    fprintf('\n  [%s]\n', tags{k});
    for r = 1:size(rows,1)
        f = rows{r,2}; b = B.(f); c = C.(f);
        if b == c, mk = '  ='; else
            if any(strcmp(f,SAFE)) && c > b, mk = ' !!';   % 安全类变差
            else, mk = '   '; end
        end
        d = '';
        if b ~= 0 && isfinite(b) && ~strcmp(f,'N')
            d = sprintf('  (%+.1f%%)', 100*(c-b)/abs(b));
        end
        fprintf('    %-18s %10s -> %-10s%s%s\n', rows{r,1}, ...
                sprintf(rows{r,3},b), sprintf(rows{r,3},c), d, mk);
    end
end

fprintf('\n== 结论: %s ==\n', ternary_(ok,'PASS (逐字节一致)','FAIL (见上方定位)'));
if ~ok
    fprintf('  若本次改动是「阶段 S 换求解器」, MD5 FAIL 属预期;\n');
    fprintf('  此时看指标表, 标 !! 的是安全类变差, 必须解释。\n');
end
if nargout == 0, clear ok; end
end

function h = md5_(f)
d = java.security.MessageDigest.getInstance('MD5');
fid = fopen(f,'r'); b = fread(fid,inf,'*uint8'); fclose(fid);
h = lower(reshape(dec2hex(typecast(d.digest(b),'uint8')).',1,[]));
end

function locate_(tb, to)
try
    B = func_ReadERD(tb); C = func_ReadERD(to);
catch e
    fprintf('        (无法读取做定位: %s)\n', e.message); return
end
if B.N ~= C.N
    fprintf('        拍数不同: 基准 %d -> 当前 %d  (先查 StopTime 和 CarSim 数据集!)\n', B.N, C.N);
end
n = min(B.N, C.N);
fn = intersect(fieldnames(B), fieldnames(C));
first = inf; who = '';
for i = 1:numel(fn)
    f = fn{i};
    if ~isnumeric(B.(f)) || numel(B.(f)) < n, continue; end
    d = find(B.(f)(1:n) ~= C.(f)(1:n), 1, 'first');
    if ~isempty(d) && d < first, first = d; who = f; end
end
if isfinite(first)
    fprintf('        首次偏离: 第 %d 拍 (t=%.2f s), 通道 %s\n', first, B.t(first), who);
else
    fprintf('        前 %d 拍所有通道一致, 差异只在尾部长度\n', n);
end
end

function s = ternary_(c,a,b)
if c, s = a; else, s = b; end
end

% -------------------------------------------------------------------------
function mdl = local_curmodel()
%  simfile.sim 当前指向哪个数据集 -> 那份 Run_all.par -> SIMULINK_MODEL_FILE
%  逐行找而不用正则: 这两个宏名里带 ) 和 $, 写成正则要一堆转义, 容易出错。
mdl = 'cq3_2019';
if exist('simfile.sim','file') ~= 2, return; end
L = strsplit(fileread('simfile.sim'), newline);
wd = '';  gid = '';
%  ⚠️ 宏名在文件里出现不止一次, 而且**其它宏的值里也会引用它**, 例如
%       SET_MACRO $(WORK_DIR)$ D:\...\CarSim2019.0_Data\            <- 定义
%       SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\...    <- 值里引用
%       INPUT $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\Run_all.par    <- 引用
%     所以光靠 strfind('WORK_DIR)$') 或"以 SET_MACRO 开头"都会抓错行。
%     正确做法: 把每行解析成 "SET_MACRO $(宏名)$ 值", 再比对**宏名**。
for i = 1:numel(L)
    t = strtrim(L{i});
    if ~strncmp(t, 'SET_MACRO', 9), continue; end
    r = strtrim(t(10:end));                     % "$(NAME)$ value"
    if ~strncmp(r, '$(', 2), continue; end
    e = strfind(r, ')$');
    if isempty(e), continue; end
    nm = r(3:e(1)-1);                           % 第一个 )$ 之前就是宏名
    v  = strtrim(r(e(1)+2:end));
    switch nm
        case 'WORK_DIR'
            wd = v;
        case 'ROOT_FILE_NAME'
            if strncmp(v,'Run_',4), gid = v(5:end); end
    end
end
if isempty(wd) || isempty(gid), return; end
mdl = func_SimModel(fullfile(wd,'Results',['Run_' gid],'Run_all.par'));
end
