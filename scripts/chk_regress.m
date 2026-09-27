function ok = chk_regress(varargin)
%CHK_REGRESS  回归验收: 跑三个控制器, 与基准比对
%   chk_regress                 跑仿真后比对
%   chk_regress('-nosim')       跳过仿真, 直接比对 erd_out 里已有的结果
%   chk_regress('-cur',DIR)     比对指定目录的结果(隐含 -nosim)
%   chk_regress('-base',DIR)    指定基准目录 (默认当前 erd_0927_base)
%   其余选项(如 '-force')原样透传给 run3b -> run_ds
%   ok = chk_regress(...)       返回 true/false
%
%   判据: 三份 ERD 的 MD5 与当前基准逐字节对比，同时输出安全和性能指标。
%   MD5 不一致时会定位到: 哪个控制器 / 首次偏离在第几拍 / 哪个通道。
root = func_ProjectRoot();
base = fullfile(root,'simulation_results','current','erd_0927_base');
dosim = true;
cur = '';
pass = {};      % 不认识的选项透传给 run3b/run_ds (如 '-force')
i = 1;
while i <= numel(varargin)
    switch lower(varargin{i})
        case '-nosim', dosim = false;
        case '-base',  base = varargin{i+1}; i = i+1;
        case '-cur',   cur  = varargin{i+1}; i = i+1; dosim = false;
        otherwise,     pass{end+1} = varargin{i}; %#ok<AGROW>
    end
    i = i + 1;
end
if ~isempty(cur), out = cur; else, out = func_ErdDir(); end

fprintf('  模型 pmpc_mil -> 基准 %s\n', base);
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
