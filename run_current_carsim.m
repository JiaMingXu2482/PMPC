function info = run_current_carsim(simfile, model)
%RUN_CURRENT_CARSIM Run the current CarSim dataset with one fixed model.
%   In CarSim: edit Procedure speed -> Send to Simulink.
%   In MATLAB: run_current_carsim('mpc_mil'|'zeng_mil'|'pmpc_mil')
%   A relative .sim path can be supplied for an isolated verification run.

root = startup_pmpc();
validModels = {'mpc_mil','zeng_mil','pmpc_mil'};
if nargin < 1 || isempty(simfile)
    simfile = 'simfile.sim';
end
if nargin < 2 || isempty(model)
    if ischar(simfile) || isstring(simfile)
        candidate = char(simfile);
        if any(strcmp(candidate, validModels))
            model = candidate;
            simfile = 'simfile.sim';
        else
            model = 'pmpc_mil';
        end
    else
        model = 'pmpc_mil';
    end
end
if ~any(strcmp(model, validModels))
    error('run_current_carsim:InvalidModel', ...
        'model must be mpc_mil, zeng_mil, or pmpc_mil.');
end
if ~ischar(simfile) && ~isstring(simfile)
    error('run_current_carsim:BadSimfile', 'simfile 必须是文件路径。');
end
simfile = char(simfile);
if ~is_absolute_(simfile), simfile = fullfile(root, simfile); end
assert(exist(simfile, 'file') == 2, 'run_current_carsim:NoSimfile', ...
    '找不到 %s；请先在 CarSim 点击 Send to Simulink。', simfile);

sf = fileread(simfile);
runId = regexp(sf, 'ROOT_FILE_NAME\)\$\s+(Run_[^\s]+)', 'tokens', 'once');
wd = regexp(sf, 'WORK_DIR\)\$\s+([^\r\n]+)', 'tokens', 'once');
assert(~isempty(runId) && ~isempty(wd), 'run_current_carsim:BadSimfile', ...
    'simfile 缺少 ROOT_FILE_NAME 或 WORK_DIR。');
dataDir = strtrim(wd{1});
src = fullfile(dataDir, 'Runs', [runId{1} '.par']);
resdir = fullfile(dataDir, 'Results', runId{1});
expanded = fullfile(resdir, 'Run_all.par');
assert(exist(src, 'file') == 2 && exist(expanded, 'file') == 2, ...
    'run_current_carsim:NotSent', ...
    '当前数据集尚无完整展开结果；请在 CarSim 对该 Run 点击 Send to Simulink。');

runText = fileread(src);
allText = fileread(expanded);
sourceName = local_name_(runText);
expandedName = local_name_(allText);
assert(strcmp(sourceName, expandedName), 'run_current_carsim:StaleName', ...
    'CarSim 数据集名称已改，但展开结果仍是 %s；请重新 Send to Simulink。', expandedName);
procLink = regexp(runText, '(?m)^PARSFILE\s+(Procedures\\[^\r\n]+)', 'tokens', 'once');
assert(~isempty(procLink), 'run_current_carsim:NoProcedure', ...
    '当前 Run 没有 Procedure 链接，无法核对车速。');
proc = fullfile(dataDir, strrep(procLink{1}, '\', filesep));
assert(exist(proc, 'file') == 2, 'run_current_carsim:NoProcedure', ...
    '找不到 Procedure：%s', proc);
vProc = local_speed_(fileread(proc));
vRun = local_speed_(runText);
vAll = local_speed_(allText);
assert(~isempty(vProc), 'run_current_carsim:NoSpeed', ...
    'Procedure 没有 SV_VXS 初速度设置。');
if isempty(vRun), requested = vProc(end); else, requested = vRun(end); end
assert(~isempty(vAll) && abs(vAll(end) - requested) < 1e-9, ...
    'run_current_carsim:StaleSpeed', ...
    'CarSim 当前设定 %.3f km/h，但 Send 后输入仍是 %.3f km/h；请重新 Send to Simulink。', ...
    requested, vAll(end));

func_SimModel(expanded, model);
assert(func_CarSimRunning(), 'run_current_carsim:NoCarSim', ...
    '请保持 CarSim Browser 运行。');
func_CarSimLib();
evalin('base', 'clear PMPC_SIMFILE PMPC_MODE PMPC_ZENGRHO PMPC_P MPC_P ZENG_P');
assignin('base', 'PMPC_SIMFILE', simfile);
clearOverride = onCleanup(@() evalin('base', 'clear PMPC_SIMFILE')); %#ok<NASGU>
switch model
    case 'mpc_mil',  mil_init_MPC;
    case 'zeng_mil', mil_init_ZENG;
    case 'pmpc_mil', mil_init_PMPC;
end

load_system(model);
modelSimfile = strrep(simfile, [root filesep], '');
set_param([model '/CarSim'], 'SIMFILE', modelSimfile);
logfile = fullfile(resdir, 'LastRun_log.txt');
oldLog = dir(logfile);
if isempty(oldLog), logstamp = 0; else, logstamp = oldLog.datenum; end
fprintf('当前 CarSim Run：%s，初速度 %.3f km/h (%.3f m/s)。\n', ...
    sourceName, requested, requested/3.6);
sim(model);
func_WaitERD(resdir, logstamp);
D = func_ReadERD(fullfile(resdir, 'LastRun'));
assert(strcmp(D.Dataset, sourceName) && abs(D.Vx(1) - requested) < 0.05, ...
    'run_current_carsim:WrongResult', ...
    '结果标签或实际初速度不匹配：%s / %.3f km/h。', D.Dataset, D.Vx(1));
info = struct('name', sourceName, 'speed_kmh', requested, ...
              'resdir', resdir, 't_end', D.t(end), 'model', model);
fprintf('CarSim 结果已生成：%s (实际初速度 %.3f km/h)。\n', ...
    resdir, D.Vx(1));
end

function name = local_name_(text)
t = regexp(text, 'FullDataName CarSim Run Control`([^`]+)`', 'tokens', 'once');
assert(~isempty(t), 'run_current_carsim:NoDatasetName', '找不到 CarSim Run 名称。');
name = t{1};
end

function v = local_speed_(text)
t = regexp(text, '(?m)^SV_VXS\s+([-+]?\d+(?:\.\d+)?)\s*$', 'tokens');
v = cellfun(@(x) str2double(x{1}), t);
end

function yes = is_absolute_(p)
yes = ~isempty(regexp(p, '^[A-Za-z]:[\\/]', 'once')) || strncmp(p, '\\', 2);
end
