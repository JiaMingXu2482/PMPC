function report = compare_pmpc_nodelay_jturn(mode)
%COMPARE_PMPC_NODELAY_JTURN Isolated 80 km/h R69 PMPC delay ablation.
%   compare_pmpc_nodelay_jturn('preflight') checks the source without writes.
%   compare_pmpc_nodelay_jturn() runs original PMPC and PMPC-noDelay against
%   one identical CarSim input, keeping the CarSim Results directory intact.
if nargin < 1, mode = 'run'; end
root = startup_pmpc();
dataDir = 'D:\Program Files\CarSim2019.0\CarSim2019.0_Data';
runId = 'Run_4cc2371c-6336-4208-b9d5-c6d20d1bd3f5';
source = fullfile(dataDir,'Results',runId,'Run_all.par');
assert(exist(source,'file')==2, ...
    'compare_pmpc_nodelay_jturn:MissingSource', ...
    'Send JT80_R69_mu0.85_PMPC to Simulink first.');
expanded = fileread(source);
assert(contains(expanded,'FullDataName CarSim Run Control`JT80_R69_mu0.85_PMPC`'), ...
    'compare_pmpc_nodelay_jturn:WrongDataset','Source is not the R69 J-turn.');
assert(abs(localLastNumber(expanded,'MU_ROAD_CONSTANT')-0.85)<1e-9, ...
    'compare_pmpc_nodelay_jturn:WrongMu','J-turn road mu is not 0.85.');
input80 = func_ReplaceLastCarSimParameter(expanded,'SV_VXS','80');
assert(abs(localLastNumber(input80,'SV_VXS')-80)<1e-9, ...
    'compare_pmpc_nodelay_jturn:WrongSpeed','Isolated input must be 80 km/h.');
report = struct('source_run',runId, ...
    'source_speed_kmh',localLastNumber(expanded,'SV_VXS'), ...
    'test_speed_kmh',80,'mu',0.85,'radius_m',69, ...
    'models',{{'pmpc_mil','pmpc_nodelay_mil'}});
if strcmpi(mode,'preflight'), return; end
assert(strcmpi(mode,'run'),'compare_pmpc_nodelay_jturn:BadMode', ...
    'Mode must be preflight or run.');
assert(func_CarSimRunning(), 'compare_pmpc_nodelay_jturn:NoCarSim', ...
    'Keep the CarSim Browser running.');
func_CarSimLib();

outRoot = fullfile(root,'results_pmpc_nodelay_jturn', ...
    datestr(now,'yyyymmdd_HHMMSS'));
mkdir(outRoot);
inputPath = fullfile(outRoot,'Run_all_80.par');
localWriteText(inputPath,input80);
template = fileread(fullfile(root,'simfile.sim'));
WayPoints_Collect = func_WayPoints(5,69,false); %#ok<NASGU>
waypointPath = fullfile(outRoot,'WayPoints_JTurn_R69.mat');
save(waypointPath,'WayPoints_Collect');
models = report.models;
results = repmat(struct('model','','ey_peak_m',NaN,'ey_rms_m',NaN, ...
    'vx_rms_kmh',NaN,'min_kmh',NaN,'ltr_peak',NaN,'t_end_s',NaN, ...
    'output_prefix',''),1,numel(models));
cleanup = onCleanup(@() localCleanup(models)); %#ok<NASGU>
for k = 1:numel(models)
    model = models{k};
    outDir = fullfile(outRoot,model);
    mkdir(outDir);
    prefix = fullfile(outDir,'LastRun');
    simfile = fullfile(outDir,'run.sim');
    localWriteSimfile(template,simfile,runId,inputPath,prefix);
    assignin('base','PMPC_SIMFILE',simfile);
    assignin('base','PMPC_LONGCOORD',1);
    assignin('base','PMPC_VERBOSE',0);
    assignin('base','PMPC_NLCSNN',1);
    assignin('base','PMPC_ABL',0);
    if strcmp(model,'pmpc_mil')
        P = mil_init_PMPC();
    else
        P = mil_init_PMPCnoDelay();
    end
    assert(P.S0.Constraints.PrioModeRT==1 && ...
        P.S0.Constraints.LongCoordMode==1 && P.Pm.NLCSNN.enabled, ...
        'compare_pmpc_nodelay_jturn:WrongController', ...
        '%s must retain PMPC priorities, speed coordination, and NLCSNN.',model);
    load_system(model);
    relativeSimfile = erase(simfile,[root filesep]);
    set_param([model '/CarSim'],'SIMFILE',relativeSimfile);
    stopTime = func_CarSimStopTime(input80);
    if isfinite(stopTime) && stopTime>0
        set_param(model,'StopTime',num2str(stopTime,'%.12g'));
    end
    sim(model);
    func_WaitERD(outDir,0,180);
    localRepairVs(prefix);
    D = func_ReadERD(prefix);
    assert(strcmp(D.Dataset,'JT80_R69_mu0.85_PMPC') && abs(D.Vx(1)-80)<0.05, ...
        'compare_pmpc_nodelay_jturn:WrongResult', ...
        '%s produced %s at %.3f km/h.',model,D.Dataset,D.Vx(1));
    metrics = func_Metrics(D,struct('wp',waypointPath));
    results(k) = struct('model',model, ...
        'ey_peak_m',metrics.ey_pk,'ey_rms_m',metrics.ey_rms, ...
        'vx_rms_kmh',sqrt(mean(D.Vx.^2)),'min_kmh',min(D.Vx), ...
        'ltr_peak',metrics.ltr_pk,'t_end_s',D.t(end), ...
        'output_prefix',prefix);
    fprintf('%s: e_y max %.6f m | RMS %.6f m | Vx RMS %.3f km/h | LTR max %.3f\n', ...
        model,results(k).ey_peak_m,results(k).ey_rms_m, ...
        results(k).vx_rms_kmh,results(k).ltr_peak);
    close_system(model,0);
end
report.results = results;
report.output_dir = outRoot;
save(fullfile(outRoot,'report.mat'),'report');
end

function value = localLastNumber(input,key)
matches = regexp(input,['(?m)^' key '\s+([-+]?\d+(?:\.\d+)?)\s*$'], ...
    'tokens');
assert(~isempty(matches),'compare_pmpc_nodelay_jturn:MissingParameter',key);
value = str2double(matches{end}{1});
end

function localWriteSimfile(template,path,runId,inputPath,prefix)
text = localReplaceMacro(template,'ROOT_FILE_NAME',runId);
text = localReplaceMacro(text,'OUTPUT_PATH',fileparts(prefix));
text = localReplaceMacro(text,'OUTPUT_FILE_PREFIX',prefix);
oldInput = regexp(text,'(?m)^INPUT\s+[^\r\n]*','match','once');
assert(~isempty(oldInput),'compare_pmpc_nodelay_jturn:MissingInput', ...
    'simfile.sim has no INPUT entry.');
text = strrep(text,oldInput,['INPUT ' inputPath]);
localWriteText(path,text);
end

function text = localReplaceMacro(text,key,value)
old = regexp(text,['(?m)^SET_MACRO \$\(' key '\)\$[^\r\n]*'], ...
    'match','once');
assert(~isempty(old),'compare_pmpc_nodelay_jturn:MissingMacro',key);
text = strrep(text,old,['SET_MACRO $(' key ')$ ' value]);
end

function localWriteText(path,value)
fid = fopen(path,'w');
assert(fid>=0,'compare_pmpc_nodelay_jturn:CannotWrite','%s',path);
guard = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid,value,'char');
end

function localRepairVs(prefix)
path = [prefix '.vs'];
raw = fileread(path);
try
    jsondecode(raw);
    return;
catch
end
bad = regexp(raw,'(?m)^[ \t]*"Input File"[^\r\n]*','match','once');
assert(~isempty(bad),'compare_pmpc_nodelay_jturn:BadERD', ...
    'Malformed ERD header: %s',path);
clean = strrep(raw,bad,'    "Input File" : "<legacy path omitted>",');
jsondecode(clean);
copyfile(path,[path '.raw']);
localWriteText(path,clean);
end

function localCleanup(models)
evalin('base','clear PMPC_SIMFILE PMPC_LONGCOORD PMPC_VERBOSE PMPC_NLCSNN PMPC_ABL');
for k = 1:numel(models)
    if bdIsLoaded(models{k}), close_system(models{k},0); end
end
end
