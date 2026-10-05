function report = compare_pmpc_longcoord(mode)
%COMPARE_PMPC_LONGCOORD Compare 80 km/h DLC and R69 without touching LastRun.
%   compare_pmpc_longcoord('preflight') validates all eight input cases.
%   compare_pmpc_longcoord() executes them and saves isolated ERD/metrics in
%   simulation_results/archive/results_pmpc_longcoord (ignored by Git).
%   CarSim Browser must be open.
if nargin < 1, mode = 'run'; end
root = startup_pmpc();
template = fileread(fullfile(root,'simfile.sim'));
work = regexp(template,'WORK_DIR\)\$\s+([^\r\n]+)','tokens','once');
assert(~isempty(work),'compare_pmpc_longcoord:BadSimfile', ...
    'simfile.sim has no WORK_DIR.');
dataDir = strtrim(work{1});
outRoot = fullfile(root,'simulation_results','archive','results_pmpc_longcoord');
if strcmpi(mode,'focused') || strcmpi(mode,'references')
    outRoot = fullfile(root,'simulation_results','archive','results_pmpc_longcoord_v2');
end
labels = {'DLC80_mu0.5_PMPC','JT80_R69_mu0.85_PMPC'};
zengLabels = {'DLC80_mu0.5_ZENG','JT80_R69_mu0.85_ZENG'};
cases = repmat(struct('name','','tag','','model','','mode',0, ...
    'runId','','expandedPar','','outputPrefix','','simfile','', ...
    'initialKmh',0,'expandedKmh',0,'mu',0),8,1);
n = 0;
for maneuver = 1:2
    for variant = 1:4
        n = n+1;
        tags = {'OFF','ON','CURVATURE','ZENG'};
        tag = tags{variant};
        if variant == 4
            name = zengLabels{maneuver}; model = 'zeng_mil'; coordMode = 0;
        else
            name = labels{maneuver}; model = 'pmpc_mil'; coordMode = variant-1;
        end
        runFile = local_find_run(dataDir,name);
        [~,runId] = fileparts(runFile);
        expanded = fullfile(dataDir,'Results',runId,'Run_all.par');
        assert(exist(expanded,'file')==2, ...
            'compare_pmpc_longcoord:NotSent', ...
            '%s must be sent to Simulink from CarSim first.',name);
        runText = fileread(runFile);
        allText = fileread(expanded);
        expandedName = local_name(allText);
        assert(strcmp(name,expandedName), ...
            'compare_pmpc_longcoord:StaleRun','%s expanded as %s.',name,expandedName);
        procedure = regexp(runText, ...
            '(?m)^PARSFILE\s+(Procedures\\[^\r\n]+)','tokens','once');
        assert(~isempty(procedure),'compare_pmpc_longcoord:NoProcedure',name);
        procPath = fullfile(dataDir,strrep(procedure{1},'\',filesep));
        assert(exist(procPath,'file')==2, ...
            'compare_pmpc_longcoord:NoProcedure','%s',procPath);
        requested = local_number(fileread(procPath),'SV_VXS');
        runSpeed = local_number(runText,'SV_VXS');
        if isfinite(runSpeed), requested = runSpeed; end
        expandedSpeed = local_number(allText,'SV_VXS');
        assert(abs(requested-80)<1e-9 && abs(expandedSpeed-80)<1e-9, ...
            'compare_pmpc_longcoord:WrongSpeed', ...
            '%s: procedure/run %.3f, expanded %.3f km/h.', ...
            name,requested,expandedSpeed);
        expectedMu = 0.5;
        if maneuver == 2, expectedMu = 0.85; end
        expandedMu = local_number(allText,'MU_ROAD_CONSTANT');
        assert(abs(expandedMu-expectedMu)<1e-9, ...
            'compare_pmpc_longcoord:WrongMu','%s mu %.3f.',name,expandedMu);
        func_SimModel(expanded,model);
        runOut = fullfile(outRoot, ...
            sprintf('%s_%s',name,tag));
        prefix = fullfile(runOut,'LastRun');
        assert(~startsWith(lower(prefix), ...
            lower(fullfile(dataDir,'Results'))), ...
            'compare_pmpc_longcoord:NotIsolated',name);
        cases(n) = struct('name',name,'tag',tag,'model',model, ...
            'mode',coordMode,'runId',runId,'expandedPar',expanded, ...
            'outputPrefix',prefix,'simfile',fullfile(runOut,'run.sim'), ...
            'initialKmh',requested,'expandedKmh',expandedSpeed, ...
            'mu',expectedMu);
    end
end
if strcmpi(mode,'preflight'), report = cases; return; end
assert(strcmpi(mode,'run') || strcmpi(mode,'focused') ...
    || strcmpi(mode,'references'), ...
    'compare_pmpc_longcoord:BadMode', ...
    'Mode must be run, focused, references or preflight.');
if strcmpi(mode,'focused')
    cases = cases(strcmp({cases.tag},'ON'));
elseif strcmpi(mode,'references')
    cases = cases((contains({cases.name},'DLC') & strcmp({cases.tag},'ZENG')) ...
        | (contains({cases.name},'JT80') ...
        & (strcmp({cases.tag},'OFF') | strcmp({cases.tag},'ZENG'))));
end
assert(func_CarSimRunning(),'compare_pmpc_longcoord:NoCarSim', ...
    'Open CarSim Browser before running the comparison.');
func_CarSimLib();
if ~exist(outRoot,'dir'), mkdir(outRoot); end
% Keep metric references with this isolated comparison. setup_pmpc does
% not save generated waypoints, so a fresh checkout has no J-turn MAT.
dlcWaypoints = fullfile(outRoot,'WayPoints_DLC.mat');
jturnWaypoints = fullfile(outRoot,'WayPoints_JTurn_R69.mat');
WayPoints_Collect = func_WayPoints(1,69,false); %#ok<NASGU>
save(dlcWaypoints,'WayPoints_Collect');
WayPoints_Collect = func_WayPoints(5,69,false); %#ok<NASGU>
save(jturnWaypoints,'WayPoints_Collect');
cleanup = onCleanup(@() local_cleanup()); %#ok<NASGU>
report = repmat(struct('name','','tag','','ey_peak_m',NaN, ...
    'ey_rms_m',NaN,'corner_out_pct',NaN,'min_kmh',NaN, ...
    'brake_onset_s',NaN,'brake_peak_N',NaN,'brake_slew_Nps',NaN, ...
    'ltr_peak',NaN,'speed_mismatch_rms_mps',NaN, ...
    'longitudinal_onset_s',NaN,'coordinator_fault_pct',NaN, ...
    't_end_s',NaN,'common_station_m',200, ...
    'diag_available',false,'outputPrefix',''),numel(cases),1);
for k=1:numel(cases)
    c = cases(k);
    runOut = fileparts(c.outputPrefix);
    if ~exist(runOut,'dir'), mkdir(runOut); end
    local_write_simfile(template,c.simfile,c.runId,c.outputPrefix);
    assignin('base','PMPC_SIMFILE',c.simfile);
    assignin('base','PMPC_LONGCOORD',c.mode);
    assignin('base','PMPC_VERBOSE',0);
    if strcmp(c.model,'pmpc_mil')
        mil_init_PMPC();
    else
        mil_init_ZENG();
    end
    load_system(c.model);
    simfileRelative = erase(c.simfile,[root filesep]);
    set_param([c.model '/CarSim'],'SIMFILE',simfileRelative);
    oldFiles = local_result_fingerprints(c.runId,dataDir);
    simOut = [];
    simWall = 0;
    logPath = [c.outputPrefix '_log.txt'];
    complete = exist([c.outputPrefix '.vsb'],'file')==2 ...
        && exist(logPath,'file')==2 ...
        && contains(fileread(logPath),'Run stopped at t =');
    if ~complete
        tic;
        simOut = sim(c.model);
        simWall = toc;
        func_WaitERD(runOut,0,180);
        assert(isequal(oldFiles,local_result_fingerprints(c.runId,dataDir)), ...
            'compare_pmpc_longcoord:OverwroteUserResult', ...
            'Original CarSim LastRun changed during %s %s.',c.name,c.tag);
    end
    local_repair_erd_header(c.outputPrefix);
    D = func_ReadERD(c.outputPrefix);
    assert(strcmp(D.Dataset,c.name) && abs(D.Vx(1)-80)<0.05, ...
        'compare_pmpc_longcoord:WrongResult', ...
        '%s %s yielded %s / %.3f km/h.',c.name,c.tag,D.Dataset,D.Vx(1));
    D = local_to_common_station(D,200);
    waypointPath = dlcWaypoints;
    if contains(c.name,'JT80'), waypointPath = jturnWaypoints; end
    M = func_Metrics(D,struct('wp',waypointPath));
    r = report(k);
    r.name = c.name; r.tag = c.tag;
    r.ey_peak_m = M.ey_pk; r.ey_rms_m = M.ey_rms;
    r.corner_out_pct = M.out_pct;
    r.min_kmh = min(D.Vx);
    r.ltr_peak = M.ltr_pk;
    r.t_end_s = D.t(end);
    r.outputPrefix = c.outputPrefix;
    dt = D.t(2)-D.t(1);
    Tb = -(D.My_Bk_L1+D.My_Bk_R1+D.My_Bk_L2+D.My_Bk_R2);
    active = find(Tb>50,1,'first');  % CarSim records a 2 N*m zero-command offset
    if ~isempty(active), r.brake_onset_s = D.t(active); end
    r.brake_peak_N = max(Tb)/0.347;
    r.brake_slew_Nps = max(abs(diff(Tb)))/0.347/dt;
    r.diag_available = false;
    if strcmp(c.model,'pmpc_mil')
        try
            diag = [];
            stored = fullfile(runOut,'longitudinal_diagnostics.mat');
            if ~isempty(simOut)
                diag = simOut.get('PMPC_LONG_DIAG');
            elseif exist(stored,'file')==2
                saved = load(stored,'diag');
                diag = saved.diag;
            end
            if ~isempty(diag)
                r.diag_available = true;
                v = diag.signals.values;
                r.speed_mismatch_rms_mps = sqrt(mean(v(:,12).^2));
                firstLong = find(v(:,9)>1,1,'first');
                if ~isempty(firstLong)
                    r.longitudinal_onset_s = diag.time(firstLong);
                end
                r.coordinator_fault_pct = 100*mean(v(:,8)>0);
                if ~isempty(simOut), save(stored,'diag'); end
            end
        catch
            % ERD-based metrics remain valid if the optional logger is absent.
        end
    end
    report(k) = r;
    fprintf('%s %-9s: e_y peak %.3f m, RMS %.3f m, Vmin %.2f km/h, wall %.1f s\n', ...
        c.name,c.tag,r.ey_peak_m,r.ey_rms_m,r.min_kmh,simWall);
    save(fullfile(outRoot,['report_' lower(mode) '.mat']), ...
        'report','cases');
    close_system(c.model,0);
end

function E = local_to_common_station(D,station)
assert(isfield(D,'Station'), ...
    'compare_pmpc_longcoord:NoStation','ERD lacks Station.');
i = find(D.Station<=station,1,'last');
assert(~isempty(i) && i>1, ...
    'compare_pmpc_longcoord:ShortRun','No valid 0–%.0f m interval.',station);
E = D;
fields = fieldnames(D);
for fieldIdx=1:numel(fields)
    value = D.(fields{fieldIdx});
    if isnumeric(value) && isvector(value) && numel(value)==D.N
        E.(fields{fieldIdx}) = value(1:i);
    end
end
E.N = i;
end

function local_repair_erd_header(prefix)
% CarSim 2019 emits malformed JSON in "Input File" for a Chinese path.
% Keep the raw header next to the corrected one for auditability.
path = [prefix '.vs'];
raw = fileread(path);
try
    jsondecode(raw);
    return;
catch
end
badLine = regexp(raw,'(?m)^[ \t]*"Input File"[^\r\n]*', ...
    'match','once');
assert(~isempty(badLine), ...
    'compare_pmpc_longcoord:BadERD','Malformed ERD header: %s',path);
clean = strrep(raw,badLine,'    "Input File" : "<legacy path omitted>",');
jsondecode(clean);
copyfile(path,[path '.raw']);
fid = fopen(path,'w');
assert(fid>=0,'compare_pmpc_longcoord:HeaderWriteFailed','%s',path);
fwrite(fid,clean,'char'); fclose(fid);
end
end

function path = local_find_run(dataDir,name)
files = dir(fullfile(dataDir,'Runs','Run_*.par'));
path = '';
for k=1:numel(files)
    candidate = fullfile(files(k).folder,files(k).name);
    fid = fopen(candidate,'r');
    if fid<0, continue; end
    head = fread(fid,500,'*char').'; fclose(fid);
    if strcmp(local_name(head),name)
        path = candidate; return;
    end
end
assert(~isempty(path),'compare_pmpc_longcoord:NoRun', ...
    'No CarSim Run named %s.',name);
end

function name = local_name(t)
v = regexp(t,'FullDataName CarSim Run Control`([^`]+)`', ...
    'tokens','once');
name = '';
if ~isempty(v), name = v{1}; end
end

function value = local_number(t,key)
v = regexp(t,['(?m)^' key '\s+([-+]?\d+(?:\.\d+)?)\s*$'], ...
    'tokens');
value = NaN;
if ~isempty(v), value = str2double(v{end}{1}); end
end

function local_write_simfile(template,path,runId,prefix)
outDir = fileparts(prefix);
t = local_replace_macro(template,'ROOT_FILE_NAME',runId);
t = local_replace_macro(t,'OUTPUT_PATH',outDir);
t = local_replace_macro(t,'OUTPUT_FILE_PREFIX',prefix);
assert(contains(t,['SET_MACRO $(OUTPUT_FILE_PREFIX)$ ' prefix]), ...
    'compare_pmpc_longcoord:BadOutputPath', ...
    'Generated simfile output prefix differs from the requested path.');
fid = fopen(path,'w');
assert(fid>=0,'compare_pmpc_longcoord:WriteFailed','%s',path);
fwrite(fid,t,'char'); fclose(fid);
end

function t = local_replace_macro(t,key,value)
pattern = ['(?m)^SET_MACRO \$\(' key '\)\$[^\r\n]*'];
old = regexp(t,pattern,'match','once');
assert(~isempty(old),'compare_pmpc_longcoord:MissingMacro','%s',key);
t = strrep(t,old,['SET_MACRO $(' key ')$ ' value]);
end

function f = local_result_fingerprints(runId,dataDir)
base = fullfile(dataDir,'Results',runId,'LastRun');
names = {[base '.vs'],[base '.vsb'],[base '_log.txt']};
f = zeros(numel(names),2);
for k=1:numel(names)
    d = dir(names{k});
    if ~isempty(d), f(k,:) = [d.bytes,d.datenum]; end
end
end

function local_cleanup()
evalin('base','clear PMPC_SIMFILE PMPC_LONGCOORD PMPC_VERBOSE');
for model = {'pmpc_mil','zeng_mil'}
    if bdIsLoaded(model{1}), close_system(model{1},0); end
end
end
