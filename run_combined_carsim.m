function report = run_combined_carsim(model,R)
%RUN_COMBINED_CARSIM Build and run a 90 km/h DLC + J-turn comparison case.
% Source CarSim datasets and the selected GUI run are left untouched.
if nargin < 1 || isempty(model), model = 'pmpc_mil'; end
if nargin < 2 || isempty(R), R = 70; end
root = startup_pmpc();
S = build_combined_carsim(model,R);
assert(func_CarSimRunning(),'run_combined_carsim:NoCarSim', ...
    'Open CarSim Browser for the solver license before running this case.');
func_CarSimLib();
priorSimfile = local_base_value('PMPC_SIMFILE');
priorMode = local_base_value('PMPC_LONGCOORD');
priorVerbose = local_base_value('PMPC_VERBOSE');
cleanup = onCleanup(@() local_cleanup(model,priorSimfile,priorMode,priorVerbose)); %#ok<NASGU>
assignin('base','PMPC_SIMFILE',S.simfile);
assignin('base','PMPC_LONGCOORD',double(strcmp(model,'pmpc_mil')));
assignin('base','PMPC_VERBOSE',0);
switch model
    case 'mpc_mil', mil_init_MPC();
    case 'zeng_mil', mil_init_ZENG();
    case 'pmpc_mil', mil_init_PMPC();
end
load_system(model);
set_param([model '/CarSim'],'SIMFILE',erase(S.simfile,[root filesep]));
set_param(model,'StopTime','50');
fprintf('Running %s: DLC 90 km/h, turn target 80 km/h, station-switched mu.\n',S.name);
simOut = sim(model);
runDir = fileparts(S.outputPrefix);
func_WaitERD(runDir,0,180);
local_repair_erd_header(S.outputPrefix);
D = func_ReadERD(S.outputPrefix);
assert(strcmp(D.Dataset,S.name) && abs(D.Vx(1)-90)<0.1, ...
    'run_combined_carsim:WrongOutput', ...
    'Wrong CarSim result: %s, initial %.3f km/h.',D.Dataset,D.Vx(1));
assert(D.Station(end) > S.course.turn_x+50, ...
    'run_combined_carsim:ShortRun', ...
    'Simulation ended before the main J-turn section.');
W = S.waypoints;
dlcEndS = W(find(W(:,2)>=S.course.dlc_end_x,1,'first'),7);
turnS = W(find(W(:,2)>=S.turn_x,1,'first'),7);
sections = [0,dlcEndS;dlcEndS,turnS;turnS,W(end,7)];
labels = {'DLC','recovery','J-turn'};
report = struct('name',S.name,'initial_kmh',D.Vx(1), ...
    'model',model, ...
    'switch_station_m',S.mu_switch_station,'turn_station_m',turnS, ...
    'final_station_m',D.Station(end),'mu_dlc',S.course.mu_dlc, ...
    'mu_jturn',S.course.mu_jturn,'segments',[], ...
    'outputPrefix',S.outputPrefix);
segments = repmat(struct('name','','ey_peak_m',NaN,'ey_rms_m',NaN, ...
    'corner_out_pct',NaN,'ltr_peak',NaN,'min_kmh',NaN,'peak_kmh',NaN, ...
    'entry_kmh',NaN,'vx_rms_kmh',NaN),3,1);
for j=1:3
    if j==3
        ix = find(D.Station>=sections(j,1) & D.Station<=sections(j,2));
    else
        ix = find(D.Station>=sections(j,1) & D.Station<sections(j,2));
    end
    if numel(ix)<2, continue; end
    ds = local_slice(D,ix);
    mu = S.course.mu_dlc;
    if j==3, mu = S.course.mu_jturn; end
    M = func_Metrics(ds,struct('mu',mu));
    segments(j) = struct('name',labels{j},'ey_peak_m',M.ey_pk, ...
        'ey_rms_m',M.ey_rms,'corner_out_pct',M.out_pct, ...
        'ltr_peak',M.ltr_pk,'min_kmh',min(ds.Vx), ...
        'peak_kmh',max(ds.Vx), ...
        'entry_kmh',ds.Vx(1),'vx_rms_kmh',sqrt(mean(ds.Vx.^2)));
    if j~=2
        fprintf('%-8s e_y max %.6f m | e_y RMS %.6f m | Vx RMS %.3f km/h\n', ...
            labels{j},M.ey_pk,M.ey_rms,segments(j).vx_rms_kmh);
    end
end
report.segments = segments;
if any(strcmp(simOut.who,'PMPC_LONG_DIAG'))
    trace = simOut.get('PMPC_LONG_DIAG');
    report.long_diag_time = trace.time;
    report.long_diag = trace.signals.values;
end
save(S.reportFile,'report');
end

function E = local_slice(D,indices)
E = D;
fields = fieldnames(D);
for j=1:numel(fields)
    v = D.(fields{j});
    if isnumeric(v) && isvector(v) && numel(v)==D.N
        E.(fields{j}) = v(indices);
    end
end
E.N = numel(indices);
end

function result = local_base_value(name)
result = struct('exists',false,'value',[]);
if evalin('base',['exist(''' name ''',''var'')'])
    result.exists = true;
    result.value = evalin('base',name);
end
end

function local_cleanup(model,priorSimfile,priorMode,priorVerbose)
if bdIsLoaded(model), close_system(model,0); end
local_restore('PMPC_SIMFILE',priorSimfile);
local_restore('PMPC_LONGCOORD',priorMode);
local_restore('PMPC_VERBOSE',priorVerbose);
end

function local_restore(name,prior)
if prior.exists
    assignin('base',name,prior.value);
else
    evalin('base',['clear ' name]);
end
end

function local_repair_erd_header(prefix)
path = [prefix '.vs'];
raw = fileread(path);
try
    jsondecode(raw);
    return;
catch
end
badLine = regexp(raw,'(?m)^[ \t]*"Input File"[^\r\n]*', ...
    'match','once');
assert(~isempty(badLine),'run_combined_carsim:BadERD','%s',path);
clean = strrep(raw,badLine,'    "Input File" : "<legacy path omitted>",');
jsondecode(clean);
copyfile(path,[path '.raw']);
f = fopen(path,'w');
assert(f>=0,'run_combined_carsim:HeaderWriteFailed','%s',path);
fwrite(f,clean,'char'); fclose(f);
end
