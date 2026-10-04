function R = verify_pmpc_comb_isolated()
%VERIFY_PMPC_COMB_ISOLATED Run the current COMB PMPC dataset without LastRun.
root = startup_pmpc();
template = fileread(fullfile(root,'simfile.sim'));
runId = regexp(template,'ROOT_FILE_NAME\)\$\s+(Run_[^\s]+)','tokens','once');
work = regexp(template,'WORK_DIR\)\$\s+([^\r\n]+)','tokens','once');
assert(~isempty(runId) && ~isempty(work),'Invalid simfile.sim');
dataDir = strtrim(work{1});
expanded = fullfile(dataDir,'Results',runId{1},'Run_all.par');
assert(exist(expanded,'file')==2,'Send COMB PMPC to Simulink first.');
assert(contains(fileread(expanded),'COMB90_DLC05_JT_R70_mu0.5to0.85_PMPC'), ...
    'Current simfile.sim is not the COMB PMPC case.');
outDir = fullfile(root,'results_pmpc_comb_fix', ...
    ['mode3_simple_' datestr(now,'yyyymmdd_HHMMSS')]);
mkdir(outDir);
prefix = fullfile(outDir,'LastRun');
simfile = fullfile(outDir,'run.sim');
oldPath = regexp(template, ...
    '(?m)^SET_MACRO \$\(OUTPUT_PATH\)\$[^\r\n]*','match','once');
oldPrefix = regexp(template, ...
    '(?m)^SET_MACRO \$\(OUTPUT_FILE_PREFIX\)\$[^\r\n]*','match','once');
assert(~isempty(oldPath) && ~isempty(oldPrefix),'Missing output macros.');
t = strrep(template,oldPath,['SET_MACRO $(OUTPUT_PATH)$ ' outDir]);
t = strrep(t,oldPrefix,['SET_MACRO $(OUTPUT_FILE_PREFIX)$ ' prefix]);
fid = fopen(simfile,'w'); assert(fid>0,'Cannot write isolated simfile.');
fwrite(fid,t,'char'); fclose(fid);
original = fullfile(dataDir,'Results',runId{1},'LastRun');
before = local_fingerprint(original);
assignin('base','PMPC_SIMFILE',simfile);
assignin('base','PMPC_LONGCOORD',3);
assignin('base','PMPC_VERBOSE',0);
cleanup = onCleanup(@() local_cleanup()); %#ok<NASGU>
mil_init_PMPC();
load_system('pmpc_mil');
set_param('pmpc_mil/CarSim','SIMFILE',strrep(simfile,[root filesep],''));
stopTime = func_CarSimStopTime(fileread(expanded));
assert(isfinite(stopTime) && stopTime > 10,'Unexpected COMB stop time.');
set_param('pmpc_mil','StopTime',num2str(stopTime,'%.12g'));
simOut = sim('pmpc_mil');
func_WaitERD(outDir,0,180);
assert(isequal(before,local_fingerprint(original)), ...
    'Original CarSim LastRun was modified.');
local_repair_header(prefix);
D = func_ReadERD(prefix);
assert(strcmp(D.Dataset,'COMB90_DLC05_JT_R70_mu0.5to0.85_PMPC'), ...
    'Unexpected dataset result.');
R = func_EyConsoleSummary(D);
R.t_end = D.t(end);
R.outputPrefix = prefix;
longDiag = simOut.get('PMPC_LONG_DIAG'); %#ok<NASGU>
save(fullfile(outDir,'longitudinal_diagnostics.mat'),'longDiag');
save(fullfile(outDir,'summary.mat'),'R');
end

function f = local_fingerprint(prefix)
paths = {[prefix '.vs'],[prefix '.vsb'],[prefix '_log.txt']};
f = zeros(3,2);
for k = 1:3
    d = dir(paths{k});
    if ~isempty(d), f(k,:) = [d.bytes d.datenum]; end
end
end

function local_repair_header(prefix)
path = [prefix '.vs'];
raw = fileread(path);
try
    jsondecode(raw);
    return;
catch
end
bad = regexp(raw,'(?m)^[ \t]*"Input File"[^\r\n]*','match','once');
assert(~isempty(bad),'Malformed ERD header.');
clean = strrep(raw,bad,'    "Input File" : "<legacy path omitted>",');
jsondecode(clean);
copyfile(path,[path '.raw']);
fid = fopen(path,'w'); assert(fid>0,'Cannot repair ERD header.');
fwrite(fid,clean,'char'); fclose(fid);
end

function local_cleanup()
evalin('base','clear PMPC_SIMFILE PMPC_LONGCOORD PMPC_VERBOSE');
if bdIsLoaded('pmpc_mil'), close_system('pmpc_mil',0); end
end
