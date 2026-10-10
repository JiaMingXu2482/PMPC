function captureDir = run_com_capture(captureDir)
%RUN_COM_CAPTURE Re-run the currently sent COM road with three controllers.
% The current CarSim input and controller tuning/model files are read only.
% Each controller receives an isolated copy of the same expanded road input.

root = startup_pmpc();
sourceSim = fullfile(root, 'simfile.sim');
assert(isfile(sourceSim), 'Send the COM run to Simulink first.');
sourceText = fileread(sourceSim);
dataDir = macro_(sourceText, 'WORK_DIR');
sourceId = macro_(sourceText, 'ROOT_FILE_NAME');
sourcePar = fullfile(dataDir, 'Results', sourceId, 'Run_all.par');
assert(isfile(sourcePar), 'The sent CarSim Run_all.par is missing.');
par = fileread(sourcePar);
nameToken = regexp(par, 'FullDataName CarSim Run Control`([^`]+)`', 'tokens', 'once');
assert(~isempty(nameToken) && startsWith(nameToken{1}, 'COMB'), ...
    'The current CarSim run is not a COM case.');
assert(~isempty(regexp(par, '(?m)^SIMULINK_MODEL_FILE[ \t]+[^\r\n]+', 'once')), ...
    'The sent CarSim input has no Simulink model binding.');
assert(func_CarSimRunning(), 'Keep CarSim Browser open for the solver license.');
func_CarSimLib();

if nargin < 1 || isempty(captureDir)
    captureDir = fullfile(root, 'simulation_results', 'combined', ...
        ['COM_capture_' datestr(now, 'yyyymmdd_HHMMSS')]);
    assert(~isfolder(captureDir), 'Capture folder already exists.');
    mkdir(captureDir);
else
    assert(isfolder(captureDir), 'Capture folder does not exist.');
end
models = {'mpc_mil', 'zeng_mil', 'pmpc_mil'};
manifest = struct('sourceSimfile', sourceSim, 'sourceParfile', sourcePar, ...
    'sourceName', nameToken{1}, 'models', {models}, 'captureDir', captureDir);
save(fullfile(captureDir, 'manifest.mat'), 'manifest');

for j = 1:numel(models)
    model = models{j};
    tag = upper(erase(model, '_mil'));
    runId = ['Run_COM_capture_' tag];
    resdir = fullfile(captureDir, 'Results', runId);
    mkdir(resdir);
    datasetName = regexprep(nameToken{1}, '_[^_]+$', ['_' tag]);
    clone = strrep(par, nameToken{1}, datasetName);
    modelLine = regexp(clone, ...
        '(?m)^SIMULINK_MODEL_FILE[ \t]+[^\r\n]+', 'match', 'once');
    clone = strrep(clone, modelLine, ...
        ['SIMULINK_MODEL_FILE ' fullfile(root, [model '.slx'])]);
    parfile = fullfile(resdir, 'Run_all.par');
    write_text_(parfile, clone);

    simText = replace_macro_(sourceText, 'ROOT_FILE_NAME', runId);
    simText = replace_macro_(simText, 'WORK_DIR', [captureDir filesep]);
    simText = replace_macro_(simText, 'OUTPUT_PATH', ...
        fullfile(captureDir, 'Results'));
    simText = replace_macro_(simText, 'OUTPUT_FILE_PREFIX', ...
        fullfile(resdir, 'LastRun'));
    simfile = fullfile(captureDir, [runId '.sim']);
    write_text_(simfile, simText);

    if ~isfile(fullfile(resdir, 'LastRun.vsb')) || ...
            ~isfile(fullfile(resdir, [model '_logs.mat']))
        evalin('base', 'clear PMPC_SIMFILE PMPC_MODE PMPC_ZENGRHO PMPC_P MPC_P ZENG_P');
        assignin('base', 'PMPC_SIMFILE', simfile);
        switch model
            case 'mpc_mil',  mil_init_MPC();
            case 'zeng_mil', mil_init_ZENG();
            case 'pmpc_mil', mil_init_PMPC();
        end
        load_system(model);
        set_param([model '/CarSim'], 'SIMFILE', erase(simfile, [root filesep]));
        stopTime = func_CarSimStopTime(clone);
        if isfinite(stopTime) && stopTime > 0
            set_param(model, 'StopTime', num2str(stopTime, '%.12g'));
        end
        func_EnableControllerLogging(model);
        fprintf('Running %s (%s) ...\n', datasetName, model);
        evalc('simOut = sim(model);');
        func_WaitERD(resdir, 0);
        func_SaveControllerLogs(simOut, resdir, model);
        close_system(model, 0);
    else
        fprintf('Reusing completed %s CarSim result.\n', model);
    end
    repair_erd_header_(fullfile(resdir, 'LastRun.vs'));
    D = func_ReadERD(fullfile(resdir, 'LastRun'));
    assert(strcmp(D.Dataset, datasetName), ...
        'Wrong CarSim output for %s: %s', model, D.Dataset);
    signals = struct('t_s', D.t, 'station_m', D.Station, ...
        'vx_kmh', D.Vx, 'vx_target_kmh', D.VxTarget, ...
        'beta_deg', D.Beta, 'yaw_rate_deg_s', D.AVz, ...
        'ltr', D.LTR);
    save(fullfile(resdir, 'vehicle_signals.mat'), 'signals', '-v7.3');
    fprintf('%s: %d samples, t_end %.3f s, station_end %.2f m\n', ...
        model, D.N, D.t(end), D.Station(end));
end
evalin('base', 'clear PMPC_SIMFILE');
fprintf('COM capture saved to %s\n', captureDir);
end

function value = macro_(text, key)
t = regexp(text, ['(?m)^SET_MACRO \$\(' key '\)\$[ \t]+([^\r\n]+)'], ...
    'tokens', 'once');
assert(~isempty(t), 'Missing SIMFILE macro %s.', key);
value = strtrim(t{1});
end

function text = replace_macro_(text, key, value)
old = regexp(text, ['(?m)^SET_MACRO \$\(' key '\)\$[^\r\n]*'], ...
    'match', 'once');
assert(~isempty(old), 'Missing SIMFILE macro %s.', key);
text = strrep(text, old, ['SET_MACRO $(' key ')$ ' value]);
end

function write_text_(path, contents)
fid = fopen(path, 'w');
assert(fid >= 0, 'Cannot write %s.', path);
closer = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid, contents, 'char');
end

function repair_erd_header_(path)
raw = fileread(path);
try
    jsondecode(raw);
    return;
catch
end
badLine = regexp(raw, '(?m)^[ \t]*"Input File"[^\r\n]*', ...
    'match', 'once');
assert(~isempty(badLine), 'Unexpected CarSim ERD header format: %s', path);
clean = strrep(raw, badLine, '    "Input File" : "<legacy path omitted>",');
jsondecode(clean);
copyfile(path, [path '.raw']);
write_text_(path, clean);
end
