function tests = test_controller_logging
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testPriorityDiagnosticsExposeOnlineDecision(testCase)
cert = (1:36).';
actual = func_PriorityDiagnostics(cert);
expected = [1; 2; 3; 4; 5; 6; 7; 26; 27; 34; 35; 36];
verifyEqual(testCase, actual, expected);
end

function testControllerPortsAreLoggedWithoutChangingConnections(testCase)
models = {'mpc_mil', 'zeng_mil', 'pmpc_mil', 'pmpc_nodelay_mil'};
cleanup = onCleanup(@() local_close_models(models)); %#ok<NASGU>
for k = 1:numel(models)
    model = models{k};
    load_system(model);
    ph = get_param([model '/PMPC_MF'], 'PortHandles');
    linesBefore = arrayfun(@(p) get_param(p, 'Line'), ph.Outport);

    func_EnableControllerLogging(model);

    verifyEqual(testCase, get_param(model, 'SignalLogging'), 'on');
    verifyEqual(testCase, get_param(ph.Outport(1), 'DataLogging'), 'on');
    verifyEqual(testCase, get_param(ph.Outport(1), 'DataLoggingName'), 'controller_sys');
    verifyEqual(testCase, get_param(ph.Outport(2), 'DataLogging'), 'on');
    verifyEqual(testCase, get_param(ph.Outport(2), 'DataLoggingName'), 'damper_i_cmd');
    if numel(ph.Outport) >= 3
        verifyEqual(testCase, get_param(ph.Outport(3), 'DataLogging'), 'on');
        verifyEqual(testCase, get_param(ph.Outport(3), 'DataLoggingName'), 'priority_diag');
    end
    linesAfter = arrayfun(@(p) get_param(p, 'Line'), ph.Outport);
    verifyEqual(testCase, linesAfter, linesBefore);
end
end

function testSimulationLogsAreSavedBesideCarSimResult(testCase)
outDir = tempname;
mkdir(outDir);
cleanup = onCleanup(@() local_remove_dir(outDir)); %#ok<NASGU>
logsout = Simulink.SimulationData.Dataset;
logsout = logsout.addElement(local_signal('controller_sys', [10; 20]));
logsout = logsout.addElement(local_signal('damper_i_cmd', [0; 1]));
logsout = logsout.addElement(local_signal('priority_diag', [2; 3]));
simOut = struct('logsout', logsout);

outFile = func_SaveControllerLogs(simOut, outDir, 'pmpc_mil');

verifyEqual(testCase, outFile, fullfile(outDir, 'pmpc_mil_logs.mat'));
verifyEqual(testCase, exist(outFile, 'file'), 2);
saved = load(outFile, 'logsout');
verifyEqual(testCase, saved.logsout.numElements, 3);
verifyEqual(testCase, saved.logsout.getElement('controller_sys').Values.Data, [10; 20]);
end

function signal = local_signal(name, data)
signal = Simulink.SimulationData.Signal;
signal.Name = name;
signal.Values = timeseries(data, [0; 0.01]);
end

function local_close_models(models)
for k = 1:numel(models)
    if bdIsLoaded(models{k}), close_system(models{k}, 0); end
end
end

function local_remove_dir(path)
file = fullfile(path, 'pmpc_mil_logs.mat');
if exist(file, 'file') == 2, delete(file); end
if exist(path, 'dir') == 7, rmdir(path); end
end
