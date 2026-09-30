function S = capture_nlcsnn_signals(outFile)
%CAPTURE_NLCSNN_SIGNALS Run current CarSim dataset and log NLCSNN internals.
% Logging changes are in-memory only; the Simulink model is not saved.

root = startup_pmpc();
if nargin < 1 || isempty(outFile)
    outFile = fullfile(root, 'scratchpad', 'nlcsnn_internal_signals.mat');
end
simfile = fullfile(root, 'simfile.sim');
sf = fileread(simfile);
runId = regexp(sf, 'ROOT_FILE_NAME\)\$\s+(Run_[^\s]+)', 'tokens', 'once');
wd = regexp(sf, 'WORK_DIR\)\$\s+([^\r\n]+)', 'tokens', 'once');
assert(~isempty(runId) && ~isempty(wd), 'Bad simfile.sim');
resdir = fullfile(strtrim(wd{1}), 'Results', runId{1});
func_SimModel(fullfile(resdir, 'Run_all.par'));
assert(func_CarSimRunning(), 'CarSim must remain open.');
func_CarSimLib();

evalin('base', 'clear PMPC_MODE PMPC_ZENGRHO PMPC_ABL PMPC_P');
assignin('base', 'PMPC_SIMFILE', simfile);
assignin('base', 'PMPC_VERBOSE', 0);
evalin('base', 'setup_pmpc;');
% Diagnostic-only override: keep production setup unchanged while testing
% causal acceleration filters against the 30 Hz training preprocessing.
if evalin('base', 'exist(''PMPC_NLCSNN_TAU_ACCEL_DIAG'',''var'')')
    Pdiag = evalin('base', 'PMPC_P');
    Pdiag.Pm.NLCSNN.tau_accel = evalin('base', ...
        'PMPC_NLCSNN_TAU_ACCEL_DIAG');
    assignin('base', 'PMPC_P', Pdiag);
    assignin('base', 'NLCSNN', Pdiag.Pm.NLCSNN);
end

mdl = 'pmpc_mil';
load_system(mdl);
cleanup = onCleanup(@() local_cleanup_(mdl)); %#ok<NASGU>
set_param([mdl '/CarSim'], 'SIMFILE', 'simfile.sim');
set_param(mdl, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

local_log_port_([mdl '/PMPC_MF'], 'Outport', 2, 'i_cmd_100Hz');
local_log_port_([mdl '/NLCSNN_Forward_1kHz'], 'Outport', 1, 'F_raw_1kHz');
% The top-level v/a outputs deliberately rate-transition to 100 Hz for the
% controller. Log plant_fn outputs before those transitions to observe the
% true 1 kHz plant inputs.
local_log_port_([mdl '/NLCSNN_Forward_1kHz/plant_fn'], 'Outport', 7, 'v_plant_1kHz');
local_log_port_([mdl '/NLCSNN_Forward_1kHz/plant_fn'], 'Outport', 8, 'a_plant_1kHz');
local_log_port_([mdl '/NLCSNN_Forward_1kHz/RT_i'], 'Outport', 1, 'i_hold_1kHz');
local_log_port_([mdl '/SemiActive_Passivity_1kHz'], 'Outport', 1, 'F_safe_1kHz');

out = sim(mdl);
L = out.logsout;
S = struct();
names = {'i_cmd_100Hz','F_raw_1kHz','v_plant_1kHz', ...
         'a_plant_1kHz','i_hold_1kHz','F_safe_1kHz'};
for k = 1:numel(names)
    V = L.getElement(names{k}).Values;
    d = squeeze(V.Data);
    if size(d,1) ~= numel(V.Time), d = d.'; end
    S.([names{k} '_t']) = V.Time(:);
    S.(names{k}) = d;
end
S.runId = runId{1};
S.resdir = resdir;
save(outFile, '-struct', 'S');
fprintf('NLCSNN internal signals saved: %s\n', outFile);
end

function local_log_port_(block, kind, index, name)
ph = get_param(block, 'PortHandles');
p = ph.(kind)(index);
set_param(p, 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

function local_cleanup_(mdl)
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalin('base', 'clear PMPC_SIMFILE');
end
