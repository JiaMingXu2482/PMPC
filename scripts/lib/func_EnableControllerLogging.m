function func_EnableControllerLogging(model)
%FUNC_ENABLECONTROLLERLOGGING  Log existing controller outputs for this run.
%   The third output may be unconnected; Simulink can log its source port
%   without changing or saving the user's model wiring.

set_param(model, 'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on');
ph = get_param([model '/PMPC_MF'], 'PortHandles');
assert(numel(ph.Outport) >= 2, ...
    'func_EnableControllerLogging:MissingOutputs', ...
    '%s/PMPC_MF must expose sys and i_cmd.', model);
if strcmp(model,'pmpc_mil') || strcmp(model,'pmpc_nodelay_mil')
    root = sfroot;
    chart = root.find('-isa','Stateflow.EMChart','Path',[model '/PMPC_MF']);
    expected='long_diag = func_PriorityDiagnostics(St.cert);';
    if strcmp(model,'pmpc_mil'), expected='long_diag = func_PMPCTerminalDiagnostics(St.cert);'; end
    assert(numel(chart)==1 && contains(chart.Script, ...
        expected), ...
        'func_EnableControllerLogging:StalePriorityChart', ...
        '%s/PMPC_MF still outputs a placeholder diagnostic; synchronize the chart first.',model);
end
names = {'controller_sys', 'damper_i_cmd', 'priority_diag'};
for k = 1:min(numel(ph.Outport), numel(names))
    set_param(ph.Outport(k), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', names{k});
end
plant = [model '/NLCSNN_Forward_1kHz'];
if getSimulinkBlockHandle(plant) ~= -1
    ports = get_param(plant,'PortHandles');
    if numel(ports.Outport) >= 6
        indices = [1 6];
        forceNames = {'damper_force_applied','damper_force_raw'};
        for k=1:2
            set_param(ports.Outport(indices(k)),'DataLogging','on', ...
                'DataLoggingNameMode','Custom','DataLoggingName',forceNames{k});
        end
    end
end
end
