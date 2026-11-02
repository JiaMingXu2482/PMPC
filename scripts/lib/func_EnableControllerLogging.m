function func_EnableControllerLogging(model)
%FUNC_ENABLECONTROLLERLOGGING  Log existing controller outputs for this run.
%   The third output may be unconnected; Simulink can log its source port
%   without changing or saving the user's model wiring.

set_param(model, 'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on');
ph = get_param([model '/PMPC_MF'], 'PortHandles');
assert(numel(ph.Outport) >= 2, ...
    'func_EnableControllerLogging:MissingOutputs', ...
    '%s/PMPC_MF must expose sys and F_des.', model);
root = sfroot;
chart = root.find('-isa','Stateflow.EMChart','Path',[model '/PMPC_MF']);
expected='long_diag(1:12) = func_PriorityDiagnostics(St.cert(1:36));';
if strcmp(model,'pmpc_mil'), expected='long_diag = func_PMPCTerminalDiagnostics(St.cert);'; end
assert(numel(chart)==1 && contains(chart.Script, expected), ...
    'func_EnableControllerLogging:StalePriorityChart', ...
    '%s/PMPC_MF still outputs a placeholder diagnostic; synchronize the chart first.',model);
names = {'controller_sys', 'damper_F_des', 'priority_diag'};
for k = 1:min(numel(ph.Outport), numel(names))
    set_param(ph.Outport(k), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', names{k});
end
plant = [model '/减振器正模型'];
if getSimulinkBlockHandle(plant) ~= -1
    ports = get_param(plant,'PortHandles');
    if numel(ports.Outport)==4
        forceNames={'damper_force_L1','damper_force_L2', ...
            'damper_force_R1','damper_force_R2'};
        for k=1:4
            set_param(ports.Outport(k),'DataLogging','on', ...
                'DataLoggingNameMode','Custom','DataLoggingName',forceNames{k});
        end
    end
end
end
