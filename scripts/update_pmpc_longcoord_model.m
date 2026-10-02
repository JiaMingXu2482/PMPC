function update_pmpc_longcoord_model()
%UPDATE_PMPC_LONGCOORD_MODEL Wire PMPC diagnostics and longitudinal-brake gate.
% Only pmpc_mil is changed. CarSim's 54-channel interface is untouched.
root = fileparts(fileparts(mfilename('fullpath')));
model = 'pmpc_mil';
func_CarSimLib();
assignin('base','PMPC_CONTROLLER_VARIANT',3);
P = setup_pmpc();
assignin('base','PMPC_P',P);
assignin('base','NLCSNN',P.Pm.NLCSNN);
load_system(fullfile(root,[model '.slx']));

machine = sfroot;
chart = machine.find('-isa','Stateflow.EMChart', ...
    'Path',[model '/PMPC_MF']);
assert(numel(chart)==1,'Expected one PMPC_MF MATLAB Function chart.');
chart.Script = fileread(fullfile(root,'controller','pmpc_block.m'));

gate = [model '/PID velocity control'];
sumBlock = [model '/PMPC_BrakeTorqueSum'];
requestBlock = [model '/PMPC_LongBrakeRequest'];
if getSimulinkBlockHandle(requestBlock) == -1
    add_block('simulink/Signal Routing/Selector',requestBlock, ...
        'NumberOfDimensions','1','IndexMode','One-based', ...
        'IndexOptions','Index vector (dialog)','Indices','9', ...
        'InputPortWidth','12','Position',[575 540 610 570]);
end
pidPorts = get_param(gate,'PortHandles');
oldLine = get_param(pidPorts.Inport(3),'Line');
if oldLine ~= -1
    delete_line(oldLine);
end
if getSimulinkBlockHandle(sumBlock) ~= -1
    delete_block(sumBlock);
end
requestPorts = get_param(requestBlock,'PortHandles');
if get_param(requestPorts.Inport(1),'Line') == -1
    add_line(model,'PMPC_MF/3','PMPC_LongBrakeRequest/1', ...
        'autorouting','on');
end
add_line(model,'PMPC_LongBrakeRequest/1', ...
    'PID velocity control/3','autorouting','on');
% Port 3 is now the PMPC longitudinal brake request (N), not yaw DB torque.
% Keep the existing integrator freeze and also block throttle immediately.
set_param([gate '/DB_on'],'const','1');
throttleGate = [gate '/LongBrakeThrottleGate'];
if getSimulinkBlockHandle(throttleGate) == -1
    add_block('simulink/Signal Routing/Switch',throttleGate, ...
        'Criteria','u2 > Threshold','Threshold','0', ...
        'Position',[610 155 645 195]);
    outPorts = get_param([gate '/Out1'],'PortHandles');
    outLine = get_param(outPorts.Inport(1),'Line');
    if outLine ~= -1
        delete_line(outLine);
    end
    set_param([gate '/Out1'],'Position',[690 167 720 183]);
    add_line(gate,'Constant1/1','LongBrakeThrottleGate/1','autorouting','on');
    add_line(gate,'DB_on/1','LongBrakeThrottleGate/2','autorouting','on');
    add_line(gate,'Saturation/1','LongBrakeThrottleGate/3','autorouting','on');
    add_line(gate,'LongBrakeThrottleGate/1','Out1/1','autorouting','on');
end
set_param(throttleGate,'Threshold','0');

scope = [model '/PMPC_LongCoord_Diagnostics'];
if getSimulinkBlockHandle(scope) == -1
    add_block('simulink/Sinks/Scope',scope, ...
        'Position',[470 590 500 620]);
end
chartPorts = get_param([model '/PMPC_MF'],'PortHandles');
assert(numel(chartPorts.Outport)==3,'PMPC_MF must expose a diagnostic port.');
scopePorts = get_param(scope,'PortHandles');
if get_param(scopePorts.Inport(1),'Line') == -1
    add_line(model,'PMPC_MF/3','PMPC_LongCoord_Diagnostics/1', ...
        'autorouting','on');
end
logger = [model '/PMPC_LongCoord_Log'];
if getSimulinkBlockHandle(logger) == -1
    add_block('simulink/Sinks/To Workspace',logger, ...
        'VariableName','PMPC_LONG_DIAG','SaveFormat','Structure With Time', ...
        'Position',[470 640 590 670]);
end
logPorts = get_param(logger,'PortHandles');
if get_param(logPorts.Inport(1),'Line') == -1
    add_line(model,'PMPC_MF/3','PMPC_LongCoord_Log/1', ...
        'autorouting','on');
end
save_system(model);
end
