function update_pmpc_longcoord_model()
%UPDATE_PMPC_LONGCOORD_MODEL Wire PMPC diagnostics and actual-brake PID gate.
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
if getSimulinkBlockHandle(sumBlock) == -1
    add_block('simulink/Math Operations/Sum',sumBlock, ...
        'Inputs','++++','Position',[590 530 620 580]);
end
pidPorts = get_param(gate,'PortHandles');
oldLine = get_param(pidPorts.Inport(3),'Line');
if oldLine ~= -1
    delete_line(oldLine);
end
sumPorts = get_param(sumBlock,'PortHandles');
for k=1:4
    if get_param(sumPorts.Inport(k),'Line') == -1
        add_line(model,sprintf('split18/%d',k), ...
            sprintf('PMPC_BrakeTorqueSum/%d',k),'autorouting','on');
    end
end
add_line(model,'PMPC_BrakeTorqueSum/1', ...
    'PID velocity control/3','autorouting','on');
% DB_on already takes abs(signal), delays one tick and gates the throttle
% and integrator. Its old 50 N*m threshold was on an unrelated channel;
% 1 N*m rejects numerical noise but catches symmetric braking promptly.
set_param([gate '/DB_on'],'const','1');

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
