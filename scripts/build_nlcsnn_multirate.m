function build_nlcsnn_multirate(mdl)
%BUILD_NLCSNN_MULTIRATE Build the 100 Hz inverse / 1 kHz forward split.
%   The only new top-level functional block is NLCSNN_Forward_1kHz.

if nargin < 1 || isempty(mdl)
    mdl = 'pmpc_mil';
end

localPrepareWorkspace();
load_system(mdl);
open_system(mdl);

localConfigureController(mdl);
localRemoveLegacyForwardPath(mdl);
localCreateForwardBlock(mdl);
localCreatePassivityBlock(mdl);
localWireForwardBlock(mdl);

set_param(mdl, 'FixedStep', '0.001');
set_param(mdl, 'SimulationCommand', 'update');
save_system(mdl);
fprintf(['NLCSNN multirate build complete. PMPC_MF runs at 0.01 s; ' ...
         'NLCSNN_Forward_1kHz and CarSim run at 0.001 s.\n']);
end

function localPrepareWorkspace()
if evalin('base', 'exist(''PMPC_P'', ''var'')') ~= 1
    evalin('base', 'setup_pmpc;');
end
P = evalin('base', 'PMPC_P');
assert(isfield(P, 'Pm') && isfield(P.Pm, 'NLCSNN'), ...
    'build_nlcsnn_multirate:MissingConfiguration', ...
    'Run setup_pmpc before building the NLCSNN model.');
assert(P.Pm.NLCSNN.dt_plant == 0.001, ...
    'build_nlcsnn_multirate:InvalidPlantStep', ...
    'NLCSNN.dt_plant must be 0.001 s.');
assignin('base', 'NLCSNN', P.Pm.NLCSNN);
end

function localConfigureController(mdl)
block = [mdl '/PMPC_MF'];
chart = localFindChart(block);
codePath = which('pmpc_block.m');
assert(~isempty(codePath), 'build_nlcsnn_multirate:MissingController', ...
    'pmpc_block.m is not on the MATLAB path.');
chart.Script = fileread(codePath);
parameter = chart.find('-isa', 'Stateflow.Data', 'Name', 'PMPC_P');
assert(~isempty(parameter), 'build_nlcsnn_multirate:MissingControllerParameter', ...
    'PMPC_P was not created as a MATLAB Function parameter.');
parameter(1).Scope = 'Parameter';
parameter(1).Tunable = false;
end

function localRemoveLegacyForwardPath(mdl)
% A previous assembly left the real plant inside a generic Subsystem wrapper.
% Remove only that wrapper, never unrelated user subsystems.
wrapper = [mdl '/Subsystem'];
if localBlockExists(wrapper) && ~isempty(find_system(wrapper, ...
        'SearchDepth', 2, 'Name', 'NLCSNN_Plant'))
    delete_block(wrapper);
end

names = {'NLCSNN_Plant','NLCSNN_Forward_1kHz','SemiActive_Passivity_1kHz', ...
         'Fd_demux','Fd_plant_demux','Fd_safe_demux', ...
         'RT_h','RT_x','RT_v','RT_a','RT_i', ...
         'SIG_h','SIG_x','SIG_v','SIG_a','SIG_i_cmd'};
for k = 1:numel(names)
    localDeleteTopLevel(mdl, names{k});
end
end

function localCreateForwardBlock(mdl)
sub = [mdl '/NLCSNN_Forward_1kHz'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [300 125 520 330], 'BackgroundColor', 'lightBlue');
localDeleteChildren(sub, 'Inport');
localDeleteChildren(sub, 'Outport');

add_block('simulink/Sources/In1', [sub '/u54'], ...
    'Port', '1', 'Position', [25 55 55 69]);
add_block('simulink/Sources/In1', [sub '/i_cmd'], ...
    'Port', '2', 'Position', [25 105 55 119]);
add_block('simulink/Sinks/Out1', [sub '/Fd_plant'], ...
    'Port', '1', 'Position', [710 55 740 69]);
add_block('simulink/Sinks/Out1', [sub '/h_plant'], ...
    'Port', '2', 'Position', [710 155 740 169]);
add_block('simulink/Sinks/Out1', [sub '/x_plant'], ...
    'Port', '3', 'Position', [710 205 740 219]);
add_block('simulink/Sinks/Out1', [sub '/v_plant'], ...
    'Port', '4', 'Position', [710 255 740 269]);
add_block('simulink/Sinks/Out1', [sub '/a_plant'], ...
    'Port', '5', 'Position', [710 305 740 319]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [sub '/plant_fn'], 'Position', [245 45 405 245]);
add_block('simulink/Discrete/Unit Delay', [sub '/UD_h'], ...
    'InitialCondition', 'zeros(8,4)', 'SampleTime', '0.001', ...
    'Position', [445 305 485 345]);
add_block('simulink/Discrete/Unit Delay', [sub '/UD_af'], ...
    'InitialCondition', 'zeros(4,1)', 'SampleTime', '0.001', ...
    'Position', [445 360 485 400]);
add_block('simulink/Discrete/Unit Delay', [sub '/UD_vp'], ...
    'InitialCondition', 'zeros(4,1)', 'SampleTime', '0.001', ...
    'Position', [445 415 485 455]);
add_block('simulink/Discrete/Unit Delay', [sub '/UD_init'], ...
    'InitialCondition', '0', 'SampleTime', '0.001', ...
    'Position', [445 470 485 510]);

add_block('simulink/Signal Attributes/Rate Transition', [sub '/RT_i'], ...
    'Position', [90 100 130 124]);
add_block('simulink/Signal Attributes/Signal Specification', [sub '/SIG_i'], ...
    'Dimensions', '4', 'Position', [155 100 180 124]);
localAddStateOutputPath(sub, 'h', '[8 4]', 155);
localAddStateOutputPath(sub, 'x', '[4 1]', 205);
localAddStateOutputPath(sub, 'v', '[4 1]', 255);
localAddStateOutputPath(sub, 'a', '[4 1]', 305);

chart = localFindChart([sub '/plant_fn']);
chart.Script = localPlantScript();
parameter = chart.find('-isa', 'Stateflow.Data', 'Name', 'NLCSNN');
assert(~isempty(parameter), 'build_nlcsnn_multirate:MissingPlantParameter', ...
    'NLCSNN was not created as a MATLAB Function parameter.');
parameter(1).Scope = 'Parameter';
parameter(1).Tunable = false;
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = '0.001';

localWirePlantInternals(sub);
end

function localCreatePassivityBlock(mdl)
sub = [mdl '/SemiActive_Passivity_1kHz'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [590 320 760 420], 'BackgroundColor', 'green');
localDeleteChildren(sub, 'Inport');
localDeleteChildren(sub, 'Outport');

add_block('simulink/Sources/In1', [sub '/F_raw'], ...
    'Port', '1', 'Position', [25 55 55 69]);
add_block('simulink/Sources/In1', [sub '/u54'], ...
    'Port', '2', 'Position', [25 105 55 119]);
add_block('simulink/Sinks/Out1', [sub '/F_safe'], ...
    'Port', '1', 'Position', [410 75 440 89]);
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [sub '/passivity_fn'], 'Position', [145 45 315 130]);

chart = localFindChart([sub '/passivity_fn']);
chart.Script = localPassivityScript();
parameter = chart.find('-isa', 'Stateflow.Data', 'Name', 'NLCSNN');
assert(~isempty(parameter), 'build_nlcsnn_multirate:MissingPassivityParameter', ...
    'NLCSNN was not created as a MATLAB Function parameter.');
parameter(1).Scope = 'Parameter';
parameter(1).Tunable = false;
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = '0.001';

connect = @(source, destination) add_line(sub, source, destination, ...
    'autorouting', 'on');
connect('F_raw/1', 'passivity_fn/1');
connect('u54/1', 'passivity_fn/2');
connect('passivity_fn/1', 'F_safe/1');
end

function localAddStateOutputPath(sub, name, dimensions, y)
add_block('simulink/Signal Attributes/Rate Transition', [sub '/RT_' name], ...
    'Position', [500 y 540 y+24]);
add_block('simulink/Signal Attributes/Signal Specification', ...
    [sub '/SIG_' name], 'Dimensions', dimensions, ...
    'Position', [570 y 595 y+24]);
end

function localWirePlantInternals(sub)
connect = @(source, destination) add_line(sub, source, destination, ...
    'autorouting', 'on');
connect('u54/1', 'plant_fn/1');
connect('i_cmd/1', 'RT_i/1');
connect('RT_i/1', 'SIG_i/1');
connect('SIG_i/1', 'plant_fn/2');
connect('UD_h/1', 'plant_fn/3');
connect('UD_af/1', 'plant_fn/4');
connect('UD_vp/1', 'plant_fn/5');
connect('UD_init/1', 'plant_fn/6');
connect('plant_fn/2', 'UD_h/1');
connect('plant_fn/3', 'UD_af/1');
connect('plant_fn/4', 'UD_vp/1');
connect('plant_fn/5', 'UD_init/1');
connect('plant_fn/1', 'Fd_plant/1');

names = {'h','x','v','a'};
sourcePorts = [2 6 7 8];
destinations = {'h_plant/1','x_plant/1','v_plant/1','a_plant/1'};
for k = 1:4
    name = names{k};
    connect(sprintf('plant_fn/%d', sourcePorts(k)), ['RT_' name '/1']);
    connect(['RT_' name '/1'], ['SIG_' name '/1']);
    connect(['SIG_' name '/1'], destinations{k});
end
end

function localWireForwardBlock(mdl)
sub = [mdl '/NLCSNN_Forward_1kHz'];
constraint = [mdl '/SemiActive_Passivity_1kHz'];
controller = [mdl '/PMPC_MF'];
controllerPorts = get_param(controller, 'PortHandles');
subPorts = get_param(sub, 'PortHandles');
constraintPorts = get_param(constraint, 'PortHandles');
assert(numel(controllerPorts.Inport) == 5 && numel(controllerPorts.Outport) == 2, ...
    'build_nlcsnn_multirate:ControllerPorts', ...
    'PMPC_MF must have five inputs and two outputs.');

% Branch CarSim's 54-vector to the 1 kHz plant without slowing it to 100 Hz.
line = get_param(controllerPorts.Inport(1), 'Line');
assert(line ~= -1, 'build_nlcsnn_multirate:MissingCarSimMeasurement', ...
    'PMPC_MF input 1 must be driven by the CarSim export vector.');
sourcePort = get_param(line, 'SrcPortHandle');
for k = 2:5
    localClearInputLine(mdl, controllerPorts.Inport(k));
end
% add_line can invalidate PortHandles. Remove every old state connection
% first, then acquire fresh handles for the new multirate wiring.
controllerPorts = get_param(controller, 'PortHandles');
subPorts = get_param(sub, 'PortHandles');
add_line(mdl, sourcePort, subPorts.Inport(1), 'autorouting', 'on');
add_line(mdl, sourcePort, constraintPorts.Inport(2), 'autorouting', 'on');
add_line(mdl, controllerPorts.Outport(2), subPorts.Inport(2), ...
    'autorouting', 'on');
for k = 2:5
    add_line(mdl, subPorts.Outport(k), controllerPorts.Inport(k), ...
        'autorouting', 'on');
end

demux = [mdl '/Fd_safe_demux'];
add_block('simulink/Signal Routing/Demux', demux, 'Outputs', '4', ...
    'Position', [810 250 820 330]);
demuxPorts = get_param(demux, 'PortHandles');
add_line(mdl, subPorts.Outport(1), constraintPorts.Inport(1), ...
    'autorouting', 'on');
add_line(mdl, constraintPorts.Outport(1), demuxPorts.Inport(1), ...
    'autorouting', 'on');
packPorts = get_param([mdl '/pack11'], 'PortHandles');
for k = 1:4
    localClearInputLine(mdl, packPorts.Inport(k+4));
    add_line(mdl, demuxPorts.Outport(k), packPorts.Inport(k+4), ...
        'autorouting', 'on');
end
localTerminateLegacyForceOutputs(mdl);
end

function localTerminateLegacyForceOutputs(mdl)
splitter = [mdl '/split18'];
if ~localBlockExists(splitter)
    return
end
ports = get_param(splitter, 'PortHandles');
for k = 1:4
    output = ports.Outport(k+4);
    if get_param(output, 'Line') == -1
        term = sprintf('%s/TERM_Fd%d', mdl, k);
        if ~localBlockExists(term)
            add_block('simulink/Sinks/Terminator', term, ...
                'Position', [590 245+25*k 610 255+25*k]);
        end
        termPorts = get_param(term, 'PortHandles');
        add_line(mdl, output, termPorts.Inport(1), 'autorouting', 'on');
    end
end
end

function localClearInputLine(mdl, inputPort)
line = get_param(inputPort, 'Line');
if line ~= -1
    % A deleted wrapper can leave a dangling line whose source port is -1.
    % Deleting by line handle works for both ordinary and dangling lines.
    delete_line(line);
end
end

function localDeleteTopLevel(mdl, name)
blocks = find_system(mdl, 'SearchDepth', 1, 'Name', name);
for k = 1:numel(blocks)
    if ~strcmp(blocks{k}, mdl)
        delete_block(blocks{k});
    end
end
end

function localDeleteChildren(sub, blockType)
blocks = find_system(sub, 'SearchDepth', 1, 'BlockType', blockType);
for k = 1:numel(blocks)
    if ~strcmp(blocks{k}, sub)
        delete_block(blocks{k});
    end
end
end

function exists = localBlockExists(path)
exists = getSimulinkBlockHandle(path) ~= -1;
end

function chart = localFindChart(path)
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', path);
assert(numel(chart) == 1, 'build_nlcsnn_multirate:MissingChart', ...
    'Expected exactly one MATLAB Function chart at %s.', path);
end

function script = localPlantScript()
script = sprintf([ ...
    'function [F_plant,h_new,a_filt_new,v_prev_new,init_new,x_plant,v_plant,a_plant] = ...\n' ...
    '    plant_fn(u54,i_cmd,h,a_filt,v_prev,init_done,NLCSNN)\n' ...
    '%%#codegen\n' ...
    'u54 = reshape(u54,54,1);\n' ...
    'i_cmd = reshape(i_cmd,4,1);\n' ...
    'h = reshape(h,8,4);\n' ...
    'a_filt = reshape(a_filt,4,1);\n' ...
    'v_prev = reshape(v_prev,4,1);\n' ...
    'cmpRD = u54(1:4);\n' ...
    'cmpD = u54(51:54);\n' ...
    '[F_plant,h_new,a_filt_new,v_prev_new,init_new,x_plant,v_plant,a_plant] = ...\n' ...
    '    func_NLCSNNPlant4(NLCSNN.net,cmpD,cmpRD,i_cmd,h,a_filt,v_prev,init_done,NLCSNN);\n' ...
    'end\n']);
end

function script = localPassivityScript()
script = sprintf([ ...
    'function F_safe = passivity_fn(F_raw,u54,NLCSNN)\n' ...
    '%%#codegen\n' ...
    'F_raw = reshape(F_raw,4,1);\n' ...
    'u54 = reshape(u54,54,1);\n' ...
    'cmpRD = u54(1:4);\n' ...
    'F_safe = func_SemiActivePassivity(F_raw,cmpRD,NLCSNN.passivity);\n' ...
    'end\n']);
end
