function build_controller_variants()
%BUILD_CONTROLLER_VARIANTS Build fixed MPC, ZENG, and PMPC top-level models.
%   All models share the NLCSNN 1 kHz plant and CarSim packing from the
%   current pmpc_mil topology. Only the 100 Hz MATLAB Function entry point
%   and its fixed parameter bundle differ.

startup_pmpc();
source = 'pmpc_mil';
assert(exist([source '.slx'], 'file') == 4, ...
    'build_controller_variants:MissingSource', 'Cannot find %s.slx.', source);

variants = { ...
    'mpc_mil',  'mpc_block',  'MPC_P',  false; ...
    'zeng_mil', 'zeng_block', 'ZENG_P', true;  ...
    'pmpc_mil', 'pmpc_block', 'PMPC_P', false};

for k = 1:size(variants,1)
    target = variants{k,1};
    entry = variants{k,2};
    parameterName = variants{k,3};
    hasRho = variants{k,4};

    if ~strcmp(target, source)
        if bdIsLoaded(source), close_system(source, 0); end
        load_system(source);
        save_system(source, target);
        close_system(source, 0);
    end

    if bdIsLoaded(target), close_system(target, 0); end
    load_system(target);
    localConfigureController(target, entry, parameterName);
    localAddCarSimSignalSpecification(target);
    if hasRho
        localAddZengRhoOutport(target);
    else
        localRemoveZengRhoOutport(target);
    end
    save_system(target);
    close_system(target, 0);
end

fprintf('Built fixed controller models: mpc_mil, zeng_mil, pmpc_mil.\n');
end

function localAddCarSimSignalSpecification(mdl)
% vs_sf exposes a dynamically sized vector. Declare the shared boundary
% once before it fans out to the controller and the 1 kHz NLCSNN plant.
block = [mdl '/CarSim_Output_54'];
if getSimulinkBlockHandle(block) ~= -1
    set_param(block, 'Dimensions', '54');
    return
end

carSimPorts = get_param([mdl '/CarSim'], 'PortHandles');
sourceLine = get_param(carSimPorts.Outport(1), 'Line');
assert(sourceLine ~= -1, 'build_controller_variants:MissingCarSimLine', ...
    'CarSim output is not connected in %s.', mdl);
destinations = get_param(sourceLine, 'DstPortHandle');
destinations = destinations(destinations ~= -1);
assert(~isempty(destinations), 'build_controller_variants:MissingCarSimDestinations', ...
    'CarSim output has no destinations in %s.', mdl);

carSimPosition = get_param([mdl '/CarSim'], 'Position');
yMid = round((carSimPosition(2) + carSimPosition(4)) / 2);
add_block('simulink/Signal Attributes/Signal Specification', block, ...
    'Dimensions', '54', ...
    'Position', [carSimPosition(3)+45 yMid-15 carSimPosition(3)+115 yMid+15]);
specPorts = get_param(block, 'PortHandles');

delete_line(sourceLine);
add_line(mdl, carSimPorts.Outport(1), specPorts.Inport(1), 'autorouting', 'on');
for k = 1:numel(destinations)
    add_line(mdl, specPorts.Outport(1), destinations(k), 'autorouting', 'on');
end
end

function localConfigureController(mdl, entry, parameterName)
chart = localFindChart([mdl '/PMPC_MF']);
oldNames = {'PMPC_P','MPC_P','ZENG_P'};
old = Stateflow.Data.empty;
for k = 1:numel(oldNames)
    data = chart.find('-isa', 'Stateflow.Data', 'Name', oldNames{k});
    if ~isempty(data)
        old = data(1);
        break
    end
end
if ~isempty(old) && ~strcmp(old.Name, parameterName)
    old.Name = parameterName;
end

codePath = which([entry '.m']);
assert(~isempty(codePath), 'build_controller_variants:MissingEntryPoint', ...
    '%s.m is not on the MATLAB path.', entry);
chart.Script = fileread(codePath);
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = '0.01';
chart.SupportVariableSizing = true;

portContracts = { ...
    'u',       '54'; ...
    'h_plant', '[8 4]'; ...
    'x_plant', '[4 1]'; ...
    'v_plant', '[4 1]'; ...
    'a_plant', '[4 1]'; ...
    'sys',     '54'; ...
    'i_cmd',   '4'};
for k = 1:size(portContracts,1)
    localSetFixedSize(chart, portContracts{k,1}, portContracts{k,2});
end

parameter = chart.find('-isa', 'Stateflow.Data', 'Name', parameterName);
assert(numel(parameter) == 1, 'build_controller_variants:MissingParameter', ...
    '%s must be a MATLAB Function parameter in %s.', parameterName, mdl);
parameter.Scope = 'Parameter';
parameter.Tunable = false;
end

function localAddZengRhoOutport(mdl)
localRemoveZengRhoOutport(mdl);

selector = [mdl '/rho_ZENG_select'];
outport = [mdl '/rho_ZENG'];
add_block('simulink/User-Defined Functions/MATLAB Function', selector, ...
    'Position', [700 640 830 690]);
chart = localFindChart(selector);
chart.Script = sprintf([ ...
    'function rho_ZENG = rho_ZENG_select(sys)\n' ...
    '%%#codegen\n' ...
    'sys = reshape(sys,54,1);\n' ...
    'rho_ZENG = sys(14);\n' ...
    'end\n']);
chart.SupportVariableSizing = false;
localSetFixedSize(chart, 'sys', '54');
localSetFixedSize(chart, 'rho_ZENG', '1');

add_block('simulink/Sinks/Out1', outport, ...
    'Port', '1', 'Position', [900 655 930 675]);
controller = get_param([mdl '/PMPC_MF'], 'PortHandles');
selectorPorts = get_param(selector, 'PortHandles');
outPorts = get_param(outport, 'PortHandles');
add_line(mdl, controller.Outport(1), selectorPorts.Inport(1), 'autorouting', 'on');
add_line(mdl, selectorPorts.Outport(1), outPorts.Inport(1), 'autorouting', 'on');
end

function localSetFixedSize(chart, name, sizeText)
data = chart.find('-isa', 'Stateflow.Data', 'Name', name);
assert(numel(data) == 1, 'build_controller_variants:MissingPortData', ...
    'Expected one Stateflow data item named %s in %s.', name, chart.Path);
data.Props.Array.Size = sizeText;
data.Props.Array.IsDynamic = false;
end

function localRemoveZengRhoOutport(mdl)
for name = {'rho_ZENG_select','rho_ZENG'}
    block = [mdl '/' name{1}];
    if getSimulinkBlockHandle(block) ~= -1
        delete_block(block);
    end
end
end

function chart = localFindChart(path)
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', path);
assert(numel(chart) == 1, 'build_controller_variants:MissingChart', ...
    'Expected exactly one MATLAB Function chart at %s.', path);
end
