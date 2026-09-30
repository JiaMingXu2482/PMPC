function tests = test_nlcsnn_multirate_model
%TEST_NLCSNN_MULTIRATE_MODEL Regression checks for the 100 Hz / 1 kHz split.
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
addpath('D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers\Matlab84+');
startup_pmpc();
end

function testDedicatedForwardPlantExistsAtOneKhz(testCase)
% A single top-level plant must own the 1 kHz forward rollout.  The
% controller itself only issues i_cmd at 100 Hz.
P = setup_pmpc(); %#ok<NASGU>
mdl = 'pmpc_mil';
load_system(mdl);
cleanup = onCleanup(@() close_models(mdl)); %#ok<NASGU>

plant = [mdl '/NLCSNN_Forward_1kHz'];
plants = find_system(mdl, 'SearchDepth', 1, 'Name', 'NLCSNN_Forward_1kHz');
verifyEqual(testCase, numel(plants), 1);
if numel(plants) ~= 1
    return
end
verifyEqual(testCase, numel(find_system(plant, 'SearchDepth', 1, ...
    'Name', 'plant_fn')), 1);

root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', [plant '/plant_fn']);
verifyEqual(testCase, numel(chart), 1);
verifyEqual(testCase, chart.SampleTime, '0.001');
end

function testPassivityConstraintFeedsCarSimDamperPorts(testCase)
% pack11 inputs 5:8 must receive only the 1 kHz passivity-constrained
% force, never raw neural force or the controller diagnostic vector.
P = setup_pmpc(); %#ok<NASGU>
mdl = 'pmpc_mil';
load_system(mdl);
cleanup = onCleanup(@() close_models(mdl)); %#ok<NASGU>

constraint = [mdl '/SemiActive_Passivity_1kHz'];
constraints = find_system(mdl, 'SearchDepth', 1, ...
    'Name', 'SemiActive_Passivity_1kHz');
verifyEqual(testCase, numel(constraints), 1);
if numel(constraints) ~= 1
    return
end
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', ...
    'Path', [constraint '/passivity_fn']);
verifyEqual(testCase, numel(chart), 1);
verifyEqual(testCase, chart.SampleTime, '0.001');

packPorts = get_param([mdl '/pack11'], 'PortHandles');
for k = 1:4
    line = get_param(packPorts.Inport(k+4), 'Line');
    verifyNotEqual(testCase, line, -1);
    source = get_param(line, 'SrcBlockHandle');
    verifyEqual(testCase, get_param(source, 'Name'), 'Fd_safe_demux');
end
end

function close_models(mdl)
close_system(mdl, 0);
if bdIsLoaded('Solver_SF')
    close_system('Solver_SF', 0);
end
end
