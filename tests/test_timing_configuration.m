function tests = test_timing_configuration
tests = functiontests(localfunctions);
end

function testMpcUses50msPredictionWith100HzTrigger(testCase)
modelDir = fileparts(fileparts(mfilename('fullpath')));
solverDir = 'D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers\Matlab84+';
addpath(modelDir, solverDir);

clear global;
cq32019mpc(0, [], [], 0);
global MPCParameters;

verifyEqual(testCase, MPCParameters.Ts, 0.05, 'AbsTol', 1e-12);
verifyEqual(testCase, MPCParameters.Np, 12);
verifyEqual(testCase, MPCParameters.Nc, 6);

load_system('Solver_SF');
load_system(fullfile(modelDir, 'cq3_2019.slx'));
cleanup = onCleanup(@() closeModels()); %#ok<NASGU>
trigger = sprintf('cq3_2019/Function-Call\nGenerator2');
verifyEqual(testCase, str2double(get_param(trigger, 'sample_time')), ...
    0.01, 'AbsTol', 1e-12);
end

function closeModels
close_system('cq3_2019', 0);
close_system('Solver_SF', 0);
end
