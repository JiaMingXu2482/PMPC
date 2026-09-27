function tests = test_timing_configuration
tests = functiontests(localfunctions);
end

function testMpcTimingParametersAndSubsystem(testCase)
modelDir = fileparts(fileparts(mfilename('fullpath')));
solverDir = 'D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers\Matlab84+';
addpath(modelDir, solverDir);
startup_pmpc();

P = setup_pmpc();
MPCParameters = P.Pm.MPCParameters;

verifyEqual(testCase, MPCParameters.Ts, 0.05, 'AbsTol', 1e-12);
verifyEqual(testCase, MPCParameters.Np, 20);
verifyEqual(testCase, MPCParameters.Nc, 6);
verifyEqual(testCase, MPCParameters.Ts_exec, 0.01, 'AbsTol', 1e-12);

load_system('Solver_SF');
load_system(fullfile(modelDir, 'pmpc_mil.slx'));
cleanup = onCleanup(@() closeModels()); %#ok<NASGU>
blocks = find_system('pmpc_mil','LookUnderMasks','all','FollowLinks','on','Name','PMPC_MF');
verifyNotEmpty(testCase, blocks);
verifyEqual(testCase, get_param(blocks{1}, 'TreatAsAtomicUnit'), 'on');
verifyEqual(testCase, get_param(blocks{1}, 'ScheduleAs'), 'Sample time');
end

function closeModels
close_system('pmpc_mil', 0);
close_system('Solver_SF', 0);
end
