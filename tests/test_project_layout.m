function tests = test_project_layout
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
testCase.TestData.root = startup_pmpc();
end

function testRuntimeFilesResolveFromOrganizedFolders(testCase)
root = testCase.TestData.root;
verifyEqual(testCase, which('pmpc_step'), fullfile(root,'controller','pmpc_step.m'));
verifyEqual(testCase, which('func_SolveMPCQP'), fullfile(root,'controller','func_SolveMPCQP.m'));
verifyEqual(testCase, which('run_ds'), fullfile(root,'scripts','run_ds.m'));
verifyEqual(testCase, which('TireCarpet_265_75R16.mat'), ...
    fullfile(root,'data','TireCarpet_265_75R16.mat'));
end

function testRootKeepsCarSimEntryFiles(testCase)
root = testCase.TestData.root;
required = {'setup_pmpc.m','pmpc_mil.slx','simfile.sim'};
for i = 1:numel(required)
    verifyTrue(testCase, ismember(exist(fullfile(root,required{i}),'file'), [2 4]), required{i});
end
verifyEqual(testCase, exist(fullfile(root,'cq3_2019.slx'),'file'), 0);
verifyEqual(testCase, exist(fullfile(root,'pmpc_hil.slx'),'file'), 0);
end

function testCurrentBaselineWasPreserved(testCase)
base = fullfile(testCase.TestData.root,'simulation_results','current','erd_0927_base');
verifyEqual(testCase, exist(base,'dir'), 7);
d = dir(base);
verifyEqual(testCase, sum(~[d.isdir]), 13);
end

function testDeadControllerFilesAreAbsent(testCase)
root = testCase.TestData.root;
dead = {'func_bezierInterp.m','func_FindBezierControlPointsND.m', ...
    'func_RLSFilter_Calpha_f.m','func_RLSFilter_Calpha_r.m', ...
    'func_tire_init_Calpha.m','func_Fz_Calpha.m'};
for i = 1:numel(dead)
    verifyEqual(testCase, exist(fullfile(root,'controller',dead{i}),'file'), 0, dead{i});
end
end

function testDeadParameterFieldsAreAbsent(testCase)
P = setup_pmpc();
dead = {'C_table','Cf0','Cr0'};
for i = 1:numel(dead)
    verifyFalse(testCase, isfield(P,dead{i}), dead{i});
    verifyFalse(testCase, isfield(P.Pm,dead{i}), ['Pm.' dead{i}]);
end
stepText = fileread(fullfile(testCase.TestData.root,'controller','pmpc_step.m'));
verifyFalse(testCase, contains(stepText,'Pm.C_table'));
verifyFalse(testCase, contains(stepText,'Pm.Cf0'));
verifyFalse(testCase, contains(stepText,'Pm.Cr0'));
end

function testSetupHelpersAreLocal(testCase)
root = testCase.TestData.root;
verifyEqual(testCase, exist(fullfile(root,'controller','func_InitialParams.m'),'file'), 0);
verifyEqual(testCase, exist(fullfile(root,'controller','wsget.m'),'file'), 0);

assignin('base','PMPC_TS',0.04);
cleanup = onCleanup(@() evalin('base','clear PMPC_TS PMPC_P')); %#ok<NASGU>
P = setup_pmpc();
verifyEqual(testCase, P.Pm.MPCParameters.Ts, 0.04, 'AbsTol', 1e-12);
verifyTrue(testCase, isfield(P.S0.InitialParams,'InitialGapflag'));
verifyTrue(testCase, isfield(P.S0.InitialParams,'prevstate'));
verifySize(testCase, P.S0.InitialParams.t_solve, [1 20000]);
verifySize(testCase, P.S0.InitialParams.ey_hist, [1 20000]);
verifySize(testCase, P.S0.InitialParams.epsi_hist, [1 20000]);
end
