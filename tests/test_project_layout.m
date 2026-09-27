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

P = setup_pmpc();
verifyTrue(testCase, isfield(P.S0.InitialParams,'InitialGapflag'));
verifyTrue(testCase, isfield(P.S0.InitialParams,'prevstate'));
verifySize(testCase, P.S0.InitialParams.t_solve, [1 20000]);
verifySize(testCase, P.S0.InitialParams.ey_hist, [1 20000]);
verifySize(testCase, P.S0.InitialParams.epsi_hist, [1 20000]);
end

function testBaseWorkspaceOverrideWhenDatasetDoesNotDefineIt(testCase)
par = fullfile(func_CarSimResDir(), 'Run_all.par');
if exist(par,'file') == 2
    token = regexp(fileread(par), ...
        '(?m)^\s*(?:DEFINE_PARAMETER\s+)?PMPC_TS\s*=', 'once');
    assumeTrue(testCase, isempty(token), ...
        'Current CarSim dataset defines PMPC_TS and correctly has precedence.');
end

hadTs = evalin('base','exist(''PMPC_TS'',''var'')') == 1;
oldTs = [];
if hadTs, oldTs = evalin('base','PMPC_TS'); end
cleanup = onCleanup(@() restoreBaseVar('PMPC_TS',hadTs,oldTs)); %#ok<NASGU>
assignin('base','PMPC_TS',0.04);
P = setup_pmpc();
verifyEqual(testCase, P.Pm.MPCParameters.Ts, 0.04, 'AbsTol', 1e-12);
end

function testUtilitiesLiveOutsideController(testCase)
root = testCase.TestData.root;
lib = fullfile(root,'scripts','lib');
verifyEqual(testCase, which('func_ProjectRoot'), fullfile(lib,'func_ProjectRoot.m'));
verifyEqual(testCase, which('func_RunMode'), fullfile(lib,'func_RunMode.m'));
verifyEqual(testCase, which('func_Metrics'), fullfile(lib,'func_Metrics.m'));
verifyEqual(testCase, func_ProjectRoot(), root);

utilities = {'func_CarSimLib.m','func_CarSimResDir.m','func_CarSimRunning.m', ...
    'func_ErdDir.m','func_ManeuverFromName.m','func_Metrics.m', ...
    'func_ProjectRoot.m','func_ReadERD.m','func_RunMode.m', ...
    'func_SimModel.m','func_WaitERD.m'};
for i = 1:numel(utilities)
    verifyEqual(testCase, exist(fullfile(root,'controller',utilities{i}),'file'), 0, utilities{i});
    verifyEqual(testCase, exist(fullfile(lib,utilities{i}),'file'), 2, utilities{i});
end
end

function restoreBaseVar(name, existed, value)
if existed
    assignin('base',name,value);
else
    evalin('base',['clear ' name]);
end
end
