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
required = {'setup_pmpc.m','cq3_2019.slx','pmpc_mil.slx','pmpc_hil.slx','simfile.sim'};
for i = 1:numel(required)
    verifyTrue(testCase, ismember(exist(fullfile(root,required{i}),'file'), [2 4]), required{i});
end
end

function testCurrentBaselineWasPreserved(testCase)
base = fullfile(testCase.TestData.root,'simulation_results','current','erd_0927_base');
verifyEqual(testCase, exist(base,'dir'), 7);
d = dir(base);
verifyEqual(testCase, sum(~[d.isdir]), 13);
end
