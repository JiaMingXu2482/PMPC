function tests = test_controller_model_mapping
tests = functiontests(localfunctions);
end

function testMapsKnownControllerTagsToFixedModels(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root); startup_pmpc();

verifyEqual(testCase, func_ControllerModelForTag('MPC'), 'mpc_mil');
verifyEqual(testCase, func_ControllerModelForTag('zeng'), 'zeng_mil');
verifyEqual(testCase, func_ControllerModelForTag('PMPC'), 'pmpc_mil');
end

function testRejectsUnknownControllerTag(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root); startup_pmpc();

verifyError(testCase, @() func_ControllerModelForTag('baseline'), ...
    'func_ControllerModelForTag:InvalidTag');
end
