function tests = test_sim_model_selection
tests = functiontests(localfunctions);
end

function testAcceptsExplicitMatchingModel(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root); startup_pmpc();
parFile = writePar('mpc_mil');
cleanup = onCleanup(@() delete(parFile)); %#ok<NASGU>

verifyEqual(testCase, func_SimModel(parFile, 'mpc_mil'), 'mpc_mil');
end

function testRejectsMismatchedExplicitModel(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root); startup_pmpc();
parFile = writePar('zeng_mil');
cleanup = onCleanup(@() delete(parFile)); %#ok<NASGU>

verifyError(testCase, @() func_SimModel(parFile, 'pmpc_mil'), ...
    'func_SimModel:WrongModel');
end

function parFile = writePar(model)
parFile = [tempname '.par'];
fid = fopen(parFile, 'w');
assert(fid ~= -1, 'Unable to create temporary Run_all.par fixture.');
fprintf(fid, 'SIMULINK_MODEL_FILE %s.slx\n', model);
fclose(fid);
end
