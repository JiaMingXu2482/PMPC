function tests = test_nlcsnn_configuration
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testStartupAddsNLCSNNPath(testCase)
verifyEqual(testCase, exist('nlcsnn_damper_step', 'file'), 2);
end

function testSetupLoadsFixedConfigurationAndState(testCase)
backup = local_backup('PMPC_NLCSNN');
cleanup = onCleanup(@() local_restore(backup)); %#ok<NASGU>
assignin('base', 'PMPC_NLCSNN', 1);
P = setup_pmpc();

verifyTrue(testCase, P.Pm.NLCSNN.enabled);
verifyEqual(testCase, P.Pm.NLCSNN.i_max, 1.6, 'AbsTol', 0);
verifyEqual(testCase, P.Pm.NLCSNN.temp, 42.5, 'AbsTol', 0);
verifyEqual(testCase, P.Pm.NLCSNN.x_ref, 281.0645, 'AbsTol', 0);
verifySize(testCase, P.Pm.NLCSNN.net.Wx1, [256 63]);
verifySize(testCase, P.S0.InitialParams.prevstate.nlcsnn.h, [8 4]);
verifySize(testCase, P.S0.InitialParams.prevstate.nlcsnn.i_prev, [4 1]);
verifySize(testCase, P.S0.InitialParams.prevstate.nlcsnn.v_prev, [4 1]);
verifySize(testCase, P.S0.InitialParams.prevstate.nlcsnn.a_filt, [4 1]);
verifyFalse(testCase, P.S0.InitialParams.prevstate.nlcsnn.initialized);
end

function testStateEstimationAppendsDisplacementWithoutRenumbering(testCase)
P = setup_pmpc();
u = (1:54)';
[veh, hat] = func_StateEstimation(u, P.S0.VehiclePara);

verifyEqual(testCase, [veh.D_l1; veh.D_l2; veh.D_r1; veh.D_r2], ...
    (51:54)', 'AbsTol', 0);
verifyEqual(testCase, [veh.V_l1; veh.V_l2; veh.V_r1; veh.V_r2], ...
    (1:4)'/1000, 'AbsTol', 0);
verifyEqual(testCase, hat.LTR, 50, 'AbsTol', 0);
end

function testPmpcBlockRejectsLegacyInputSize(testCase)
P = setup_pmpc();
verifyError(testCase, @() pmpc_block(zeros(50,1), P), ...
    'pmpc_block:InvalidInputSize');
end

function backup = local_backup(name)
backup = struct('name', name, ...
    'exists', evalin('base', sprintf('exist(''%s'',''var'')', name)) == 1, ...
    'value', []);
if backup.exists
    backup.value = evalin('base', name);
end
end

function local_restore(backup)
if backup.exists
    assignin('base', backup.name, backup.value);
else
    evalin('base', ['clear ' backup.name]);
end
end
