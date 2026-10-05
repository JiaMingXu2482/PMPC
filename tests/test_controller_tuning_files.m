function tests = test_controller_tuning_files
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function teardownOnce(~)
evalin('base', 'clear PMPC_CONTROLLER_VARIANT PMPC_MODE PMPC_ZENGRHO PMPC_VERBOSE PMPC_P MPC_P ZENG_P NLCSNN');
end

function testEachControllerHasItsOwnEditableTuningBundle(testCase)
names = {'Atuning_mpc','Atuning_zeng','Atuning_pmpc'};
labels = {'MPC','ZENG','PMPC'};
for k = 1:3
    T = feval(names{k});
    verifyEqual(testCase,T.controller,labels{k});
verifyEqual(testCase,[T.Ts,T.Np,T.Nc],[0.1,10,5]);
    verifyEqual(testCase,[T.Q5,T.Q6,T.R1,T.R2,T.R3,T.V2], ...
        [2e4,2e2,1,10,5,8e4]);
    verifyEqual(testCase,[T.W3,T.W4],[5e6,5e6]);
end
verifyFalse(testCase,isfield(Atuning_zeng(),'ZengLong_on'));
verifyFalse(testCase,isfield(Atuning_pmpc(),'LongCoordMode'));
verifyEqual(testCase,Atuning_pmpc().prio_vartheta,2);
end

function testInitializersUseBundlesAndKeepDefaults(testCase)
init = {@mil_init_MPC,@mil_init_ZENG,@mil_init_PMPC};
for k = 1:3
    P = init{k}();
    verifyEqual(testCase,P.Pm.MPCParameters.ControllerVariant,k);
    verifyEqual(testCase,[P.Pm.MPCParameters.Ts, ...
        P.Pm.MPCParameters.Np,P.Pm.MPCParameters.Nc],[0.1,10,5]);
    verifyEqual(testCase,[P.Pm.CostWeights.Q5,P.Pm.CostWeights.Q6, ...
        P.Pm.CostWeights.V2],[2e4,2e2,8e4]);
end
verifyFalse(testCase,isfield(P.S0.Constraints,'LongCoordMode'));
verifyFalse(testCase,isfield(P.S0,'LongCoord'));
end
