function tests = test_controller_initializers
tests = functiontests(localfunctions);
end

function testFixedInitializersCreateMatchingParameterBundles(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();

verifyInitializer(testCase, @mil_init_MPC,  'MPC_P',  1, 6, 2, false);
verifyInitializer(testCase, @mil_init_ZENG, 'ZENG_P', 2, 7, 2, true);
verifyInitializer(testCase, @mil_init_PMPC, 'PMPC_P', 3, 7, 1, false);

evalin('base', 'clear PMPC_CONTROLLER_VARIANT PMPC_MODE PMPC_ZENGRHO PMPC_VERBOSE MPC_P ZENG_P PMPC_P NLCSNN');
end

function testLegacyInitializerClearsFixedVariantOverride(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
assignin('base', 'PMPC_CONTROLLER_VARIANT', 1);
mil_init(1);

P = evalin('base', 'PMPC_P');
verifyEqual(testCase, P.Pm.MPCParameters.ControllerVariant, 3);
verifyEqual(testCase, P.S0.Constraints.ControllerMode, 1);
evalin('base', 'clear PMPC_CONTROLLER_VARIANT PMPC_MODE PMPC_ZENGRHO PMPC_VERBOSE PMPC_P NLCSNN');
end

function verifyInitializer(testCase, initializer, bundleName, variant, nx, mode, zengOn)
initializer();

P = evalin('base', bundleName);
verifyEqual(testCase, P.Pm.MPCParameters.ControllerVariant, variant);
verifyEqual(testCase, P.Pm.MPCParameters.Nx, nx);
verifyEqual(testCase, P.S0.Constraints.ControllerMode, mode);
verifyEqual(testCase, logical(P.S0.Constraints.ZengRho_on), zengOn);
verifyEqual(testCase, evalin('base', 'PMPC_CONTROLLER_VARIANT'), variant);
end
