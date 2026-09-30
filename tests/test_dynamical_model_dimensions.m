function tests = test_dynamical_model_dimensions
tests = functiontests(localfunctions);
end

function testPredictionModelMatchesVariantStateDimension(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();

for variant = [1 2 3]
    assignin('base', 'PMPC_CONTROLLER_VARIANT', variant);
    assignin('base', 'PMPC_VERBOSE', 0);
    P = setup_pmpc();
    M = P.Pm.MPCParameters;
    measured = struct('x_dot', 20, 'delta_f', 0.02);
    S = func_DynamicalModel(P.S0.VehiclePara, M, measured, P.Pm.DiscreteModle);

    verifySize(testCase, S.C_aug, [M.Ny M.Nx + M.Nu]);
    verifySize(testCase, S.A_aug, [M.Nx + M.Nu, M.Nx + M.Nu, M.Np]);
    verifySize(testCase, S.B_aug, [M.Nx + M.Nu, M.Nu, M.Np]);
    verifySize(testCase, S.D_aug, [M.Nx + M.Nu, 3, M.Np]);
end

evalin('base', 'clear PMPC_CONTROLLER_VARIANT PMPC_VERBOSE');
end
