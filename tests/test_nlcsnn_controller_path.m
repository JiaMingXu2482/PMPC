function tests = test_nlcsnn_controller_path
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testCostWeightingUsesNLCSNNBounds(testCase)
[P, u] = configuredController(2, 0);
[veh, hat] = func_StateEstimation(u, P.S0.VehiclePara);
[ctx, ~] = func_NLCSNNContext(P.Pm.NLCSNN.net, ...
    [veh.D_l1;veh.D_r1;veh.D_l2;veh.D_r2], ...
    1000*[veh.V_l1;veh.V_r1;veh.V_l2;veh.V_r2], ...
    P.S0.InitialParams.prevstate.nlcsnn, P.Pm.NLCSNN);
limits = struct('Fdu',ctx.F_hi, 'Fdl',ctx.F_lo, ...
    'Fd_center',ctx.F_center, 'external',true);

[~,~,~,~,~,~,~,dMdmax,~,~,Mdmax,Mdmin,~,Fdu,Fdl,Mdnom] = ...
    func_CostWeightingRegulation_QuadSlacks(P.Pm.MPCParameters, ...
    P.Pm.CostWeights, P.S0.Constraints, 0.2, hat, ...
    P.S0.VehiclePara, veh, limits);
Bd = 0.5*[-P.S0.VehiclePara.ldf; P.S0.VehiclePara.ldf; ...
          -P.S0.VehiclePara.ldr; P.S0.VehiclePara.ldr];

verifyEqual(testCase, Fdu, ctx.F_hi, 'AbsTol', 1e-10);
verifyEqual(testCase, Fdl, ctx.F_lo, 'AbsTol', 1e-10);
verifyEqual(testCase, Mdnom, Bd.'*ctx.F_center, 'AbsTol', 1e-9);
verifyEqual(testCase, dMdmax, Mdmax-Mdmin, 'AbsTol', 1e-9);
end

function testLegacyActuatorIsBypassed(testCase)
[P, u] = configuredController(2, 0);
S = P.S0;
S.InitialParams.InitialGapflag = 1;
S.InitialParams.prevstate.s_act(:) = NaN;
S.Constraints.dmp_act_on = 1;

[sys, Sout] = pmpc_step(u, P.Pm, S);

verifyTrue(testCase, all(isfinite(sys(5:8))));
verifyTrue(testCase, all(isnan(Sout.InitialParams.prevstate.s_act)));
verifyTrue(testCase, Sout.InitialParams.prevstate.nlcsnn.initialized);
end

function testAllThreeModesUseNLCSNN(testCase)
modes = [2 2 1];
zeng = [0 1 0];
for k = 1:3
    [P, u] = configuredController(modes(k), zeng(k));
    S = P.S0;
    S.InitialParams.InitialGapflag = 1;
    [sys, Sout] = pmpc_step(u, P.Pm, S);
    verifyTrue(testCase, Sout.InitialParams.prevstate.nlcsnn.initialized);
    verifyGreaterThanOrEqual(testCase, ...
        Sout.InitialParams.prevstate.nlcsnn.i_prev, zeros(4,1));
    verifyLessThanOrEqual(testCase, ...
        Sout.InitialParams.prevstate.nlcsnn.i_prev, ...
        P.Pm.NLCSNN.i_max*ones(4,1));
    verifyTrue(testCase, all(isfinite(sys(5:8))));
end
end

function testForceOutputStaysInsideReachableBounds(testCase)
[P, u] = configuredController(1, 0);
state0 = P.S0.InitialParams.prevstate.nlcsnn;
[veh, ~] = func_StateEstimation(u, P.S0.VehiclePara);
[ctx, ~] = func_NLCSNNContext(P.Pm.NLCSNN.net, ...
    [veh.D_l1;veh.D_r1;veh.D_l2;veh.D_r2], ...
    1000*[veh.V_l1;veh.V_r1;veh.V_l2;veh.V_r2], ...
    state0, P.Pm.NLCSNN);
S = P.S0;
S.InitialParams.InitialGapflag = 1;

[sys, ~] = pmpc_step(u, P.Pm, S);
Factual = [sys(5);sys(7);sys(6);sys(8)];

verifyGreaterThanOrEqual(testCase, Factual, ctx.F_lo-1e-6);
verifyLessThanOrEqual(testCase, Factual, ctx.F_hi+1e-6);
end

function testQpDimensionsStayFixed(testCase)
[P, ~] = configuredController(1, 0);
verifyEqual(testCase, P.Pm.MPCParameters.Nx, 7);
verifyEqual(testCase, P.Pm.MPCParameters.Ne, 8);
verifyEqual(testCase, P.Pm.MPCParameters.Nr, 3);
verifySize(testCase, P.S0.cert, [36 1]);
end

function [P, u] = configuredController(mode, zeng)
names = {'PMPC_MODE','PMPC_ZENGRHO','PMPC_VERBOSE','PMPC_NLCSNN'};
values = {mode,zeng,0,1};
for k = 1:numel(names)
    assignin('base', names{k}, values{k});
end
P = setup_pmpc();
for k = 1:numel(names)
    evalin('base', ['clear ' names{k}]);
end
u = zeros(54,1);
u(1:4) = [35;-45;55;-65];       % CmpRD [mm/s]
u(17:20) = [5000;4100;5000;4100];
u(42) = 72;                      % 20 m/s
u(49) = 72;
u(51:54) = [2;3;4;5];           % CmpD [mm]
end
