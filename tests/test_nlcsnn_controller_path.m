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
% Multirate (2026-09-29): (x,v,a,h) are sampled from the 1 kHz plant.
[hp, xp, vp_, ap] = plantSample(P, u);
ctx = func_NLCSNNContext(P.Pm.NLCSNN.net, xp, vp_, ap, hp, P.Pm.NLCSNN);
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

[hp, xp, vp_, ap] = plantSample(P, u);
[sys, Sout, i_cmd] = pmpc_step(u, P.Pm, S, hp, xp, vp_, ap);

verifyTrue(testCase, all(isfinite(sys(5:8))));
verifyTrue(testCase, all(isnan(Sout.InitialParams.prevstate.s_act)));
% NLCSNN path is the active one: a finite current command was produced.
verifyTrue(testCase, all(isfinite(i_cmd)));
end

function testAllThreeModesUseNLCSNN(testCase)
modes = [2 2 1];
zeng = [0 1 0];
for k = 1:3
    [P, u] = configuredController(modes(k), zeng(k));
    S = P.S0;
    S.InitialParams.InitialGapflag = 1;
    [hp, xp, vp_, ap] = plantSample(P, u);
    [sys, Sout, ~] = pmpc_step(u, P.Pm, S, hp, xp, vp_, ap);
    % The plant owns h now; the controller keeps i_prev across ticks.
    verifyTrue(testCase, all(isfinite(hp(:))));
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
[hp, xp, vp_, ap] = plantSample(P, u);
ctx = func_NLCSNNContext(P.Pm.NLCSNN.net, xp, vp_, ap, hp, P.Pm.NLCSNN);
S = P.S0;
S.InitialParams.InitialGapflag = 1;

[sys, ~, ~] = pmpc_step(u, P.Pm, S, hp, xp, vp_, ap);
Factual = [sys(5);sys(7);sys(6);sys(8)];

% Tolerance covers the inverse solver's bisection residual (~0.014 N max).
verifyGreaterThanOrEqual(testCase, Factual, ctx.F_lo-1e-3);
verifyLessThanOrEqual(testCase, Factual, ctx.F_hi+1e-3);
end

function testQpDimensionsStayFixed(testCase)
[P, ~] = configuredController(1, 0);
verifyEqual(testCase, P.Pm.MPCParameters.Nx, 7);
verifyEqual(testCase, P.Pm.MPCParameters.Ne, 8);
verifyEqual(testCase, P.Pm.MPCParameters.Nr, 3);
verifySize(testCase, P.S0.cert, [36 1]);
end

function [hp, xp, vp_, ap] = plantSample(P, u, nSteps)
% Emulate the 1 kHz plant between two 100 Hz controller ticks, holding
% i_cmd = 0 (ZOH). Returns the tick-sampled (h,x,v,a) in controller order.
if nargin < 3, nSteps = 10; end
cmpD  = u(51:54);   % [L1;L2;R1;R2] CarSim order, [mm]
cmpRD = u(1:4);     % [L1;L2;R1;R2] CarSim order, [mm/s]
hp = zeros(8,4); afp = zeros(4,1); vpp = zeros(4,1); initp = 0;
for k = 1:nSteps
    [~, hp, afp, vpp, initp, xp, vp_, ap] = func_NLCSNNPlant4( ...
        P.Pm.NLCSNN.net, cmpD, cmpRD, zeros(4,1), ...
        hp, afp, vpp, initp, P.Pm.NLCSNN);
end
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
