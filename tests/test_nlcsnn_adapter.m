function tests = test_nlcsnn_adapter
tests = functiontests(localfunctions);
end

function testPredictorMatchesStepStartForce(testCase)
[net, cfg] = fixture();
x = 276.2;
v = -37.0;
a = 125.0;
curr = 0.8;
h = linspace(-0.2, 0.2, 8)';

F_predict = nlcsnn_predict_force(net, x, v, a, curr, cfg.temp, h);
[F_step, ~] = nlcsnn_damper_step(net, x, v, a, curr, 0, ...
    cfg.temp, cfg.dt_plant, h);

verifyEqual(testCase, F_predict, F_step, 'AbsTol', 1e-9);
end

function testPlantKinematicsMapping(testCase)
% Multirate (2026-09-29): the plant owns the CarSim->NLCSNN kinematics and
% the first-step accel guard; the controller only reads (x,v,a,h).
[net, cfg] = fixture();
cmpD = [1; 2; 3; 4];        % CarSim order [L1;L2;R1;R2]
cmpRD = [10; -20; 30; -40]; % CarSim order, [mm/s]

[~, ~, ~, ~, ~, xp, vp_, ap] = func_NLCSNNPlant4(net, cmpD, cmpRD, ...
    zeros(4,1), zeros(8,4), zeros(4,1), zeros(4,1), 0, cfg);

o = [1 3 2 4];  % CarSim -> controller [L1;R1;L2;R2]
verifyEqual(testCase, xp, cfg.x_ref - cmpD(o), 'AbsTol', 0);
verifyEqual(testCase, vp_, -cmpRD(o), 'AbsTol', 0);
verifyEqual(testCase, ap, zeros(4,1), 'AbsTol', 0);  % no spike, init = 0

ctx = func_NLCSNNContext(net, xp, vp_, ap, zeros(8,4), cfg);
verifyLessThanOrEqual(testCase, ctx.F_lo, ctx.F_hi);
verifyEqual(testCase, ctx.F_center, 0.5*(ctx.F_lo+ctx.F_hi), ...
    'AbsTol', 1e-12);
end

function testPlantCornerStatesAreIndependent(testCase)
% The per-corner independence property moved with h into the plant.
[net, cfg] = fixture();
cmpD = zeros(4,1);
cmpRD = [15; -25; 35; -45];   % CarSim order
hA = zeros(8,4);
hB = zeros(8,4);
% NOTE: plant h is controller order [L1;R1;L2;R2]; perturb corner 2 (R1).
hB(:,2) = linspace(0.01, 0.08, 8)';
for k = 1:10
    [~, hA, ~, ~, ~, ~, ~, ~] = func_NLCSNNPlant4(net, cmpD, cmpRD, ...
        zeros(4,1), hA, zeros(4,1), zeros(4,1), 1, cfg);
    [~, hB, ~, ~, ~, ~, ~, ~] = func_NLCSNNPlant4(net, cmpD, cmpRD, ...
        zeros(4,1), hB, zeros(4,1), zeros(4,1), 1, cfg);
end

verifyEqual(testCase, hA(:,[1 3 4]), hB(:,[1 3 4]), 'AbsTol', 1e-12);
verifyNotEqual(testCase, hA(:,2), hB(:,2));
end

function testCurrentIsBounded(testCase)
[net, cfg] = fixture();
[~, hp, ~, ~, ~, xp, vp_, ap] = func_NLCSNNPlant4(net, zeros(4,1), ...
    [80; -80; 120; -120], zeros(4,1), ...
    zeros(8,4), zeros(4,1), zeros(4,1), 0, cfg);
ctx = func_NLCSNNContext(net, xp, vp_, ap, hp, cfg);
Fdes = [1e9; -1e9; 1e9; -1e9];

[i_cmd, F_pred, ~] = func_NLCSNNApply(net, ctx, Fdes, zeros(4,1), cfg);

verifyGreaterThanOrEqual(testCase, i_cmd, zeros(4,1));
verifyLessThanOrEqual(testCase, i_cmd, cfg.i_max*ones(4,1));
verifyTrue(testCase, all(isfinite(F_pred)));
end

function testNearZeroAuthorityIsFinite(testCase)
[net, cfg] = fixture();
[~, hp, ~, ~, ~, xp, vp_, ap] = func_NLCSNNPlant4(net, zeros(4,1), ...
    zeros(4,1), zeros(4,1), ...
    zeros(8,4), zeros(4,1), zeros(4,1), 0, cfg);
ctx = func_NLCSNNContext(net, xp, vp_, ap, hp, cfg);

[i_cmd, F_pred, i_prev_new] = func_NLCSNNApply(net, ctx, ...
    1e6*ones(4,1), zeros(4,1), cfg);

verifyTrue(testCase, all(isfinite(F_pred)));
verifyTrue(testCase, all(isfinite(i_cmd)));
verifyTrue(testCase, all(isfinite(i_prev_new)));
end

function testNonFiniteInputsFallBackSafely(testCase)
[net, cfg] = fixture();
[Fp, hp, ~, ~, ~, xp, vp_, ap] = func_NLCSNNPlant4(net, ...
    [NaN; 2; Inf; 4], [NaN; 5; Inf; 7], zeros(4,1), ...
    zeros(8,4), zeros(4,1), zeros(4,1), 0, cfg);
ctx = func_NLCSNNContext(net, xp, vp_, ap, hp, cfg);
[i_cmd, F_pred, ~] = func_NLCSNNApply(net, ctx, ctx.F_center, ...
    zeros(4,1), cfg);

verifyTrue(testCase, all(isfinite(xp)));
verifyTrue(testCase, all(isfinite(vp_)));
verifyTrue(testCase, all(isfinite(ap)));
verifyTrue(testCase, all(isfinite(Fp)));
verifyTrue(testCase, all(isfinite(hp(:))));
verifyTrue(testCase, all(isfinite(F_pred)));
verifyTrue(testCase, all(isfinite(i_cmd)));
end

function [net, cfg] = fixture()
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'controller'));
addpath(fullfile(root, 'nlcsnn'));
net = nlcsnn_damper_init(fullfile(root, 'nlcsnn', 'nlcsnn_weights.mat'));
cfg = struct('dt_plant', 0.001, 'temp', 42.5, 'i_max', 1.6, ...
    'x_ref', 281.0645, 'tau_accel', 0.02);
end
