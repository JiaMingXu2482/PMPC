function tests = test_nlcsnn_adapter
tests = functiontests(localfunctions);
end

function testPredictorMatchesStepStartForce(testCase)
[net, cfg, state] = fixture();
x = 276.2;
v = -37.0;
a = 125.0;
curr = 0.8;
h = linspace(-0.2, 0.2, 8)';

F_predict = nlcsnn_predict_force(net, x, v, a, curr, cfg.temp, h);
[F_step, ~] = nlcsnn_damper_step(net, x, v, a, curr, 0, ...
    cfg.temp, cfg.dt, h);

verifyEqual(testCase, F_predict, F_step, 'AbsTol', 1e-9);
verifySize(testCase, state.h, [8 4]);
end

function testCompressionMapping(testCase)
[net, cfg, state] = fixture();
cmpD = [1; 2; 3; 4];
cmpRD = [10; -20; 30; -40];

[ctx, state] = func_NLCSNNContext(net, cmpD, cmpRD, state, cfg);

verifyEqual(testCase, ctx.x, cfg.x_ref - cmpD, 'AbsTol', 0);
verifyEqual(testCase, ctx.v, -cmpRD, 'AbsTol', 0);
verifyEqual(testCase, ctx.a, zeros(4,1), 'AbsTol', 0);
verifyLessThanOrEqual(testCase, ctx.F_lo, ctx.F_hi);
verifyEqual(testCase, ctx.F_center, 0.5*(ctx.F_lo+ctx.F_hi), ...
    'AbsTol', 1e-12);
verifyTrue(testCase, state.initialized);
end

function testCornerStatesAreIndependent(testCase)
[net, cfg, stateA] = fixture();
[ctxA, stateA] = func_NLCSNNContext(net, zeros(4,1), ...
    [15; -25; 35; -45], stateA, cfg);
stateB = stateA;
stateB.h(:,2) = linspace(0.01, 0.08, 8)';
ctxB = func_NLCSNNContext(net, zeros(4,1), ...
    [15; -25; 35; -45], stateB, cfg);
Fdes = ctxA.F_center;

[~, ~, outA] = func_NLCSNNApply(net, ctxA, Fdes, stateA, cfg);
[~, ~, outB] = func_NLCSNNApply(net, ctxB, Fdes, stateB, cfg);

verifyEqual(testCase, outA.h(:,[1 3 4]), outB.h(:,[1 3 4]), ...
    'AbsTol', 1e-12);
verifyNotEqual(testCase, outA.h(:,2), outB.h(:,2));
end

function testCurrentIsBounded(testCase)
[net, cfg, state] = fixture();
[ctx, state] = func_NLCSNNContext(net, zeros(4,1), ...
    [80; -80; 120; -120], state, cfg);
Fdes = [1e9; -1e9; 1e9; -1e9];

[Factual, icmd] = func_NLCSNNApply(net, ctx, Fdes, state, cfg);

verifyGreaterThanOrEqual(testCase, icmd, zeros(4,1));
verifyLessThanOrEqual(testCase, icmd, cfg.i_max*ones(4,1));
verifyTrue(testCase, all(isfinite(Factual)));
end

function testNearZeroAuthorityIsFinite(testCase)
[net, cfg, state] = fixture();
[ctx, state] = func_NLCSNNContext(net, zeros(4,1), zeros(4,1), ...
    state, cfg);

[Factual, icmd, state] = func_NLCSNNApply(net, ctx, ...
    1e6*ones(4,1), state, cfg);

verifyTrue(testCase, all(isfinite(Factual)));
verifyTrue(testCase, all(isfinite(icmd)));
verifyTrue(testCase, all(isfinite(state.h(:))));
end

function testNonFiniteAccelerationFallsBackSafely(testCase)
[net, cfg, state] = fixture();
[~, state] = func_NLCSNNContext(net, zeros(4,1), ...
    [1; 2; 3; 4], state, cfg);

[ctx, state] = func_NLCSNNContext(net, [NaN; 2; Inf; 4], ...
    [NaN; 5; Inf; 7], state, cfg);
[Factual, icmd, state] = func_NLCSNNApply(net, ctx, ...
    ctx.F_center, state, cfg);

verifyTrue(testCase, all(isfinite(ctx.x)));
verifyTrue(testCase, all(isfinite(ctx.v)));
verifyTrue(testCase, all(isfinite(ctx.a)));
verifyTrue(testCase, all(isfinite(Factual)));
verifyTrue(testCase, all(isfinite(icmd)));
verifyTrue(testCase, all(isfinite(state.h(:))));
end

function [net, cfg, state] = fixture()
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'controller'));
addpath(fullfile(root, 'nlcsnn'));
net = nlcsnn_damper_init(fullfile(root, 'nlcsnn', 'nlcsnn_weights.mat'));
cfg = struct('dt', 0.01, 'temp', 42.5, 'i_max', 1.6, ...
    'x_ref', 281.0645, 'tau_accel', 0.02);
state = struct('h', zeros(8,4), 'i_prev', zeros(4,1), ...
    'v_prev', zeros(4,1), 'a_filt', zeros(4,1), ...
    'initialized', false);
end
