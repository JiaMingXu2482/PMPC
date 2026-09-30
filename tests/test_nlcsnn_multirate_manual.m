%TEST_NLCSNN_MULTIRATE  Smoke test for the multirate refactor.
%   Plant @1000 Hz (func_NLCSNNPlant4) + controller @100 Hz
%   (func_NLCSNNContext + func_NLCSNNApply).
%
%   Run in MATLAB from a directory on whose path are:
%     nlcsnn_damper_init.m, nlcsnn_damper_step.m, nlcsnn_predict_force.m,
%     nlcsnn_force_to_current.m, nlcsnn_weights.mat,
%     func_NLCSNNPlant4.m, func_NLCSNNContext.m, func_NLCSNNApply.m
%   (e.g. the PMPC repo's controller/ folder).
%
%   Prints PASS/FAIL per test. Hard failures throw.

clear; clc;

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'controller'));
addpath(fullfile(repoRoot, 'nlcsnn'));

net = nlcsnn_damper_init();
cfg = struct('x_ref', 281.0645, 'temp', 42.5, 'i_max', 1.6, ...
             'dt_plant', 0.001, 'tau_accel', 0.02);
o_c2p = [1 3 2 4];   % CarSim [L1;L2;R1;R2] -> controller [L1;R1;L2;R2]

npass = 0; nfail = 0;
function ok = check(name, cond)
    if cond
        fprintf('  [PASS] %s\n', name); ok = true;
    else
        fprintf('  [FAIL] %s\n', name); ok = false;
    end
end

fprintf('== T1: 2 s plant rollout, 10 mm / 5 Hz, i = 0.8 A ==\n');
N = 2000;
t = (0:N-1)' * 0.001;
cmpD  = 10*sin(2*pi*5*t);          % mm, CarSim order base signal
cmpRD = 10*2*pi*5*cos(2*pi*5*t);   % mm/s
cmpD  = repmat(cmpD, 1, 4);        % 4 identical corners (N x 4)
cmpRD = repmat(cmpRD, 1, 4);
h = zeros(8,4); af = zeros(4,1); vp = zeros(4,1); init = 0;
Flog = zeros(N,4);
for k = 1:N
    i_cmd = 0.8*ones(4,1);
    [Fk, h, af, vp, init, ~, ~, ~] = func_NLCSNNPlant4( ...
        net, cmpD(k,:)', cmpRD(k,:)', i_cmd, h, af, vp, init, cfg);
    Flog(k,:) = Fk';
end
if check('all finite', all(isfinite(Flog(:))) && all(isfinite(h(:))))
    npass = npass+1; else, nfail = nfail+1; end
fmax = max(abs(Flog(500:end,:)), [], 'all');   % skip transient
fprintf('       max|F| after transient = %.1f N\n', fmax);
if check('force magnitude plausible (100..20000 N)', fmax > 100 && fmax < 20000)
    npass = npass+1; else, nfail = nfail+1; end

fprintf('== T2: current direction (0 A hardest -> 1.6 A softest) ==\n');
h = zeros(8,4); af = zeros(4,1); vp = zeros(4,1); init = 0;
Fa = zeros(500,1); Fb = zeros(500,1);
for k = 1:500
    [Fk, h, af, vp, init, ~, ~, ~] = func_NLCSNNPlant4( ...
        net, cmpD(k,:)', cmpRD(k,:)', zeros(4,1), h, af, vp, init, cfg);
    Fa(k) = Fk(1);
end
h = zeros(8,4); af = zeros(4,1); vp = zeros(4,1); init = 0;
for k = 1:500
    [Fk, h, af, vp, init, ~, ~, ~] = func_NLCSNNPlant4( ...
        net, cmpD(k,:)', cmpRD(k,:)', 1.6*ones(4,1), h, af, vp, init, cfg);
    Fb(k) = Fk(1);
end
rmsA = sqrt(mean(Fa(200:end).^2)); rmsB = sqrt(mean(Fb(200:end).^2));
fprintf('       RMS force: 0 A = %.1f N, 1.6 A = %.1f N\n', rmsA, rmsB);
if check('0 A harder than 1.6 A', rmsA > rmsB)
    npass = npass+1; else, nfail = nfail+1; end

fprintf('== T3: first-step accel guard (no differentiation spike) ==\n');
h = zeros(8,4); af = zeros(4,1); vp = zeros(4,1); init = 0;
[~, ~, ~, ~, ~, ~, ~, a0] = func_NLCSNNPlant4(net, zeros(4,1), ...
    500*ones(4,1), zeros(4,1), h, af, vp, 0, cfg);   % v = 500 mm/s, init=0
[~, ~, ~, ~, ~, ~, ~, a1] = func_NLCSNNPlant4(net, zeros(4,1), ...
    500*ones(4,1), zeros(4,1), h, af, vp, 1, cfg);   % same, init=1
fprintf('       a_plant: init=0 -> %.1f, init=1 -> %.1f mm/s^2\n', a0(1), a1(1));
if check('init=0 gives ~0 accel', all(abs(a0) < 1))
    npass = npass+1; else, nfail = nfail+1; end
if check('init=1 differentiates (large a)', all(abs(a1) > 1000))
    npass = npass+1; else, nfail = nfail+1; end

fprintf('== T4: controller path (Context + Apply) on a plant sample ==\n');
% warm up plant, then emulate one 100 Hz tick with frozen inputs
h = zeros(8,4); af = zeros(4,1); vp = zeros(4,1); init = 0;
for k = 1:500
    [~, h, af, vp, init, ~, ~, ~] = func_NLCSNNPlant4( ...
        net, cmpD(k,:)', cmpRD(k,:)', 0.8*ones(4,1), h, af, vp, init, cfg);
end
cD = cmpD(500,:)'; cR = cmpRD(500,:)';
h_before = h;   % delay state BEFORE the tick step
[Fp, h_new, af_new, vp_new, ~, xp, vp_, ap] = func_NLCSNNPlant4( ...
    net, cD, cR, 0.8*ones(4,1), h, af, vp, init, cfg);
ctx = func_NLCSNNContext(net, xp, vp_, ap, h_new, cfg);
if check('F_lo <= F_center <= F_hi', ...
        all(ctx.F_lo <= ctx.F_center) && all(ctx.F_center <= ctx.F_hi))
    npass = npass+1; else, nfail = nfail+1; end
if check('bounds finite', all(isfinite([ctx.F_lo; ctx.F_hi])))
    npass = npass+1; else, nfail = nfail+1; end
F_des = ctx.F_center;   % feasible target by construction
[i_cmd, F_pred, i_prev_new] = func_NLCSNNApply(net, ctx, F_des, ...
    0.8*ones(4,1), cfg);
if check('i_cmd in [0, 1.6]', all(i_cmd >= 0) && all(i_cmd <= 1.6))
    npass = npass+1; else, nfail = nfail+1; end
err = max(abs(F_pred - F_des));
fprintf('       max|F_pred - F_des| = %.3f N\n', err);
if check('inverse tracks feasible target (< 5 N)', err < 5)
    npass = npass+1; else, nfail = nfail+1; end
if check('i_prev passthrough', isequal(i_prev_new, i_cmd))
    npass = npass+1; else, nfail = nfail+1; end

fprintf('== T5: plant force == predict_force at the same (x,v,a,h,i) ==\n');
% The plant's per-step force must be exactly the training-order
% predict-then-advance: F_plant(tick) == -predict_force(x,v,a,i,h_before).
% This is the invariant the controller's inverse model relies on.
Fp_ctl = Fp(o_c2p);   % to controller order
Fref = zeros(4,1);
for j = 1:4
    Fref(j) = -nlcsnn_predict_force(net, xp(j), vp_(j), ap(j), ...
                                    0.8, cfg.temp, h_before(:,j));
end
herr = max(abs(Fp_ctl - Fref));
fprintf('       max|F_plant - (-predict_force)| = %.3e N\n', herr);
if check('plant/controller speak the same force (< 1e-6 N)', herr < 1e-6)
    npass = npass+1; else, nfail = nfail+1; end

fprintf('== T6: interface dimensions ==\n');
ok = isequal(size(Fp),[4 1]) && isequal(size(h_new),[8 4]) && ...
     isequal(size(xp),[4 1]) && isequal(size(vp_),[4 1]) && ...
     isequal(size(ap),[4 1]) && isequal(size(i_cmd),[4 1]) && ...
     isequal(size(F_pred),[4 1]);
if check('all ports correctly sized', ok)
    npass = npass+1; else, nfail = nfail+1; end

fprintf('\n%d passed, %d failed\n', npass, nfail);
if nfail > 0, error('test_nlcsnn_multirate: %d test(s) failed', nfail); end
