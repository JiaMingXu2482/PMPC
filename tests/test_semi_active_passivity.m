function tests = test_semi_active_passivity
%TEST_SEMI_ACTIVE_PASSIVITY Contract for the physical damper safety layer.
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root, 'controller'));
end

function testProjectionNeverInjectsEnergy(testCase)
% Removing the cone projection would allow F*CmpRD < 0 and fail here.
cfg = local_config();
Fraw = [100; -200; 80; -120];
cmpRD = [20; 30; -40; -50];

Fsafe = func_SemiActivePassivity(Fraw, cmpRD, cfg);

verifyGreaterThanOrEqual(testCase, Fsafe .* cmpRD, -1e-9*ones(4,1));
verifyGreaterThan(testCase, Fsafe(1), 90);
verifyGreaterThanOrEqual(testCase, Fsafe(2)*cmpRD(2), ...
    cfg.c_min*cmpRD(2)^2 - 1e-6);
verifyGreaterThanOrEqual(testCase, Fsafe(3)*cmpRD(3), ...
    cfg.c_min*cmpRD(3)^2 - 1e-6);
verifyLessThan(testCase, Fsafe(4), -110);
end

function testProjectionIsContinuousAtZeroVelocity(testCase)
% A hard sign switch would create a force jump near a velocity reversal.
cfg = local_config();
Fraw = 250 * ones(4,1);
cmpRD = [-1e-6; 0; 1e-6; 0];

Fsafe = func_SemiActivePassivity(Fraw, cmpRD, cfg);

verifyTrue(testCase, all(isfinite(Fsafe)));
verifyEqual(testCase, Fsafe(2), 0, 'AbsTol', 1e-12);
verifyGreaterThanOrEqual(testCase, Fsafe .* cmpRD, -1e-12*ones(4,1));
verifyLessThan(testCase, max(abs(Fsafe)), 1e-3);
end

function testProjectionPreservesDissipativeForceAwayFromZero(testCase)
% An over-aggressive guard must not erase normal damping at useful speed.
cfg = local_config();
Fraw = [120; -120; 120; -120];
cmpRD = [100; -100; 100; -100];

Fsafe = func_SemiActivePassivity(Fraw, cmpRD, cfg);

verifyEqual(testCase, Fsafe, [120; -120; 120; -120], ...
    'AbsTol', 0.2);
end

function testProjectionKeepsSoftDamperDampingWhenNetworkTurnsActive(testCase)
% Outside the 1.66 Hz training range the network can predict negative
% damping.  The safety layer must then fall back to the physical soft-state
% damping instead of making the suspension effectively undamped.
cfg = local_config();
Fraw = [-100; 100; -250; 250];
cmpRD = [200; -200; 50; -50];

Fsafe = func_SemiActivePassivity(Fraw, cmpRD, cfg);

floorForce = cfg.c_min .* cmpRD;
verifyGreaterThanOrEqual(testCase, Fsafe .* cmpRD, ...
    floorForce .* cmpRD - 1e-6*ones(4,1));
verifyEqual(testCase, Fsafe, floorForce, 'AbsTol', 0.5);
end

function cfg = local_config()
cfg = struct('v_eps', 5, 'power_eps', 50, 'c_min', 0.8);
end
