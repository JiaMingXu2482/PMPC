function tests = test_damper_delay_tau
tests = functiontests(localfunctions);
end

function testReturnsFixedDelayForValidFourCornerInputs(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);

tau = func_DamperDelayTau([0.2;-0.3;0.4;-0.5], ...
    [0.1;0.2;0.3;0.4], [1;-1;1;-1], 0.015);

verifyEqual(testCase, tau, 0.015, 'AbsTol', 1e-12);
end

function testRejectsInvalidCornerVectorSizes(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);

verifyError(testCase, @() func_DamperDelayTau(zeros(3,1), ...
    zeros(4,1), zeros(4,1), 0.015), 'func_DamperDelayTau:InvalidInput');
end

function testRejectsNonpositiveFixedDelay(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);

verifyError(testCase, @() func_DamperDelayTau(zeros(4,1), ...
    zeros(4,1), zeros(4,1), 0), 'func_DamperDelayTau:InvalidDelay');
end

function testRejectsInvalidCurrentDirection(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);

verifyError(testCase, @() func_DamperDelayTau(zeros(4,1), ...
    zeros(4,1), [1;0;2;-1], 0.015), 'func_DamperDelayTau:InvalidInput');
end
