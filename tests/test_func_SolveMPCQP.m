function tests = test_func_SolveMPCQP
tests = functiontests(localfunctions);
end

function testNonFiniteWarmStartIsReset(testCase)
params = struct('Nu', 1, 'Nc', 1, 'Ne', 0, 'Nr', 0);

H = 1;
f = 0;
A = zeros(0, 1);
b = zeros(0, 1);
lb = -1;
ub = 1;
warmStart = Inf;

didThrow = false;
try
    [~, exitflag, deltaU, ~, ~, nextWarmStart] = ...
        func_SolveMPCQP(H, f, A, b, lb, ub, warmStart, params);
catch
    didThrow = true;
    exitflag = NaN;
    deltaU = NaN;
    nextWarmStart = NaN;
end

verifyFalse(testCase, didThrow);
verifyEqual(testCase, exitflag, 1);
verifyEqual(testCase, deltaU, 0, 'AbsTol', 1e-12);
verifyTrue(testCase, all(isfinite(nextWarmStart)));
end

function testWarmStartWhoseConstraintEvaluationOverflowsIsReset(testCase)
params = struct('Nu', 2, 'Nc', 1, 'Ne', 0, 'Nr', 0);

H = eye(2);
f = zeros(2,1);
A = [1, 1];
b = 1;
lb = [-1e308; -1e308];
ub = [ 1e308;  1e308];
warmStart = [9e307; 9e307];

didThrow = false;
try
    [~, exitflag, deltaU, ~, ~, nextWarmStart] = ...
        func_SolveMPCQP(H, f, A, b, lb, ub, warmStart, params);
catch
    didThrow = true;
    exitflag = NaN;
    deltaU = NaN(2,1);
    nextWarmStart = NaN(2,1);
end

verifyFalse(testCase, didThrow);
verifyEqual(testCase, exitflag, 1);
verifyEqual(testCase, deltaU, zeros(2,1), 'AbsTol', 1e-12);
verifyTrue(testCase, all(isfinite(nextWarmStart)));
end
