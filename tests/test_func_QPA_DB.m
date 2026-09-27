function tests = test_func_QPA_DB
tests = functiontests(localfunctions);
end

function testZeroWheelLoadDoesNotCreateNonFiniteQP(testCase)
vehicle = struct('lf', 1.417, 'mu', 0.85, 'tf', 1.575, ...
                 'tr', 1.575, 'rt', 0.347);
initial = struct('prevstate', struct('Tb', zeros(4,1)));
constraints = struct('Tb_max', 3000);
para = struct('Fz_l1', 4740, 'Fz_r1', 4740, ...
              'Fz_l2', 0,    'Fz_r2', 0);
tbUpper = struct('Tb_L1', 1000, 'Tb_R1', 1000, ...
                 'Tb_L2', 0,    'Tb_R2', 0);

didThrow = false;
try
    [tbFl, tbRl, tbFr, tbRr] = ...
        func_QPA_DB(vehicle, initial, constraints, para, 0, 0, tbUpper);
catch
    didThrow = true;
    tbFl = NaN; tbRl = NaN; tbFr = NaN; tbRr = NaN;
end

verifyFalse(testCase, didThrow);
verifyTrue(testCase, all(isfinite([tbFl, tbRl, tbFr, tbRr])));
verifyEqual(testCase, [tbRl, tbRr], [0, 0], 'AbsTol', 1e-12);
end
