function tests = test_pmpc_longcoord_preflight
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testPreflightChecksRealRunsAndIsolatedOutputs(testCase)
cases = compare_pmpc_longcoord('preflight');
verifyEqual(testCase,numel(cases),8);
verifyEqual(testCase,numel(unique({cases.outputPrefix})),8);
for k=1:numel(cases)
    verifyEqual(testCase,cases(k).initialKmh,80,'AbsTol',1e-9);
    verifyEqual(testCase,cases(k).expandedKmh,80,'AbsTol',1e-9);
    verifyEqual(testCase,cases(k).model, ...
        ternary(strcmp(cases(k).tag,'ZENG'),'zeng_mil','pmpc_mil'));
    verifyFalse(testCase,contains(lower(cases(k).outputPrefix), ...
        lower('CarSim2019.0_Data\Results')));
    verifyTrue(testCase,exist(cases(k).expandedPar,'file')==2);
end
end

function value = ternary(cond,yes,no)
if cond, value=yes; else, value=no; end
end
