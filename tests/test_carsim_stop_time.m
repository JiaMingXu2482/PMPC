function tests = test_carsim_stop_time
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testUsesLastExpandedStopTime(testCase)
text = sprintf('TSTOP 10\nSV_VXS 90\nTSTOP 20\n');
verifyEqual(testCase,func_CarSimStopTime(text),20);
end

function testMissingStopTimeLeavesModelDefault(testCase)
verifyTrue(testCase,isnan(func_CarSimStopTime('SV_VXS 90')));
end

function testParsesCarSimWindowsLineEndings(testCase)
text = sprintf('SV_VXS 90\r\nTSTOP 20\r\n');
verifyEqual(testCase,func_CarSimStopTime(text),20);
end
