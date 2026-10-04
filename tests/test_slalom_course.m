function tests = test_slalom_course
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testSlalomNamesUseActualRoadFrictionAndTheirOwnPath(testCase)
names = {'Slalom_mu1_PMPC','Slalom_mu1_PMPC_noDelay','Slalom_mu1_ZENG'};
for k = 1:numel(names)
    mv = func_ManeuverFromName(names{k});
    verifyEqual(testCase,mv.type,2);
    verifyEqual(testCase,mv.mu,0.85,'AbsTol',1e-12);
    verifyEqual(testCase,mv.wp,'WayPoints_Type2.mat');
    verifyEqual(testCase,mv.stop_station,160);
end
end

function testControllerPathMatchesCarSimSlalomCoordinates(testCase)
W = func_WayPoints(2,[],false);
verifySize(testCase,W,[373 9]);
verifyEqual(testCase,W(1,2:3),[0,-3.6],'AbsTol',1e-6);
verifyEqual(testCase,W(101,2:3),[50,-1.173],'AbsTol',1e-4);
verifyEqual(testCase,W(201,2:3),[100,-0.6439],'AbsTol',1e-4);
verifyEqual(testCase,W(end,2:3),[185.6637,7.2],'AbsTol',1e-4);
verifyEqual(testCase,W(end,7),187.639,'AbsTol',0.05);
verifyGreaterThan(testCase,W(end,7),160+20);
end

function testSlalomMetricsDoNotSilentlyUseDlcWaypoints(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
tag = fullfile(root,'simulation_results','current','erd_0927_base','MPC');
D = func_ReadERD(tag);
D.Dataset = 'Slalom_mu1_PMPC';
actual = func_Metrics(D);
wrong = func_Metrics(D,struct('wp','WayPoints_Type1.mat'));
verifyNotEqual(testCase,actual.ey_pk,wrong.ey_pk);
end
