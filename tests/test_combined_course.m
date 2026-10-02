function tests = test_combined_course
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testDlcJoinsShortStraightAndJTurn(testCase)
C = func_CombinedCourse(69);
W = func_WayPoints(6,69,false);
D = func_WayPoints(1,69,false);
verifyEqual(testCase,C.turn_x-C.dlc_end_x,80);
verifyEqual(testCase,W(1:size(D,1),2:3),D(:,2:3),'AbsTol',1e-10);
verifyTrue(testCase,all(diff(W(:,7))>0));
atSwitch = find(W(:,2)>=C.mu_switch_x,1);
atTurn = find(W(:,2)>=C.turn_x,1);
verifyLessThan(testCase,abs(W(atSwitch,3)),0.01);
verifyLessThan(testCase,abs(W(atTurn,3)),0.01);
verifyLessThan(testCase,W(atTurn,7)-W(atSwitch,7),55);
verifyLessThan(testCase,abs(W(end,2)-(C.turn_x+69)),1e-8);
end

function testCombinedNameSelectsTwoRoadFrictions(testCase)
for tag = {'MPC','ZENG','PMPC'}
    M = func_ManeuverFromName( ...
        ['COMB90_DLC05_JT_R69_mu0.5to0.85_' tag{1}]);
    verifyTrue(testCase,M.combined);
    verifyEqual(testCase,M.type,6);
    verifyEqual(testCase,M.mu,0.5);
    verifyEqual(testCase,M.mu_high,0.85);
    verifyEqual(testCase,M.mu_switch_x,230);
end
end

function testSharedTurnSpeedSchedule(testCase)
C = func_CombinedCourse(69);
verifyEqual(testCase,C.turn_target_kmh,80);
[target,force] = func_CombinedSpeedReference(C,195,88/3.6,90,1860);
verifyEqual(testCase,target,90);
verifyEqual(testCase,force,0);
[target,force] = func_CombinedSpeedReference(C,205,88/3.6,90,1860);
verifyGreaterThan(testCase,target,80);
verifyLessThan(testCase,target,90);
verifyGreaterThan(testCase,force,0);
[target,force] = func_CombinedSpeedReference(C,220,88/3.6,90,1860);
verifyEqual(testCase,target,80);
verifyLessThanOrEqual(testCase,force,1860*1.5+1e-9);
[target,force] = func_CombinedSpeedReference(C,220,79/3.6,90,1860);
verifyEqual(testCase,target,80);
verifyEqual(testCase,force,0);
end

function testDefaultCombinedCourseUsesR70(testCase)
C = func_CombinedCourse();
W = func_WayPoints(6,C.turn_radius,false);
verifyEqual(testCase,C.turn_radius,70);
verifyEqual(testCase,W(end,2:3),[350 170],'AbsTol',1e-8);
verifyEqual(testCase,C.turn_x-C.dlc_end_x,80);
end

function testThreeDefaultExportsAreDistinctR70Runs(testCase)
names = {'mpc_mil','zeng_mil','pmpc_mil'};
for j=1:numel(names)
    S = build_combined_carsim(names{j});
    verifyEqual(testCase,S.course.turn_radius,70);
    verifyTrue(testCase,contains(S.name,'_JT_R70_'));
    verifyTrue(testCase,contains(S.parfile,'Run_Combined90_R70_'));
    verifyTrue(testCase,isfile(S.parfile));
    verifyTrue(testCase,isfile(S.pathCsv));
    path = readmatrix(S.pathCsv);
    verifyEqual(testCase,path(1,1:2),[0 0],'AbsTol',1e-8);
    verifyEqual(testCase,path(end,1:2),[350 170],'AbsTol',1e-8);
    raw = fileread(S.parfile);
    verifyTrue(testCase,contains(raw,S.name));
    verifyTrue(testCase,contains(raw,'MU_ROAD_CARPET 2D_STEP'));
end
end
