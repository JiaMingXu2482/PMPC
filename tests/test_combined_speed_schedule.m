function tests = test_combined_speed_schedule
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testAllVariantsRequest80BeforeCombinedTurn(testCase)
for variant = [1 2 3]
    assignin('base','PMPC_CONTROLLER_VARIANT',variant);
    assignin('base','PMPC_VERBOSE',0);
    P = setup_pmpc();
    P.Pm.RoadMu.enabled = true;
    P.Pm.Reftraj = func_WayPoints(6,80,false);
    S = P.S0;
    S.InitialParams.InitialGapflag = 1;
    u = controllerInput(90,220);

    [sys,~,~] = pmpc_step(u,P.Pm,S,zeros(8,4),zeros(4,1), ...
        zeros(4,1),zeros(4,1));

    verifyEqual(testCase,sys(32),80,'AbsTol',1e-8);
end
end

function u = controllerInput(kmh,x)
u = zeros(54,1);
u(1:4) = [35;-45;55;-65];
u(17:20) = [5000;4100;5000;4100];
u(40) = x;
u(42) = kmh;
u(49) = kmh;
u(51:54) = [2;3;4;5];
end
