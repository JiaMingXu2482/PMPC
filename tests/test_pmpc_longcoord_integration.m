function tests = test_pmpc_longcoord_integration
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testWaypointGenerationCanSkipSave(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
path = fullfile(root,'data','WayPoints_Type1.mat');
before = readBytes(path);
W = func_WayPoints(1,69,false);
verifyGreaterThan(testCase,size(W,1),100);
verifyEqual(testCase,readBytes(path),before);
end

function testOffIsBaseline(testCase)
P = configured(3,0);
S = activeState(P);
[sys,S] = step(P,S,controllerInput(80,30));
verifySize(testCase,sys,[54 1]);
verifyEqual(testCase,sys(9),0.0418676779019,'AbsTol',1e-6);
verifyEqual(testCase,sys(32),80,'AbsTol',1e-10);
verifyEqual(testCase,S.LongCoord.Fx_request,0,'AbsTol',1e-12);
end

function testMpcZengUnchanged(testCase)
for variant = [1 2]
    P = configured(variant,1);
    [sys,S] = step(P,activeState(P),controllerInput(80,30));
    verifySize(testCase,sys,[54 1]);
    verifyEqual(testCase,sys(32),80,'AbsTol',1e-10);
    verifyEqual(testCase,S.LongCoord.Fx_request,0,'AbsTol',1e-12);
    verifyEqual(testCase,S.LongCoord.diag,zeros(8,1),'AbsTol',1e-12);
end
end

function testPmpcSchedulesBeforeQp(testCase)
P = configured(3,1);
P.Pm.Reftraj = func_WayPoints(5,69,false);
S = activeState(P);
[sys,S] = step(P,S,controllerInput(80,50));
verifySize(testCase,sys,[54 1]);
verifyGreaterThan(testCase,S.LongCoord.Fx_request,0);
verifyLessThan(testCase,min(S.LongCoord.Vx_pred),80/3.6);
verifyLessThanOrEqual(testCase,sys(32),80);
verifyEqual(testCase,S.LongCoord.speed_actual,80/3.6,'AbsTol',1e-10);
end

function testAllocatorFailureIsVisible(testCase)
P = configured(3,1);
P.Pm.Reftraj = func_WayPoints(5,69,false);
S = activeState(P);
[~,S] = step(P,S,controllerInput(80,50));
verifyTrue(testCase,isfinite(S.LongCoord.allocation_exitflag));
verifyEqual(testCase,S.LongCoord.prev.allocationFailed, ...
    S.LongCoord.allocation_exitflag<=0);
verifyGreaterThanOrEqual(testCase,S.LongCoord.Fx_achieved,0);
end

function testNoThrottleBrakeFight(testCase)
P = configured(3,1);
P.Pm.Reftraj = func_WayPoints(5,69,false);
[sys,S] = step(P,activeState(P),controllerInput(80,50));
brakeActual = sum(max(sys(1:4),0));
verifyEqual(testCase,S.LongCoord.brake_active,brakeActual>1);
verifyEqual(testCase,S.LongCoord.Vset_pid,sys(32),'AbsTol',1e-12);
load_system('pmpc_mil');
pidPorts = get_param('pmpc_mil/PID velocity control','PortHandles');
gateLine = get_param(pidPorts.Inport(3),'Line');
source = get_param(gateLine,'SrcPortHandle');
verifyEqual(testCase,get_param(source,'Parent'), ...
    'pmpc_mil/PMPC_BrakeTorqueSum');
verifyEqual(testCase,get_param( ...
    'pmpc_mil/PID velocity control/DB_on','const'),'1');
end

function testOutput54AndStateDimensions(testCase)
P = configured(3,1);
[sys,S] = step(P,activeState(P),controllerInput(80,50));
verifySize(testCase,sys,[54 1]);
verifySize(testCase,S.cert,[36 1]);
verifySize(testCase,S.LongCoord.diag,[8 1]);
verifySize(testCase,S.LongCoord.Vx_pred,[P.Pm.MPCParameters.Np 1]);
load_system('pmpc_mil');
controllerPorts = get_param('pmpc_mil/PMPC_MF','PortHandles');
verifyEqual(testCase,numel(controllerPorts.Outport),3);
verifyNotEqual(testCase,getSimulinkBlockHandle( ...
    'pmpc_mil/PMPC_LongCoord_Diagnostics'),-1);
verifyNotEqual(testCase,getSimulinkBlockHandle( ...
    'pmpc_mil/PMPC_LongCoord_Log'),-1);
end

function P = configured(variant,mode)
assignin('base','PMPC_CONTROLLER_VARIANT',variant);
assignin('base','PMPC_LONGCOORD',mode);
assignin('base','PMPC_NLCSNN',1);
assignin('base','PMPC_VERBOSE',0);
P = setup_pmpc();
end

function S = activeState(P)
S = P.S0;
S.InitialParams.InitialGapflag = 1;
end

function [sys,S] = step(P,S,u)
[sys,S,~] = pmpc_step(u,P.Pm,S,zeros(8,4),zeros(4,1), ...
    zeros(4,1),zeros(4,1));
end

function u = controllerInput(kmh,x)
u = zeros(54,1);
u(1:4) = [35;-45;55;-65];
u(17:20) = [5000;4100;5000;4100];
u(40) = x+1.2466;
u(42) = kmh;
u(49) = kmh;
u(51:54) = [2;3;4;5];
end

function bytes = readBytes(path)
fid = fopen(path,'rb');
assert(fid > 0);
cleanup = onCleanup(@() fclose(fid));
bytes = fread(fid,inf,'uint8=>uint8');
end
