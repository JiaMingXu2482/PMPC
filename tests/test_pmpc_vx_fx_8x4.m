function tests = test_pmpc_vx_fx_8x4
tests = functiontests(localfunctions);
end

function testPmpc8x4CompletesOneControlTick(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root); startup_pmpc();
assignin('base','PMPC_CONTROLLER_VARIANT',3);
assignin('base','PMPC_NLCSNN',0);
assignin('base','PMPC_VERBOSE',0);
c = onCleanup(@() evalin('base', ...
    'clear PMPC_CONTROLLER_VARIANT PMPC_NLCSNN PMPC_VERBOSE'));
P = setup_pmpc();
S = P.S0; S.InitialParams.InitialGapflag = 1;
u = zeros(54,1);
u(1:4) = [35;-45;55;-65];
u(17:20) = [5000;4100;5000;4100];
u(42) = 72; u(49) = 72; u(51:54) = [2;3;4;5];
[sys,S] = pmpc_step(u,P.Pm,S,zeros(8,4),zeros(4,1), ...
    zeros(4,1),zeros(4,1));
verifySize(testCase,sys,[54 1]);
verifyTrue(testCase,all(isfinite(sys)));
verifySize(testCase,S.InitialParams.U,[4 1]);
verifyEqual(testCase,S.LongCoord.qp_exitflag,1);
clear c
end

function testExtendedPredictorBrakesLongitudinalSpeed(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root,'controller'));
P = struct('Nx',8,'Ny',8,'Nu',4,'Np',2,'Nc',2,'Ts',0.1, ...
    'Ts_exec',0.01,'first_Tc',0,'tau_d',0.015,'LTV_on',0);
V = struct('m',1860,'ms',1500,'g',9.81,'lf',1.25,'lr',1.5, ...
    'Ix',900,'Iz',2500,'h_S2R',0.57,'Kt',2e4, ...
    'CbarF',-1e5,'CbarR',-1e5);
M = struct('x_dot',20,'Yawrate',0,'delta_f',0);
L = struct('x0',[zeros(7,1);20], 'u0',zeros(4,1), ...
    'dU',zeros(4,2),'kap',zeros(2,1));
Model = func_DynamicalModel8x4(V,P,M,4,L,20*ones(2,1));
verifySize(testCase,Model.A_aug,[12 12 2]);
verifySize(testCase,Model.B_aug,[12 4 2]);
[~,Theta,Gamma,Phi] = func_SystemFurture(P,Model,zeros(2,1),[0;0;0]);
verifyLessThan(testCase,Theta(8,4),0);
verifyEqual(testCase,numel(Gamma),10);
verifySize(testCase,Phi,[16 10]);
end

function testPmpcUsesSpeedStateAndBrakeInput(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root,'controller'), fullfile(root,'config'));
assignin('base','PMPC_CONTROLLER_VARIANT',3);
c = onCleanup(@() evalin('base','clear PMPC_CONTROLLER_VARIANT'));
P = setup_pmpc();
verifyEqual(testCase, P.Pm.MPCParameters.Nx, 8);
verifyEqual(testCase, P.Pm.MPCParameters.Nu, 4);
verifySize(testCase, P.S0.InitialParams.U, [4 1]);
verifySize(testCase, P.S0.WarmStart, [31 1]);
verifyEqual(testCase,P.Pm.MPCParameters.ConNodes_sh,[1;2;5]);
verifyEqual(testCase,P.Pm.MPCParameters.ConNodes_r,[1;2;5]);
verifyEqual(testCase,P.Pm.MPCParameters.ConNodes_env,[1;2;5;10]);
clear c
end
