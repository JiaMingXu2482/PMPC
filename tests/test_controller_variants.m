function tests = test_controller_variants
tests = functiontests(localfunctions);
end

function testVariantStateDimensions(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();

variants = [1 2 3];
expectedNx = [6 7 7];
expectedMode = [2 2 1];
expectedTau = [0 0.015 0.015];
for k = 1:numel(variants)
    assignin('base','PMPC_CONTROLLER_VARIANT',variants(k));
    assignin('base','PMPC_NLCSNN',1);
    assignin('base','PMPC_VERBOSE',0);
    P = setup_pmpc();
    verifyEqual(testCase, P.Pm.MPCParameters.ControllerVariant, variants(k));
    verifyEqual(testCase, P.Pm.MPCParameters.Nx, expectedNx(k));
    verifyEqual(testCase, P.Pm.MPCParameters.Ny, expectedNx(k));
    verifyEqual(testCase, P.Pm.MPCParameters.tau_d, expectedTau(k), 'AbsTol', 1e-12);
    verifyEqual(testCase, P.S0.Constraints.ControllerMode, expectedMode(k));
    verifyEqual(testCase, P.Pm.NLCSNN.enabled, true);
end
clearVariantWorkspace();
end

function testZengRhoIsSeparateScalar(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
assignin('base','PMPC_CONTROLLER_VARIANT',2);
assignin('base','PMPC_NLCSNN',1);
assignin('base','PMPC_VERBOSE',0);
P = setup_pmpc();
u = zeros(54,1);
u(1:4) = [35;-45;55;-65];
u(17:20) = [5000;4100;5000;4100];
u(42) = 72;
u(49) = 72;
u(51:54) = [2;3;4;5];
S = P.S0;
[hp,xp,vp,ap] = plantSample(P,u);
[sys,Sout,iCmd] = pmpc_step(u,P.Pm,S,hp,xp,vp,ap);
verifySize(testCase, sys, [54 1]);
verifySize(testCase, iCmd, [4 1]);
verifyTrue(testCase, isfinite(sys(14)));
verifyGreaterThanOrEqual(testCase, sys(14), 0);
verifyTrue(testCase, all(isfinite(Sout.InitialParams.prevstate.nlcsnn.i_prev)));
end

function testAllVariantsCompleteAControllerStep(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
u = controllerInput();

for variant = [1 2 3]
    assignin('base','PMPC_CONTROLLER_VARIANT',variant);
    assignin('base','PMPC_NLCSNN',1);
    assignin('base','PMPC_VERBOSE',0);
    P = setup_pmpc();
    [hp,xp,vp,ap] = plantSample(P,u);
    [~,S,~] = pmpc_step(u,P.Pm,P.S0,hp,xp,vp,ap);
    [sys,S,iCmd] = pmpc_step(u,P.Pm,S,hp,xp,vp,ap);

    verifySize(testCase, sys, [54 1]);
    verifySize(testCase, iCmd, [4 1]);
    verifyTrue(testCase, all(isfinite(sys)));
    verifyTrue(testCase, all(isfinite(S.WarmStart)));
end
clearVariantWorkspace();
end

function u = controllerInput()
u = zeros(54,1);
u(1:4) = [35;-45;55;-65];
u(17:20) = [5000;4100;5000;4100];
u(42) = 72;
u(49) = 72;
u(51:54) = [2;3;4;5];
end

function [hp,xp,vp,ap] = plantSample(P,u)
cmpD = u(51:54);
cmpRD = u(1:4);
hp = zeros(8,4); afp = zeros(4,1); vpp = zeros(4,1); initp = 0;
for k = 1:10
    [~,hp,afp,vpp,initp,xp,vp,ap] = func_NLCSNNPlant4( ...
        P.Pm.NLCSNN.net,cmpD,cmpRD,zeros(4,1),hp,afp,vpp,initp,P.Pm.NLCSNN);
end
end

function clearVariantWorkspace
evalin('base','clear PMPC_CONTROLLER_VARIANT PMPC_NLCSNN PMPC_VERBOSE');
end
