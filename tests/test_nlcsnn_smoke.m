function tests = test_nlcsnn_smoke
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testMpcZengAndPmpcAdvanceCommonPlant(testCase)
mode = [2 2 1];
zeng = [0 1 0];
for controller = 1:3
    [P, cleanup] = configuredController(mode(controller), zeng(controller)); %#ok<ASGLU>
    S = P.S0;
    S.InitialParams.InitialGapflag = 1;
    for step = 1:3
        u = deterministicInput(step);
        [sys, S] = pmpc_step(u, P.Pm, S);
        verifyTrue(testCase, all(isfinite(sys(5:8))));
        verifyTrue(testCase, all(isfinite(S.InitialParams.prevstate.nlcsnn.h(:))));
        verifyGreaterThanOrEqual(testCase, ...
            S.InitialParams.prevstate.nlcsnn.i_prev, zeros(4,1));
        verifyLessThanOrEqual(testCase, ...
            S.InitialParams.prevstate.nlcsnn.i_prev, ...
            P.Pm.NLCSNN.i_max*ones(4,1));
    end

    h = S.InitialParams.prevstate.nlcsnn.h;
    verifySize(testCase, h, [8 4]);
    verifyGreaterThan(testCase, max(max(abs(h-h(:,1)))), 0);
    verifyEqual(testCase, P.Pm.MPCParameters.Nx, 7);
    verifyEqual(testCase, P.Pm.MPCParameters.Ne, 8);
    verifyEqual(testCase, P.Pm.MPCParameters.Nr, 3);
    verifySize(testCase, S.cert, [36 1]);
    clear cleanup
end
end

function u = deterministicInput(step)
u = zeros(54,1);
u(1:4) = [25+5*step; -35-3*step; 45+4*step; -55-2*step];
u(17:20) = [5000;4100;5000;4100];
u(34) = rad2deg(0.02*step);
u(37) = rad2deg(0.003*step);
u(42) = 72;
u(46:47) = rad2deg(0.002*step);
u(48) = rad2deg(0.002*step)*18.57;
u(49) = 72;
u(51:54) = [2+step;4+step;6+step;8+step];
end

function [P, cleanup] = configuredController(mode, zeng)
names = {'PMPC_MODE','PMPC_ZENGRHO','PMPC_VERBOSE','PMPC_NLCSNN'};
backup = cell(numel(names),3);
for k = 1:numel(names)
    backup{k,1} = names{k};
    backup{k,2} = evalin('base', sprintf('exist(''%s'',''var'')', names{k})) == 1;
    if backup{k,2}, backup{k,3} = evalin('base', names{k}); end
end
cleanup = onCleanup(@() restoreBase(backup));
assignin('base','PMPC_MODE',mode);
assignin('base','PMPC_ZENGRHO',zeng);
assignin('base','PMPC_VERBOSE',0);
assignin('base','PMPC_NLCSNN',1);
P = setup_pmpc();
end

function restoreBase(backup)
for k = 1:size(backup,1)
    if backup{k,2}
        assignin('base', backup{k,1}, backup{k,3});
    else
        evalin('base', ['clear ' backup{k,1}]);
    end
end
end
