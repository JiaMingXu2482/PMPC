function tests = test_zeng_diagnostic_channels
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testZengPublishesItsAdaptiveIndicators(testCase)
backup = local_backup({'PMPC_MODE','PMPC_ZENGRHO','PMPC_VERBOSE'});
cleanup = onCleanup(@() local_restore(backup)); %#ok<NASGU>
assignin('base', 'PMPC_MODE', 2);
assignin('base', 'PMPC_ZENGRHO', 1);
assignin('base', 'PMPC_VERBOSE', 0);
P = setup_pmpc();

vx = 20;
delta = 0.02;
beta = 0.03;
yawRate = 0.25;
u = local_input(P.S0.VehiclePara, vx, delta, beta, yawRate);
S = P.S0;
S.InitialParams.InitialGapflag = 1;
[hPlant, xPlant, vPlant, aPlant] = local_plant_state(P);
[sys, ~] = pmpc_step(u, P.Pm, S, hPlant, xPlant, vPlant, aPlant);

[rhoZeng, ~, iBeta, iR] = func_ZengRho(vx, P.S0.VehiclePara.mu, ...
    delta, beta, yawRate, P.S0.VehiclePara.g);
verifyEqual(testCase, sys(14:16), [rhoZeng; iBeta; iR], 'AbsTol', 1e-10);
end

function testBaselineMpcKeepsActuatorPriorityIndicators(testCase)
backup = local_backup({'PMPC_MODE','PMPC_ZENGRHO','PMPC_VERBOSE'});
cleanup = onCleanup(@() local_restore(backup)); %#ok<NASGU>
assignin('base', 'PMPC_MODE', 2);
assignin('base', 'PMPC_ZENGRHO', 0);
assignin('base', 'PMPC_VERBOSE', 0);
P = setup_pmpc();

u = local_input(P.S0.VehiclePara, 20, 0.02, 0.03, 0.25);
S = P.S0;
S.InitialParams.InitialGapflag = 1;
[hPlant, xPlant, vPlant, aPlant] = local_plant_state(P);
[sys, ~] = pmpc_step(u, P.Pm, S, hPlant, xPlant, vPlant, aPlant);

verifyEqual(testCase, sys(14:16), ones(3,1), 'AbsTol', 1e-9);
end

function [hPlant, xPlant, vPlant, aPlant] = local_plant_state(P)
% pmpc_step now receives the state sampled from the external 1 kHz plant.
hPlant = zeros(8,4);
xPlant = P.Pm.NLCSNN.x_ref * ones(4,1);
vPlant = zeros(4,1);
aPlant = zeros(4,1);
end

function u = local_input(V, vx, delta, beta, yawRate)
u = zeros(54,1);
u(17:20) = [5000; 4100; 5000; 4100];
u(34) = rad2deg(yawRate);
u(37) = rad2deg(beta);
u(42) = vx*3.6;
u(46:47) = rad2deg(delta);
u(48) = rad2deg(delta)*V.isw;
u(49) = vx*3.6;
end

function backup = local_backup(names)
backup = cell(numel(names),3);
for k = 1:numel(names)
    backup{k,1} = names{k};
    backup{k,2} = evalin('base', sprintf('exist(''%s'',''var'')', names{k})) == 1;
    if backup{k,2}, backup{k,3} = evalin('base', names{k}); end
end
end

function local_restore(backup)
for k = 1:size(backup,1)
    name = backup{k,1};
    if backup{k,2}
        assignin('base', name, backup{k,3});
    else
        evalin('base', ['clear ' name]);
    end
end
end
