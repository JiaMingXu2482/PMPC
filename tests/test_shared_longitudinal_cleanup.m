function tests = test_shared_longitudinal_cleanup
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function teardownOnce(~)
evalin('base','clear PMPC_CONTROLLER_VARIANT PMPC_LONGCOORD PMPC_LONGLIM PMPC_VERBOSE');
end

function testCurrentBundlesDoNotExposePrivateSpeedGovernors(testCase)
verifyFalse(testCase,isfield(Atuning_pmpc(),'LongCoordMode'));
verifyFalse(testCase,isfield(Atuning_pmpc(),'Long_a_max'));
verifyFalse(testCase,isfield(Atuning_pmpc_nodelay(),'LongCoordMode'));
verifyFalse(testCase,isfield(Atuning_zeng(),'ZengLong_on'));
verifyFalse(testCase,isfield(Atuning_zeng(),'Zg_tau'));
end

function testAllVariantsOmitPrivateLongitudinalState(testCase)
for variant = 1:4
    assignin('base','PMPC_CONTROLLER_VARIANT',variant);
    assignin('base','PMPC_VERBOSE',0);
    P = setup_pmpc();
    verifyFalse(testCase,isfield(P.S0,'LongCoord'));
    verifyFalse(testCase,isfield(P.S0.Constraints,'LongCoordMode'));
    verifyFalse(testCase,isfield(P.S0.Constraints,'ZengLong_on'));
    verifyFalse(testCase,isfield(P.S0.Constraints,'LongLim_diag'));
end
end

function testLegacyWorkspaceFlagsCannotReactivatePrivateSpeedControl(testCase)
assignin('base','PMPC_CONTROLLER_VARIANT',3);
assignin('base','PMPC_LONGCOORD',3);
assignin('base','PMPC_LONGLIM',1);
P = setup_pmpc();
verifyFalse(testCase,isfield(P.S0,'LongCoord'));
verifyFalse(testCase,isfield(P.S0.Constraints,'LongCoordMode'));
verifyFalse(testCase,isfield(P.S0.Constraints,'LongLim_diag'));
end

function testPmpcWrappersKeepTwelveZeroDiagnosticChannels(testCase)
for variant = [3 4]
    assignin('base','PMPC_CONTROLLER_VARIANT',variant);
    assignin('base','PMPC_VERBOSE',0);
    P = setup_pmpc();
    P.S0.InitialParams.InitialGapflag = 1;
    P.Pm.RoadMu.enabled = false;
    u = controllerInput(80,40);
    if variant == 3
        clear pmpc_block
        [sys,i_cmd,long_diag] = pmpc_block(u,zeros(8,4), ...
            zeros(4,1),zeros(4,1),zeros(4,1),P);
        [sys2,i_cmd2,long_diag2] = pmpc_block(u,zeros(8,4), ...
            zeros(4,1),zeros(4,1),zeros(4,1),P);
    else
        clear pmpc_nodelay_block
        [sys,i_cmd,long_diag] = pmpc_nodelay_block(u,zeros(8,4), ...
            zeros(4,1),zeros(4,1),zeros(4,1),P);
        [sys2,i_cmd2,long_diag2] = pmpc_nodelay_block(u,zeros(8,4), ...
            zeros(4,1),zeros(4,1),zeros(4,1),P);
    end
    verifySize(testCase,sys,[54 1]);
    verifySize(testCase,i_cmd,[4 1]);
    verifySize(testCase,long_diag,[12 1]);
    verifyEqual(testCase,long_diag,zeros(12,1));
    verifySize(testCase,sys2,[54 1]);
    verifySize(testCase,i_cmd2,[4 1]);
    verifyEqual(testCase,long_diag2,zeros(12,1));
end
end

function testSharedQpHelpersOmitFourthInputAndEighthState(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
files = {'controller/pmpc_step.m', ...
    'controller/func_BuildQPConstraints.m', ...
    'controller/func_BuildQPCost.m', ...
    'controller/func_PriorityCert.m', ...
    'controller/func_LTRDiagnosis.m', ...
    'controller/func_SystemFurture.m'};
for k = 1:numel(files)
    source = fileread(fullfile(root,files{k}));
    verifyFalse(testCase,contains(source,'Nu == 4'),files{k});
    verifyFalse(testCase,contains(source,'Ny == 8'),files{k});
    verifyFalse(testCase,contains(source,'Nx == 8'),files{k});
    verifyFalse(testCase,contains(source,'nd == 5'),files{k});
end
end

function testPredictionHelpersOmitPrivateSpeedPreview(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
files = {'controller/func_DynamicalModel.m', ...
    'controller/func_RefTraj_LocalPlanning.m'};
for k = 1:numel(files)
    source = fileread(fullfile(root,files{k}));
    verifyFalse(testCase,contains(source,'Vx_pred'),files{k});
    verifyFalse(testCase,contains(source,'useScheduledSpeed'),files{k});
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
