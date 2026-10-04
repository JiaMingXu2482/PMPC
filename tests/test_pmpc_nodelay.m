function tests = test_pmpc_nodelay
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testNoDelayRetainsSharedPmpcWeightsWithLegacyDimensions(testCase)
assignin('base','PMPC_NLCSNN',1);
assignin('base','PMPC_LONGCOORD',1);
assignin('base','PMPC_VERBOSE',0);
assignin('base','PMPC_ABL',0);
assignin('base','PMPC_CONTROLLER_VARIANT',3);
reference = setup_pmpc();
assignin('base','PMPC_CONTROLLER_VARIANT',4);
candidate = setup_pmpc();
cleanup = onCleanup(@() evalin('base', ...
    'clear PMPC_NLCSNN PMPC_LONGCOORD PMPC_VERBOSE PMPC_ABL PMPC_CONTROLLER_VARIANT')); %#ok<NASGU>

verifyEqual(testCase,candidate.Pm.MPCParameters.ControllerVariant,4);
verifyEqual(testCase,candidate.Pm.MPCParameters.Nx,6);
verifyEqual(testCase,candidate.Pm.MPCParameters.Ny,6);
verifyEqual(testCase,candidate.Pm.MPCParameters.Nu,3);
verifyEqual(testCase,reference.Pm.MPCParameters.Nx,8);
verifyEqual(testCase,reference.Pm.MPCParameters.Nu,4);
verifyEqual(testCase,candidate.Pm.MPCParameters.tau_d,0);
verifyEqual(testCase,candidate.S0.Constraints.ControllerMode,1);
verifyEqual(testCase,candidate.S0.Constraints.PrioModeRT,1);
verifyEqual(testCase,candidate.S0.Constraints.LongCoordMode,1);
verifyEqual(testCase,candidate.Pm.NLCSNN.enabled,true);
longitudinalWeights = {'Q8','R4','S4'};
verifyEqual(testCase,rmfield(candidate.Pm.CostWeights,longitudinalWeights), ...
    rmfield(reference.Pm.CostWeights,longitudinalWeights));
verifyEqual(testCase,candidate.Pm.CostWeights.Q8,0);
verifyEqual(testCase,candidate.Pm.CostWeights.R4,0);
verifyEqual(testCase,candidate.Pm.MPCParameters.Np,reference.Pm.MPCParameters.Np);
verifyEqual(testCase,candidate.Pm.MPCParameters.Nc,reference.Pm.MPCParameters.Nc);
end

function testNoDelayMatchesSharedTuningWithout8x4Weights(testCase)
noDelay = Atuning_pmpc_nodelay();
reference = Atuning_pmpc();
verifyEqual(testCase,noDelay.controller,'PMPC-noDelay');
noDelay = rmfield(noDelay,'controller');
reference = rmfield(reference,'controller');
reference = rmfield(reference,{'Q8','R4','S4'});
verifyEqual(testCase,noDelay,reference);
end

function testOnlyNoDelayVariantLoadsItsOwnTuningBundle(testCase)
cleanup = onCleanup(@() evalin('base', ...
    'clear PMPC_CONTROLLER_VARIANT PMPC_VERBOSE')); %#ok<NASGU>
assignin('base','PMPC_VERBOSE',0);
profile clear;
profile on;
assignin('base','PMPC_CONTROLLER_VARIANT',4);
noDelay = setup_pmpc();
profile off;
info = profile('info');
called = {info.FunctionTable.FunctionName};
verifyTrue(testCase,any(contains(called,'Atuning_pmpc_nodelay')));
verifyEqual(testCase,noDelay.Pm.MPCParameters.ControllerVariant,4);

profile clear;
profile on;
assignin('base','PMPC_CONTROLLER_VARIANT',3);
reference = setup_pmpc();
profile off;
info = profile('info');
called = {info.FunctionTable.FunctionName};
verifyFalse(testCase,any(contains(called,'Atuning_pmpc_nodelay')));
longitudinalWeights = {'Q8','R4','S4'};
verifyEqual(testCase,rmfield(reference.Pm.CostWeights,longitudinalWeights), ...
    rmfield(noDelay.Pm.CostWeights,longitudinalWeights));
end

function testNoDelayIsLabeledAsFourthPeerController(testCase)
assignin('base','PMPC_CONTROLLER_VARIANT',4);
cleanup = onCleanup(@() evalin('base','clear PMPC_CONTROLLER_VARIANT')); %#ok<NASGU>
output = evalc('setup_pmpc();');
verifySubstring(testCase,output,'④ PMPC-noDelay');
end

function testNoDelayStillRunsPmpcLongitudinalCoordinator(testCase)
assignin('base','PMPC_NLCSNN',1);
assignin('base','PMPC_LONGCOORD',3);
assignin('base','PMPC_VERBOSE',0);
assignin('base','PMPC_ABL',0);
assignin('base','PMPC_CONTROLLER_VARIANT',4);
cleanup = onCleanup(@() evalin('base', ...
    'clear PMPC_NLCSNN PMPC_LONGCOORD PMPC_VERBOSE PMPC_ABL PMPC_CONTROLLER_VARIANT')); %#ok<NASGU>
P = setup_pmpc();
P.Pm.Reftraj = func_WayPoints(5,69,false);
S = P.S0;
S.InitialParams.InitialGapflag = 1;
S.LongCoord.safety_margin_prev = 123;
u = zeros(54,1);
u(1:4) = [35;-45;55;-65];
u(17:20) = [5000;4100;5000;4100];
u(40) = 50+1.2466;
u(42) = 80;
u(49) = 80;
u(51:54) = [2;3;4;5];
[sys,S] = pmpc_step(u,P.Pm,S,zeros(8,4),zeros(4,1), ...
    zeros(4,1),zeros(4,1));

verifySize(testCase,sys,[54 1]);
verifyGreaterThan(testCase,S.LongCoord.Fx_request,0);
verifyLessThan(testCase,min(S.LongCoord.Vx_pred),80/3.6);
verifyEqual(testCase,S.LongCoord.speed_actual,80/3.6,'AbsTol',1e-10);
verifyNotEqual(testCase,S.LongCoord.safety_margin_prev,123);
verifyEqual(testCase,S.LongCoord.safety_margin_prev,S.LongCoord.m_speed);
end

function testNoDelayHasOwnInitializerAndModel(testCase)
P = mil_init_PMPCnoDelay();
verifyEqual(testCase,P.Pm.MPCParameters.ControllerVariant,4);
verifyEqual(testCase,P.Pm.MPCParameters.Nx,6);
verifyEqual(testCase,evalin('base','PMPC_NODELAY_P.Pm.MPCParameters.Nx'),6);

mdl = 'pmpc_nodelay_mil';
verifyEqual(testCase,exist([mdl '.slx'],'file'),4);
load_system(mdl);
cleanup = onCleanup(@() close_system(mdl,0)); %#ok<NASGU>
verifyNotEqual(testCase,getSimulinkBlockHandle([mdl '/NLCSNN_Forward_1kHz']),-1);
verifyNotEqual(testCase,getSimulinkBlockHandle([mdl '/SemiActive_Passivity_1kHz']),-1);
rootSf = sfroot;
chart = rootSf.find('-isa','Stateflow.EMChart','Path',[mdl '/PMPC_MF']);
verifyEqual(testCase,numel(chart),1);
verifyTrue(testCase,contains(chart.Script,'pmpc_nodelay_block('));
param = chart.find('-isa','Stateflow.Data','Name','PMPC_NODELAY_P');
verifyEqual(testCase,numel(param),1);
end

function testNoDelayIsRecognizedByManualRunEntry(testCase)
verifyEqual(testCase,func_ControllerModelForTag('PMPC_NODELAY'), ...
    'pmpc_nodelay_mil');
missing = fullfile(tempdir,'no_such_nodelay_simfile.sim');
verifyError(testCase,@() run_current_carsim(missing,'pmpc_nodelay_mil'), ...
    'run_current_carsim:NoSimfile');
end

function testIsolatedJturnInputChangesOnlyFinalSpeedOverride(testCase)
input = sprintf('SV_VXS 80\nMU_ROAD_CONSTANT 0.85\nSV_VXS 90\n');
output = func_ReplaceLastCarSimParameter(input,'SV_VXS','80');
verifyEqual(testCase,output, ...
    sprintf('SV_VXS 80\nMU_ROAD_CONSTANT 0.85\nSV_VXS 80\n'));
end
