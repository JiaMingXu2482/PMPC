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
assignin('base','PMPC_VERBOSE',0);
assignin('base','PMPC_ABL',0);
assignin('base','PMPC_CONTROLLER_VARIANT',3);
reference = setup_pmpc();
assignin('base','PMPC_CONTROLLER_VARIANT',4);
candidate = setup_pmpc();
cleanup = onCleanup(@() evalin('base', ...
    'clear PMPC_NLCSNN PMPC_VERBOSE PMPC_ABL PMPC_CONTROLLER_VARIANT')); %#ok<NASGU>

verifyEqual(testCase,candidate.Pm.MPCParameters.ControllerVariant,4);
verifyEqual(testCase,candidate.Pm.MPCParameters.Nx,6);
verifyEqual(testCase,candidate.Pm.MPCParameters.Ny,6);
verifyEqual(testCase,candidate.Pm.MPCParameters.Nu,3);
verifyEqual(testCase,reference.Pm.MPCParameters.Nx,7);
verifyEqual(testCase,reference.Pm.MPCParameters.Nu,3);
verifyEqual(testCase,candidate.Pm.MPCParameters.tau_d,0);
verifyEqual(testCase,candidate.S0.Constraints.ControllerMode,1);
verifyEqual(testCase,candidate.S0.Constraints.PrioModeRT,1);
verifyFalse(testCase,isfield(candidate.S0,'LongCoord'));
verifyEqual(testCase,candidate.Pm.NLCSNN.enabled,true);
verifyEqual(testCase,candidate.Pm.CostWeights,reference.Pm.CostWeights);
verifyEqual(testCase,candidate.Pm.MPCParameters.Np,reference.Pm.MPCParameters.Np);
verifyEqual(testCase,candidate.Pm.MPCParameters.Nc,reference.Pm.MPCParameters.Nc);
end

function testNoDelayMatchesSharedTuning(testCase)
noDelay = Atuning_pmpc_nodelay();
reference = Atuning_pmpc();
verifyEqual(testCase,noDelay.controller,'PMPC-noDelay');
noDelay = rmfield(noDelay,'controller');
reference = rmfield(reference,'controller');
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
verifyEqual(testCase,reference.Pm.CostWeights,noDelay.Pm.CostWeights);
end

function testNoDelayIsLabeledAsFourthPeerController(testCase)
assignin('base','PMPC_CONTROLLER_VARIANT',4);
cleanup = onCleanup(@() evalin('base','clear PMPC_CONTROLLER_VARIANT')); %#ok<NASGU>
output = evalc('setup_pmpc();');
verifySubstring(testCase,output,'④ PMPC-noDelay');
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
