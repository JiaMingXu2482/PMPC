function tests = test_shared_longitudinal_model
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testPrivateLongitudinalScopesAreRemovedButPortsRemain(testCase)
for names = {'pmpc_mil','pmpc_nodelay_mil'}
    model = names{1};
    load_system(model);
    cleaner = onCleanup(@() close_system(model,0)); %#ok<NASGU>
    verifyEqual(testCase,getSimulinkBlockHandle([model '/PMPC_LongCoord_Diagnostics']),-1);
    verifyEqual(testCase,getSimulinkBlockHandle([model '/PMPC_LongCoord_Log']),-1);
    ports = get_param([model '/PMPC_MF'],'PortHandles');
    verifyEqual(testCase,numel(ports.Outport),3);
    root = sfroot;
    chart = root.find('-isa','Stateflow.EMChart','Path',[model '/PMPC_MF']);
    verifyEqual(testCase,numel(chart),1);
    verifyFalse(testCase,contains(chart.Script,'St.LongCoord'));
    verifyTrue(testCase,contains(chart.Script,'long_diag = zeros(12,1);'));
    pid = get_param([model '/PID velocity control'],'PortHandles');
    verifyEqual(testCase,numel(pid.Inport),3);
    line = get_param(pid.Inport(3),'Line');
    source = get_param(line,'SrcBlockHandle');
    verifyEqual(testCase,get_param(source,'Name'),'DB_Request_MFx_sys18');
    clear cleaner
end
end
