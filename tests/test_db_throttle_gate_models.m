function tests = test_db_throttle_gate_models
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testAllControllersUsePidThrottleWithoutDbOverride(testCase)
models = {'mpc_mil','zeng_mil','pmpc_mil','pmpc_nodelay_mil'};
for i = 1:numel(models)
    model = models{i};
    load_system(model);
    pid = [model '/PID velocity control'];
    outPorts = get_param([pid '/Out1'],'PortHandles');
    outLine = get_param(outPorts.Inport(1),'Line');
    verifyNotEqual(testCase,outLine,-1);
    sourceHandle = get_param(outLine,'SrcBlockHandle');
    assertNotEqual(testCase,sourceHandle,-1, ...
        'The throttle output line must have a source block.');
    verifyEqual(testCase,get_param(sourceHandle,'Name'), ...
        'Saturation','Throttle must come directly from the speed PID.');
    verifyEqual(testCase,getSimulinkBlockHandle([pid '/DB_ThrottleCut_50Nm']),-1);
    verifyEqual(testCase,getSimulinkBlockHandle([pid '/LongBrakeThrottleGate']),-1);
    close_system(model,0);
end
end

function testDbRequestSelectorUsesControllerSys18(testCase)
models = {'mpc_mil','zeng_mil','pmpc_mil','pmpc_nodelay_mil'};
for i = 1:numel(models)
    model = models{i};
    load_system(model);
    selector = [model '/DB_Request_MFx_sys18'];
    verifyEqual(testCase,str2double(get_param(selector,'Indices')),18);
    controllerPorts = get_param([model '/PMPC_MF'],'PortHandles');
    selectorPorts = get_param(selector,'PortHandles');
    inLine = get_param(selectorPorts.Inport(1),'Line');
    verifyNotEqual(testCase,inLine,-1);
    verifyEqual(testCase,get_param(inLine,'SrcPortHandle'),controllerPorts.Outport(1), ...
        'The selector must read sys, not the 12-element diagnostic output.');
    close_system(model,0);
end
end
