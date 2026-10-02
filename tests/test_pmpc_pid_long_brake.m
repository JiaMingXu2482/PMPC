function tests = test_pmpc_pid_long_brake
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
load_system(fullfile(root,'pmpc_mil.slx'));
end

function testOnlyLongitudinalBrakeRequestBlocksThrottle(testCase)
verifyGreaterThan(testCase,max(runPid(75,80,0)),0.05);
verifyEqual(testCase,max(runPid(75,80,100)),0,'AbsTol',1e-9);
end

function testPidGetsLongitudinalRequestInsteadOfYawBrakeSum(testCase)
model = 'pmpc_mil';
pid = get_param([model '/PID velocity control'],'PortHandles');
line = get_param(pid.Inport(3),'Line');
source = get_param(line,'SrcBlockHandle');
verifyEqual(testCase,get_param(source,'Name'),'PMPC_LongBrakeRequest');
verifyEqual(testCase,get_param(source,'BlockType'),'Selector');
verifyEqual(testCase,str2double(get_param(source,'Indices')),9);
selector = get_param(source,'PortHandles');
inputLine = get_param(selector.Inport(1),'Line');
verifyEqual(testCase,get_param(get_param(inputLine,'SrcBlockHandle'),'Name'), ...
    'PMPC_MF');
end

function throttle = runPid(speedKmh,targetKmh,longBrakeN)
model = 'pmpc_pid_long_brake_test_model';
if bdIsLoaded(model), close_system(model,0); end
new_system(model);
cleanup = onCleanup(@() close_system(model,0)); %#ok<NASGU>
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.01','StopTime','0.5');
add_block('pmpc_mil/PID velocity control',[model '/PID']);
values = [speedKmh targetKmh longBrakeN];
for k = 1:3
    name = sprintf('Input%d',k);
    add_block('simulink/Sources/Constant',[model '/' name], ...
        'Value',num2str(values(k)));
    add_line(model,sprintf('%s/1',name),sprintf('PID/%d',k));
end
add_block('simulink/Sinks/To Workspace',[model '/ThrottleLog'], ...
    'VariableName','throttle_trace','SaveFormat','Structure With Time');
add_line(model,'PID/1','ThrottleLog/1');
out = sim(model);
trace = out.get('throttle_trace');
throttle = trace.signals.values(:);
end
