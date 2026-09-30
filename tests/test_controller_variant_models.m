function tests = test_controller_variant_models
tests = functiontests(localfunctions);
end

function testFixedModelsExposeExpectedControllerAndPlant(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
cases = {'mpc_mil','mpc_block','MPC_P',false; ...
         'zeng_mil','zeng_block','ZENG_P',true; ...
         'pmpc_mil','pmpc_block','PMPC_P',false};

for k = 1:size(cases,1)
    mdl = cases{k,1};
    verifyEqual(testCase, exist([mdl '.slx'],'file'), 4, ...
        sprintf('%s.slx must exist.', mdl));
    load_system(mdl);
    cleanup = onCleanup(@() close_system(mdl,0)); %#ok<NASGU>

    verifyEqual(testCase, numel(find_system(mdl,'SearchDepth',1, ...
        'Name','NLCSNN_Forward_1kHz')), 1);
    verifyEqual(testCase, numel(find_system(mdl,'SearchDepth',1, ...
        'Name','SemiActive_Passivity_1kHz')), 1);

    rootSf = sfroot;
    chart = rootSf.find('-isa','Stateflow.EMChart','Path',[mdl '/PMPC_MF']);
    verifyEqual(testCase, numel(chart), 1);
    if numel(chart) == 1
        verifyTrue(testCase, contains(chart.Script, ['function [sys, i_cmd] = ' cases{k,2}]));
        data = chart.find('-isa','Stateflow.Data','Name',cases{k,3});
        verifyEqual(testCase, numel(data), 1);
    end

    rhoPort = find_system(mdl,'SearchDepth',1,'Name','rho_ZENG');
    verifyEqual(testCase, ~isempty(rhoPort), cases{k,4});
    if cases{k,4}
        rhoChart = rootSf.find('-isa','Stateflow.EMChart', ...
            'Path',[mdl '/rho_ZENG_select']);
        verifyEqual(testCase, numel(rhoChart), 1);
        verifyTrue(testCase, contains(rhoChart.Script, 'rho_ZENG = sys(14);'));
    end
    clear cleanup
end
end

function testCarSimBoundaryDeclares54Channels(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
models = {'mpc_mil','zeng_mil','pmpc_mil'};

for k = 1:numel(models)
    mdl = models{k};
    load_system(mdl);
    cleanup = onCleanup(@() close_system(mdl,0)); %#ok<NASGU>
    block = [mdl '/CarSim_Output_54'];
    verifyNotEqual(testCase, getSimulinkBlockHandle(block), -1, ...
        sprintf('%s must declare the dynamic CarSim output as 54 channels.', mdl));
    if getSimulinkBlockHandle(block) ~= -1
        verifyEqual(testCase, get_param(block,'BlockType'), 'SignalSpecification');
        verifyEqual(testCase, get_param(block,'Dimensions'), '54');
        carSimPorts = get_param([mdl '/CarSim'],'PortHandles');
        line = get_param(carSimPorts.Outport(1),'Line');
        destinations = get_param(line,'DstPortHandle');
        verifyEqual(testCase, numel(destinations), 1);
        verifyEqual(testCase, getfullname(get_param(destinations(1),'Parent')), block);
    end
    clear cleanup
end
end

function testControllerChartHasFixedPortContracts(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
models = {'mpc_mil','zeng_mil','pmpc_mil'};
names = {'u','h_plant','x_plant','v_plant','a_plant','sys','i_cmd'};
sizes = {'54','[8 4]','[4 1]','[4 1]','[4 1]','54','4'};

for k = 1:numel(models)
    mdl = models{k};
    load_system(mdl);
    cleanup = onCleanup(@() close_system(mdl,0)); %#ok<NASGU>
    rootSf = sfroot;
    chart = rootSf.find('-isa','Stateflow.EMChart','Path',[mdl '/PMPC_MF']);
    verifyEqual(testCase, numel(chart), 1);
    for j = 1:numel(names)
        data = chart.find('-isa','Stateflow.Data','Name',names{j});
        verifyEqual(testCase, numel(data), 1);
        if numel(data) == 1
            verifyEqual(testCase, strrep(data.Props.Array.Size,' ',''), ...
                strrep(sizes{j},' ',''));
        end
    end
    clear cleanup
end
end
