function tests = test_nlcsnn_v14_models
tests = functiontests(localfunctions);
end

function testAllFourModelsUseRawV14Plant(testCase)
project = fileparts(fileparts(mfilename('fullpath')));
addpath(project);
startup_pmpc();
models = {'mpc_mil','zeng_mil','pmpc_mil','pmpc_nodelay_mil'};
for k=1:numel(models)
    mdl=models{k};
    load_system(fullfile(project,[mdl '.slx']));
    plant=[mdl '/NLCSNN_Forward_1kHz'];
    verifyEqual(testCase,get_param([plant '/UD_h'],'InitialCondition'), ...
        'zeros(227,4,''single'')');
    verifyEqual(testCase,get_param([plant '/SIG_h'],'Dimensions'), ...
        '[227 4]');
    verifyEqual(testCase,getSimulinkBlockHandle([mdl ...
        '/SemiActive_Passivity_1kHz']),-1);
    root=sfroot;
    chart=root.find('-isa','Stateflow.EMChart','Path',[plant '/plant_fn']);
    verifyEqual(testCase,numel(chart),1);
    verifyTrue(testCase,contains(chart.Script,'reshape(h,227,4)'));
    controller=root.find('-isa','Stateflow.EMChart','Path',[mdl '/PMPC_MF']);
    state=controller.find('-isa','Stateflow.Data','Name','h_plant');
    verifyEqual(testCase,state.Props.Array.Size,'[227 4]');
    pp=get_param(plant,'PortHandles');
    dp=get_param([mdl '/Fd_safe_demux'],'PortHandles');
    line=get_param(dp.Inport(1),'Line');
    verifyEqual(testCase,get_param(line,'SrcPortHandle'),pp.Outport(1));
    close_system(mdl,0);
end
end
