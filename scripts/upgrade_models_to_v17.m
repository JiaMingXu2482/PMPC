function upgrade_models_to_v17()
%UPGRADE_MODELS_TO_V17 Update only the v14 state interface in four MIL SLX.
% The existing controller charts, CarSim wiring, and user tuning are kept.
startup_pmpc();
models = {'mpc_mil','zeng_mil','pmpc_mil','pmpc_nodelay_mil'};
sf = sfroot;
for k = 1:numel(models)
    model = models{k};
    load_system(model);
    plant = [model '/NLCSNN_Forward_1kHz'];
    chart = sf.find('-isa','Stateflow.EMChart','Path',[plant '/plant_fn']);
    assert(numel(chart)==1,'Missing plant chart: %s',model);
    old = 'h = reshape(h,227,4);';
    assert(contains(chart.Script,old),'Unexpected plant script: %s',model);
    chart.Script = strrep(chart.Script,old,'h = reshape(h,217,4);');
    for name = {'h','h_new'}
        data = chart.find('-isa','Stateflow.Data','Name',name{1});
        assert(numel(data)==1,'Missing %s in %s plant',name{1},model);
        data.Props.Array.Size = '[217 4]';
        data.Props.Array.IsDynamic = false;
        data.DataType = 'double';
    end
    set_param([plant '/UD_h'],'InitialCondition','zeros(217,4)');
    set_param([plant '/SIG_h'],'Dimensions','[217 4]');
    ctrl = sf.find('-isa','Stateflow.EMChart','Path',[model '/PMPC_MF']);
    assert(numel(ctrl)==1,'Missing controller chart: %s',model);
    data = ctrl.find('-isa','Stateflow.Data','Name','h_plant');
    assert(numel(data)==1,'Missing controller h_plant: %s',model);
    data.Props.Array.Size = '[217 4]';
    data.Props.Array.IsDynamic = false;
    data.DataType = 'double';
    save_system(model);
    close_system(model,0);
    fprintf('v17 state contract saved: %s (217x4 double)\n',model);
end
end
