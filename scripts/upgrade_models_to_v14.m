function upgrade_models_to_v14()
%UPGRADE_MODELS_TO_V14 Preserve controller tuning and update damper wiring.
% Uses the existing four SLX files in place; no variant is cloned over another.
startup_pmpc();
setup_pmpc();
models = {'mpc_mil','zeng_mil','pmpc_mil','pmpc_nodelay_mil'};
for k = 1:numel(models)
    mdl = models{k};
    load_system(mdl);
    plant = [mdl '/NLCSNN_Forward_1kHz'];
    assert(getSimulinkBlockHandle(plant) ~= -1, ...
        'upgrade_models_to_v14:MissingPlant', '%s has no 1 kHz plant.', mdl);
    set_param([plant '/UD_h'], 'InitialCondition', 'zeros(227,4,''single'')');
    set_param([plant '/SIG_h'], 'Dimensions', '[227 4]');
    root = sfroot;
    chart = root.find('-isa','Stateflow.EMChart','Path',[plant '/plant_fn']);
    assert(numel(chart)==1, 'Missing plant MATLAB Function in %s.',mdl);
    chart.Script = localPlantScript();
    localSizePlantPorts(chart);
    rawPort = [plant '/Fd_raw'];
    if getSimulinkBlockHandle(rawPort) == -1
        add_block('simulink/Sinks/Out1',rawPort,'Port','6', ...
            'Position',[710 355 740 369]);
        add_line(plant,'plant_fn/9','Fd_raw/1','autorouting','on');
    end
    localSizeControllerState([mdl '/PMPC_MF']);
    if strcmp(mdl,'pmpc_mil') || strcmp(mdl,'pmpc_nodelay_mil')
        localSyncPriorityDiagnostics(mdl);
    end
    passivity = [mdl '/SemiActive_Passivity_1kHz'];
    demux = [mdl '/Fd_safe_demux'];
    if getSimulinkBlockHandle(passivity) ~= -1
        pp = get_param(passivity,'PortHandles');
        for j = 1:numel(pp.Inport)
            line = get_param(pp.Inport(j),'Line');
            if line ~= -1, delete_line(line); end
        end
        for j = 1:numel(pp.Outport)
            line = get_param(pp.Outport(j),'Line');
            if line ~= -1, delete_line(line); end
        end
        delete_block(passivity);
    end
    dp = get_param(demux,'PortHandles');
    line = get_param(dp.Inport(1),'Line');
    if line ~= -1, delete_line(line); end
    sp = get_param(plant,'PortHandles');
    add_line(mdl,sp.Outport(1),dp.Inport(1),'autorouting','on');
    save_system(mdl);
    close_system(mdl,0);
    fprintf('%s: v14 plant and inverse interface updated.\n',mdl);
end

function localSyncPriorityDiagnostics(mdl)
% The controller source is embedded in the MATLAB Function chart. Refresh
% its logging output as well as the plant when upgrading an existing SLX.
if strcmp(mdl,'pmpc_mil')
    entry = 'pmpc_block.m';
else
    entry = 'pmpc_nodelay_block.m';
end
source = which(entry);
assert(~isempty(source),'Missing controller source %s.',entry);
root = sfroot;
chart = root.find('-isa','Stateflow.EMChart','Path',[mdl '/PMPC_MF']);
assert(numel(chart)==1,'Missing controller MATLAB Function in %s.',mdl);
chart.Script = fileread(source);
diagPort = chart.find('-isa','Stateflow.Data','Name','long_diag');
assert(numel(diagPort)==1,'Missing priority diagnostics output in %s.',mdl);
diagPort.Props.Array.Size = '[12 1]';
diagPort.Props.Array.IsDynamic = false;
end

function localSizePlantPorts(chart)
ports = { ...
    'u54','54'; 'i_cmd','[4 1]'; 'h','[227 4]'; ...
    'a_filt','[4 1]'; 'v_prev','[4 1]'; 'init_done','1'; ...
    'F_plant','[4 1]'; 'h_new','[227 4]'; ...
    'a_filt_new','[4 1]'; 'v_prev_new','[4 1]'; 'init_new','1'; ...
    'x_plant','[4 1]'; 'v_plant','[4 1]'; 'a_plant','[4 1]'; 'F_raw','[4 1]'};
for q=1:size(ports,1)
    data=chart.find('-isa','Stateflow.Data','Name',ports{q,1});
    assert(numel(data)==1,'Missing plant port %s.',ports{q,1});
    data.Props.Array.Size=ports{q,2};
    data.Props.Array.IsDynamic=false;
    if strcmp(ports{q,1},'h') || strcmp(ports{q,1},'h_new')
        data.DataType='single';
    end
end
end
end

function localSizeControllerState(block)
root = sfroot;
chart = root.find('-isa','Stateflow.EMChart','Path',block);
assert(numel(chart)==1,'Missing controller MATLAB Function: %s',block);
state = chart.find('-isa','Stateflow.Data','Name','h_plant');
assert(numel(state)==1,'Missing h_plant port: %s',block);
state.Props.Array.Size = '[227 4]';
state.Props.Array.IsDynamic = false;
end

function script = localPlantScript()
script = sprintf([ ...
    'function [F_plant,h_new,a_filt_new,v_prev_new,init_new,x_plant,v_plant,a_plant,F_raw] = ...\n' ...
    '    plant_fn(u54,i_cmd,h,a_filt,v_prev,init_done,NLCSNN)\n' ...
    '%%#codegen\n' ...
    'u54 = reshape(u54,54,1);\n' ...
    'i_cmd = reshape(i_cmd,4,1);\n' ...
    'h = reshape(h,227,4);\n' ...
    'a_filt = reshape(a_filt,4,1);\n' ...
    'v_prev = reshape(v_prev,4,1);\n' ...
    'cmpRD = u54(1:4);\n' ...
    'cmpD = u54(51:54);\n' ...
    '[F_plant,h_new,a_filt_new,v_prev_new,init_new,x_plant,v_plant,a_plant,F_raw] = ...\n' ...
    '    func_NLCSNNPlant4(NLCSNN.net,cmpD,cmpRD,i_cmd,h,a_filt,v_prev,init_done,NLCSNN);\n' ...
    'end\n']);
end
