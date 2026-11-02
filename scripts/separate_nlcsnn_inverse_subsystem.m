function separate_nlcsnn_inverse_subsystem()
%SEPARATE_NLCSNN_INVERSE_SUBSYSTEM Give the 10 ms inverse its own subsystem.
% Preserve the existing forward model, inverse code, and all signal routes.
startup_pmpc();
model='pmpc_mil';
assert(~bdIsLoaded(model), ...
    'Close the loaded pmpc_mil before updating its SLX on disk.');
mil_init_PMPC();
load_system(model);
iface=[model '/NLCSNN_Interface'];
old=[iface '/Inverse_100Hz'];
assert(getSimulinkBlockHandle(old)~=-1, 'Missing existing inverse block.');
assert(strcmp(get_param(old,'BlockType'),'SubSystem'), ...
    'Unexpected inverse block type.');
assert(strcmp(get_param(old,'SFBlockType'),'MATLAB Function'), ...
    'Inverse_100Hz has already been wrapped or changed.');

oldPorts=get_param(old,'PortHandles');
src=zeros(1,numel(oldPorts.Inport));
for k=1:numel(src)
    line=get_param(oldPorts.Inport(k),'Line');
    src(k)=get_param(line,'SrcPortHandle');
end
dst=cell(1,numel(oldPorts.Outport));
for k=1:numel(dst)
    line=get_param(oldPorts.Outport(k),'Line');
    dst{k}=get_param(line,'DstPortHandle');
end
pos=get_param(old,'Position');
wrapper=[iface '/NLCSNN_Inverse_100Hz'];
add_block('simulink/Ports & Subsystems/Subsystem',wrapper, ...
    'Position',pos,'BackgroundColor','lightBlue');
for p=find_system(wrapper,'SearchDepth',1,'Type','Block')'
    path=p{1};
    if ~strcmp(path,wrapper) && any(strcmp(get_param(path,'BlockType'),{'Inport','Outport'}))
        delete_block(path);
    end
end
core=[wrapper '/Inverse_Core'];
add_block(old,core,'Position',[180 80 375 245]);
inputNames={'F_des','h','x','v','a','Fgrid'};
for k=1:numel(inputNames)
    name=inputNames{k};
    add_block('simulink/Sources/In1',[wrapper '/' name], ...
        'Port',num2str(k),'Position',[25 40*k 55 40*k+14]);
    add_line(wrapper,[name '/1'],['Inverse_Core/' num2str(k)], ...
        'autorouting','on');
end
outputNames={'i_cmd','F_pred'};
for k=1:numel(outputNames)
    name=outputNames{k};
    add_block('simulink/Sinks/Out1',[wrapper '/' name], ...
        'Port',num2str(k),'Position',[470 95*k 500 95*k+14]);
    add_line(wrapper,['Inverse_Core/' num2str(k)],[name '/1'], ...
        'autorouting','on');
end
delete_block(old);
newPorts=get_param(wrapper,'PortHandles');
for k=1:numel(src)
    add_line(iface,src(k),newPorts.Inport(k),'autorouting','on');
end
for k=1:numel(dst)
    for j=1:numel(dst{k})
        line=get_param(dst{k}(j),'Line');
        if line~=-1, delete_line(line); end
        add_line(iface,newPorts.Outport(k),dst{k}(j),'autorouting','on');
    end
end
save_system(model);
close_system(model,0);
fprintf('Saved separate forward/inverse subsystems in %s.slx\n',model);
end
