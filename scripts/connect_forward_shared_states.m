function connect_forward_shared_states()
%CONNECT_FORWARD_SHARED_STATES Feed forward and inverse from the same 12 states.
startup_pmpc();
model='pmpc_mil';
assert(~bdIsLoaded(model),'Close loaded pmpc_mil before updating disk SLX.');
mil_init_PMPC();
load_system(model);
forward=[model '/NLCSNN_Interface/Forward_1kHz'];
chartBlock=[forward '/plant_fn'];
sf=sfroot;
chart=sf.find('-isa','Stateflow.EMChart','Path',chartBlock);
assert(numel(chart)==1,'Missing forward chart.');
old=get_param(chartBlock,'PortHandles');
assert(numel(old.Inport)==6,'Unexpected forward chart input layout.');
src=zeros(1,5);
for k=2:6
    line=get_param(old.Inport(k),'Line');
    src(k-1)=get_param(line,'SrcPortHandle');
end
for k=1:6
    line=get_param(old.Inport(k),'Line');
    if line~=-1, delete_line(line); end
end
for n={'x_mm','v_mmps','a_g'}
    term=[forward '/TERM_' n{1}];
    if getSimulinkBlockHandle(term)~=-1, delete_block(term); end
end
add_block('simulink/Sinks/Terminator',[forward '/TERM_u58'], ...
    'Position',[110 450 130 470]);
add_line(forward,'u58/1','TERM_u58/1','autorouting','on');

chart.Script=sprintf([ ...
    'function [F_plant,h_new,a_filt_new,v_prev_new,init_new,x_plant,v_plant,a_plant,F_raw] = ...\n' ...
    '    plant_fn(x_mm,v_mmps,a_g,i_cmd,h,a_filt,v_prev,init_done,NLCSNN)\n' ...
    '%%#codegen\n' ...
    'x_mm=reshape(x_mm,4,1); v_mmps=reshape(v_mmps,4,1);\n' ...
    'a_g=reshape(a_g,4,1); i_cmd=reshape(i_cmd,4,1);\n' ...
    'h=reshape(h,217,4); a_filt=reshape(a_filt,4,1);\n' ...
    'v_prev=reshape(v_prev,4,1);\n' ...
    '[F_plant,h_new,a_filt_new,v_prev_new,init_new,x_plant,v_plant,a_plant,F_raw] = ...\n' ...
    '    func_NLCSNNPlant4State(NLCSNN.net,x_mm,v_mmps,a_g,i_cmd,h,a_filt,v_prev,init_done,NLCSNN);\n' ...
    'end\n']);
for n={'x_mm','v_mmps','a_g','i_cmd','a_filt','v_prev'}
    d=chart.find('-isa','Stateflow.Data','Name',n{1});
    assert(numel(d)==1,'Missing chart signal %s',n{1});
    d.Props.Array.Size='[4 1]';
    d.Props.Array.IsDynamic=false;
end
h=chart.find('-isa','Stateflow.Data','Name','h');
h.Props.Array.Size='[217 4]'; h.Props.Array.IsDynamic=false;
new=get_param(chartBlock,'PortHandles');
assert(numel(new.Inport)==8,'Forward chart did not expose eight inputs.');
states={'x_mm','v_mmps','a_g'};
for k=1:3
    p=get_param([forward '/' states{k}],'PortHandles');
    add_line(forward,p.Outport(1),new.Inport(k),'autorouting','on');
end
for k=1:5
    add_line(forward,src(k),new.Inport(k+3),'autorouting','on');
end
set_param(model,'SimulationCommand','update');
save_system(model);
close_system(model,0);
fprintf('Saved common physical-state feeds to forward and inverse twins.\n');
end
