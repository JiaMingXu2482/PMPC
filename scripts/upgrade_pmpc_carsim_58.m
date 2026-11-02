function upgrade_pmpc_carsim_58()
%UPGRADE_PMPC_CARSIM_58 Wire four CarSim damper accelerations into PMPC twin.
% CarSim 55:58 = CmpAD_[L1 L2 R1 R2], compression-positive acceleration g.
startup_pmpc();
model='pmpc_mil';
assert(~bdIsLoaded(model),'Close loaded pmpc_mil before changing its SLX.');
mil_init_PMPC();
load_system(model);
iface=[model '/NLCSNN_Interface'];
forward=[iface '/Forward_1kHz'];
inverse=[iface '/NLCSNN_Inverse_100Hz'];
assert(getSimulinkBlockHandle(inverse)~=-1,'Separate inverse subsystem missing.');

% The 54 original measurements keep their controller indices unchanged.
spec=[model '/CarSim_Output_54'];
set_param(spec,'Dimensions','58','Name','CarSim_Output_58');
spec=[model '/CarSim_Output_58'];
controller=get_param([model '/PMPC_MF'],'PortHandles');
source=get_param(spec,'PortHandles');
localClear(controller.Inport(1));
split=[model '/CarSim_58_to_54'];
add_block('simulink/Signal Routing/Demux',split, ...
    'Outputs','[54 4]','Position',[1235 255 1240 335]);
sp=get_param(split,'PortHandles');
add_line(model,source.Outport(1),sp.Inport(1),'autorouting','on');
add_line(model,sp.Outport(1),controller.Inport(1),'autorouting','on');
add_block('simulink/Sinks/Terminator',[model '/TERM_CmpAD_58'], ...
    'Position',[1275 330 1295 350]);
add_line(model,'CarSim_58_to_54/2','TERM_CmpAD_58/1','autorouting','on');

% The original 1 kHz plant now consumes the measured acceleration rather
% than differentiating damper velocity. Other 54-channel model variants
% retain the legacy finite-difference path in func_NLCSNNPlant4.
set_param([iface '/u54'],'Name','u58');
set_param([forward '/u54'],'Name','u58');
sf=sfroot;
plant=sf.find('-isa','Stateflow.EMChart','Path',[forward '/plant_fn']);
assert(numel(plant)==1,'Missing forward MATLAB Function chart.');
plant.Script=sprintf([ ...
    'function [F_plant,h_new,a_filt_new,v_prev_new,init_new,x_plant,v_plant,a_plant,F_raw] = ...\n' ...
    '    plant_fn(u58,i_cmd,h,a_filt,v_prev,init_done,NLCSNN)\n' ...
    '%%#codegen\n' ...
    'u58 = reshape(u58,58,1);\n' ...
    'i_cmd = reshape(i_cmd,4,1);\n' ...
    'h = reshape(h,217,4);\n' ...
    'a_filt = reshape(a_filt,4,1);\n' ...
    'v_prev = reshape(v_prev,4,1);\n' ...
    'cmpRD = u58(1:4);\n' ...
    'cmpD = u58(51:54);\n' ...
    'cmpAD = u58(55:58);\n' ...
    '[F_plant,h_new,a_filt_new,v_prev_new,init_new,x_plant,v_plant,a_plant,F_raw] = ...\n' ...
    '    func_NLCSNNPlant4(NLCSNN.net,cmpD,cmpRD,i_cmd,h,a_filt,v_prev,init_done,NLCSNN,cmpAD);\n' ...
    'end\n']);
localSize(plant,'u58','58');

% This conversion is independent of either neural model. It allows the
% forward and inverse subsystems to be replaced independently later.
states=[iface '/Damper_States_12'];
add_block('simulink/User-Defined Functions/MATLAB Function',states, ...
    'Position',[250 440 435 555]);
sc=sf.find('-isa','Stateflow.EMChart','Path',states);
sc.Script=sprintf([ ...
    'function [x_mm,v_mmps,a_mmps2,a_g] = damper_states(u58,NLCSNN)\n' ...
    '%%#codegen\n' ...
    'u58=reshape(u58,58,1);\n' ...
    'order=[1 3 2 4];\n' ...
    'x_mm=NLCSNN.x_ref-u58(50+order);\n' ...
    'v_mmps=-u58(order);\n' ...
    'a_g=-u58(54+order);\n' ...
    'for j=1:4\n' ...
    '    if ~isfinite(x_mm(j)), x_mm(j)=NLCSNN.x_ref; end\n' ...
    '    if ~isfinite(v_mmps(j)), v_mmps(j)=0; end\n' ...
    '    if ~isfinite(a_g(j)), a_g(j)=0; end\n' ...
    'end\n' ...
    'a_mmps2=9806.65*a_g;\n' ...
    'end\n']);
for n={'u58','x_mm','v_mmps','a_mmps2','a_g'}
    name=n{1};
    if strcmp(name,'u58'), sizeText='58'; else, sizeText='[4 1]'; end
    localSize(sc,name,sizeText);
end
p=sc.find('-isa','Stateflow.Data','Name','NLCSNN');
p.Scope='Parameter'; p.Tunable=false;
add_line(iface,'u58/1','Damper_States_12/1','autorouting','on');

% Give the inverse the same twelve physical states, independently of the
% forward model's x/v/a output contract.
ip=get_param(inverse,'PortHandles');
for k=3:5, localClear(ip.Inport(k)); end
add_line(iface,'Damper_States_12/1','NLCSNN_Inverse_100Hz/3','autorouting','on');
add_line(iface,'Damper_States_12/2','NLCSNN_Inverse_100Hz/4','autorouting','on');
add_line(iface,'Damper_States_12/3','NLCSNN_Inverse_100Hz/5','autorouting','on');
localClear(get_param([iface '/x_mm'],'PortHandles').Inport(1));
localClear(get_param([iface '/v_mmps'],'PortHandles').Inport(1));
localClear(get_param([iface '/a_g'],'PortHandles').Inport(1));
add_line(iface,'Damper_States_12/1','x_mm/1','autorouting','on');
add_line(iface,'Damper_States_12/2','v_mmps/1','autorouting','on');
add_line(iface,'Damper_States_12/4','a_g/1','autorouting','on');
delete_block([iface '/a_mmps2_to_g']);

% Make the physical inputs visible on both replaceable subsystem borders.
localExternalPort(forward,3,'t_s');
localExternalPort(forward,4,'T_C');
localExternalPort(forward,5,'x_mm');
localExternalPort(forward,6,'v_mmps');
localExternalPort(forward,7,'a_g');
localExternalPort(inverse,7,'t_s');
localExternalPort(inverse,8,'T_C');
localExternalPort(inverse,9,'F_des_v17');
localExternalPort(inverse,10,'a_g');
add_line(iface,'Digital_Clock_1kHz/1','Forward_1kHz/3','autorouting','on');
add_line(iface,'Damper_Temperature_C/1','Forward_1kHz/4','autorouting','on');
add_line(iface,'Damper_States_12/1','Forward_1kHz/5','autorouting','on');
add_line(iface,'Damper_States_12/2','Forward_1kHz/6','autorouting','on');
add_line(iface,'Damper_States_12/4','Forward_1kHz/7','autorouting','on');
add_line(iface,'Digital_Clock_1kHz/1','NLCSNN_Inverse_100Hz/7','autorouting','on');
add_line(iface,'Damper_Temperature_C/1','NLCSNN_Inverse_100Hz/8','autorouting','on');
add_line(iface,'CarSim_to_v17_force/1','NLCSNN_Inverse_100Hz/9','autorouting','on');
add_line(iface,'Damper_States_12/4','NLCSNN_Inverse_100Hz/10','autorouting','on');

set_param(model,'SimulationCommand','update');
save_system(model);
close_system(model,0);
fprintf('Saved pmpc_mil.slx with 58 CarSim exports and shared four-corner states.\n');
end

function localSize(chart,name,sizeText)
d=chart.find('-isa','Stateflow.Data','Name',name);
assert(numel(d)==1,'Missing %s in %s',name,chart.Path);
d.Props.Array.Size=sizeText;
d.Props.Array.IsDynamic=false;
end

function localClear(port)
line=get_param(port,'Line');
if line~=-1, delete_line(line); end
end

function localExternalPort(parent,index,name)
add_block('simulink/Sources/In1',[parent '/' name], ...
    'Port',num2str(index),'Position',[30 390+index*35 60 405+index*35]);
add_block('simulink/Sinks/Terminator',[parent '/TERM_' name], ...
    'Position',[155 390+index*35 175 405+index*35]);
add_line(parent,[name '/1'],['TERM_' name '/1'],'autorouting','on');
end
