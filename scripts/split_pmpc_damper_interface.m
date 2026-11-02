function split_pmpc_damper_interface()
%SPLIT_PMPC_DAMPER_INTERFACE Separate PMPC force solver from damper twin.
% One-time, non-destructive transformation of the current pmpc_mil.slx.
% The original 1 kHz forward subsystem is COPIED into the replaceable
% NLCSNN_Interface before its top-level instance is disconnected.
root = startup_pmpc();
model = 'pmpc_mil';
assert(~bdIsLoaded(model), ...
    'Close the loaded pmpc_mil first, preserving any intended edits.');
mil_init_PMPC();
load_system(model);
oldPlant = [model '/NLCSNN_Forward_1kHz'];
iface = [model '/NLCSNN_Interface'];
assert(getSimulinkBlockHandle(oldPlant)~=-1 && ...
    getSimulinkBlockHandle(iface)==-1, ...
    'Expected the unsplit pmpc_mil with one top-level forward plant.');

sf = sfroot;
controller = [model '/PMPC_MF'];
chart = sf.find('-isa','Stateflow.EMChart','Path',controller);
assert(numel(chart)==1,'Missing PMPC controller chart.');
chart.Script = fileread(which('pmpc_block.m'));
localSize(chart,'u','54');
localSize(chart,'F_lo','[4 1]');
localSize(chart,'F_hi','[4 1]');
localSize(chart,'sys','54');
localSize(chart,'F_des','[4 1]');
localSize(chart,'long_diag','20');
parameter=chart.find('-isa','Stateflow.Data','Name','PMPC_P');
assert(numel(parameter)==1,'Missing PMPC_P parameter.');
parameter.Scope='Parameter'; parameter.Tunable=false;

add_block('simulink/Ports & Subsystems/Subsystem',iface, ...
    'Position',[-45 303 175 512], 'BackgroundColor','lightBlue');
localDeleteDefaultPorts(iface);
localPort(iface,'In1','u54',1,25,75);
localPort(iface,'In1','F_des',2,25,125);
outNames={'F_plant','F_lo','F_hi','i_cmd','F_raw', ...
    'x_mm','v_mmps','a_g','t_s','T_C','F_des_v17'};
for k=1:numel(outNames)
    localPort(iface,'Out1',outNames{k},k,940,35+45*k);
end
forward=[iface '/Forward_1kHz'];
add_block(oldPlant,forward,'Position',[170 55 380 255]);

bounds=[iface '/Bounds_100Hz'];
add_block('simulink/User-Defined Functions/MATLAB Function',bounds, ...
    'Position',[455 330 620 440]);
bc=sf.find('-isa','Stateflow.EMChart','Path',bounds);
bc.Script=sprintf([ ...
    'function [F_lo,F_hi,Fgrid] = bounds_fn(h,x,v,a,NLCSNN)\n' ...
    '%%#codegen\n' ...
    'ctx = func_NLCSNNContext(NLCSNN.net,reshape(x,4,1), ...\n' ...
    '    reshape(v,4,1),reshape(a,4,1),reshape(h,217,4),NLCSNN);\n' ...
    'F_lo = ctx.F_lo; F_hi = ctx.F_hi; Fgrid = ctx.Fgrid;\n' ...
    'end\n']);
localChart(bc,{'h','[217 4]';'x','[4 1]';'v','[4 1]'; ...
    'a','[4 1]';'F_lo','[4 1]';'F_hi','[4 1]';'Fgrid','[9 4]'});

inverse=[iface '/Inverse_100Hz'];
add_block('simulink/User-Defined Functions/MATLAB Function',inverse, ...
    'Position',[665 105 825 260]);
ic=sf.find('-isa','Stateflow.EMChart','Path',inverse);
ic.Script=sprintf([ ...
    'function [i_cmd,F_pred] = inverse_fn(F_des,h,x,v,a,Fgrid,NLCSNN)\n' ...
    '%%#codegen\n' ...
    '[i_cmd,F_pred] = func_NLCSNNInverseBlock(NLCSNN.net, ...\n' ...
    '    reshape(F_des,4,1),reshape(x,4,1),reshape(v,4,1), ...\n' ...
    '    reshape(a,4,1),reshape(h,217,4),reshape(Fgrid,9,4),NLCSNN);\n' ...
    'end\n']);
localChart(ic,{'F_des','[4 1]';'h','[217 4]';'x','[4 1]'; ...
    'v','[4 1]';'a','[4 1]';'Fgrid','[9 4]'; ...
    'i_cmd','[4 1]';'F_pred','[4 1]'});

add_block('simulink/Math Operations/Gain',[iface '/a_mmps2_to_g'], ...
    'Gain','1/9806.65','Position',[675 570 755 600]);
add_block('simulink/Math Operations/Gain',[iface '/CarSim_to_v17_force'], ...
    'Gain','-1','Position',[150 440 220 470]);
add_block('simulink/Sources/Digital Clock',[iface '/Digital_Clock_1kHz'], ...
    'SampleTime','0.001','Position',[675 635 745 665]);
add_block('simulink/Sources/Constant',[iface '/Damper_Temperature_C'], ...
    'Value','NLCSNN.temp','Position',[675 690 745 720]);
add_block('simulink/Sinks/Terminator',[iface '/TERM_F_pred'], ...
    'Position',[850 270 870 290]);
link=@(from,to)add_line(iface,from,to,'autorouting','on');
link('u54/1','Forward_1kHz/1');
link('F_des/1','Inverse_100Hz/1');
link('Inverse_100Hz/1','Forward_1kHz/2');
link('Forward_1kHz/1','F_plant/1');
link('Forward_1kHz/2','Bounds_100Hz/1');
link('Forward_1kHz/3','Bounds_100Hz/2');
link('Forward_1kHz/4','Bounds_100Hz/3');
link('Forward_1kHz/5','Bounds_100Hz/4');
link('Forward_1kHz/2','Inverse_100Hz/2');
link('Forward_1kHz/3','Inverse_100Hz/3');
link('Forward_1kHz/4','Inverse_100Hz/4');
link('Forward_1kHz/5','Inverse_100Hz/5');
link('Bounds_100Hz/1','F_lo/1');
link('Bounds_100Hz/2','F_hi/1');
link('Bounds_100Hz/3','Inverse_100Hz/6');
link('Inverse_100Hz/1','i_cmd/1');
link('Inverse_100Hz/2','TERM_F_pred/1');
link('Forward_1kHz/6','F_raw/1');
link('Forward_1kHz/3','x_mm/1');
link('Forward_1kHz/4','v_mmps/1');
link('Forward_1kHz/5','a_mmps2_to_g/1');
link('a_mmps2_to_g/1','a_g/1');
link('Digital_Clock_1kHz/1','t_s/1');
link('Damper_Temperature_C/1','T_C/1');
link('F_des/1','CarSim_to_v17_force/1');
link('CarSim_to_v17_force/1','F_des_v17/1');

% Reconnect only the named PMPC/twin ports; preserve other CarSim wiring.
source=get_param([model '/CarSim_Output_54'],'PortHandles');
ctrl=get_param(controller,'PortHandles');
pack=get_param([model '/Fd_safe_demux'],'PortHandles');
for k=1:numel(ctrl.Inport), localClearInput(ctrl.Inport(k)); end
old=get_param(oldPlant,'PortHandles');
for k=1:numel(old.Inport), localClearInput(old.Inport(k)); end
localClearInput(pack.Inport(1));
delete_block(oldPlant);
ctrl=get_param(controller,'PortHandles');
twin=get_param(iface,'PortHandles');
pack=get_param([model '/Fd_safe_demux'],'PortHandles');
add_line(model,source.Outport(1),ctrl.Inport(1),'autorouting','on');
add_line(model,source.Outport(1),twin.Inport(1),'autorouting','on');
add_line(model,ctrl.Outport(2),twin.Inport(2),'autorouting','on');
add_line(model,twin.Outport(2),ctrl.Inport(2),'autorouting','on');
add_line(model,twin.Outport(3),ctrl.Inport(3),'autorouting','on');
add_line(model,twin.Outport(1),pack.Inport(1),'autorouting','on');

try
    set_param(model,'SimulationCommand','update');
catch ME
    fprintf(2,'%s\n',getReport(ME,'extended','hyperlinks','off'));
    rethrow(ME);
end
save_system(model);
close_system(model,0);
fprintf('PMPC solver and replaceable NLCSNN interface saved in %s.slx\n',model);
end

function localPort(parent,kind,name,index,x,y)
if strcmp(kind,'In1'), lib='simulink/Sources/In1';
else, lib='simulink/Sinks/Out1'; end
add_block(lib,[parent '/' name],'Port',num2str(index), ...
    'Position',[x y x+30 y+14]);
end

function localDeleteDefaultPorts(parent)
b=find_system(parent,'SearchDepth',1,'Type','Block');
for k=1:numel(b)
    if ~strcmp(b{k},parent) && any(strcmp(get_param(b{k},'BlockType'),{'Inport','Outport'}))
        delete_block(b{k});
    end
end
end

function localSize(chart,name,sizeText)
d=chart.find('-isa','Stateflow.Data','Name',name);
assert(numel(d)==1,'Missing chart data %s in %s',name,chart.Path);
d.Props.Array.Size=sizeText;
d.Props.Array.IsDynamic=false;
end

function localChart(chart,sizes)
assert(numel(chart)==1,'Expected one MATLAB Function chart.');
for k=1:size(sizes,1), localSize(chart,sizes{k,1},sizes{k,2}); end
p=chart.find('-isa','Stateflow.Data','Name','NLCSNN');
assert(numel(p)==1,'Missing NLCSNN parameter in %s',chart.Path);
p.Scope='Parameter'; p.Tunable=false;
chart.ChartUpdate='DISCRETE'; chart.SampleTime='0.01';
end

function localClearInput(port)
line=get_param(port,'Line');
if line~=-1, delete_line(line); end
end
