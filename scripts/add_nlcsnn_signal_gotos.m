function add_nlcsnn_signal_gotos()
%ADD_NLCSNN_SIGNAL_GOTOS Expose manual v17 replacement signals near twin.
% Does not modify NLCSNN_Interface, pmpc_block, or CarSim configuration.
startup_pmpc();
model='pmpc_mil';
assert(~bdIsLoaded(model), ...
    'Close loaded pmpc_mil first; then reopen the saved SLX after this edit.');
mil_init_PMPC();
load_system(model);

iface=[model '/NLCSNN_Interface'];
assert(strcmp(get_param([iface '/t_s'],'Port'),'9'), ...
    'Expected NLCSNN_Interface output 9 to be the existing Digital Clock.');
assert(strcmp(get_param([model '/Demux'],'Outputs'),'4'), ...
    'Expected the existing F_des Demux to have four outputs.');

% PMPC F_des is in controller order [L1 R1 L2 R2]. Its existing Demux
% already branches to Scope2, so attach the four Gotos to those ports.
corners={'L1','R1','L2','R2'};
fdes=get_param([model '/Demux'],'PortHandles');
for k=1:4
    tag=['Fdes_' corners{k}];
    dst=localGoto(model,tag,[95 595+34*k 205 613+34*k]);
    add_line(model,fdes.Outport(k),dst,'autorouting','on');
end

% split18 belongs to pick18 (controller sys), not CarSim u58. Tap the
% existing 58-wide CarSim measurement signal without changing its wiring.
spec=get_param([model '/CarSim_Output_58'],'PortHandles');
split=[model '/NLCSNN_u58_fields'];
add_block('simulink/Signal Routing/Demux',split, ...
    'Outputs','[4 46 4 4]','Position',[-620 375 -615 565]);
sp=get_param(split,'PortHandles');
add_line(model,spec.Outport(1),sp.Inport(1),'autorouting','on');
localDemux(model,'NLCSNN_cmpRD_carsim_order',[-555 395 -550 475]);
localDemux(model,'NLCSNN_cmpD_carsim_order',[-555 510 -550 590]);
add_line(model,sp.Outport(1), ...
    get_param([model '/NLCSNN_cmpRD_carsim_order'],'PortHandles').Inport(1), ...
    'autorouting','on');
add_line(model,sp.Outport(3), ...
    get_param([model '/NLCSNN_cmpD_carsim_order'],'PortHandles').Inport(1), ...
    'autorouting','on');
localTerminate(model,'TERM_u58_middle',sp.Outport(2),[-545 485 -525 505]);
localTerminate(model,'TERM_u58_accel',sp.Outport(4),[-545 600 -525 620]);

% CarSim four-corner order is [L1 L2 R1 R2]. Map its Demux outputs to
% controller-order labels [L1 R1 L2 R2] without swapping signal values.
map=[1 3 2 4];
rd=get_param([model '/NLCSNN_cmpRD_carsim_order'],'PortHandles');
d=get_param([model '/NLCSNN_cmpD_carsim_order'],'PortHandles');
for k=1:4
    tag=['cmpRD_' corners{k}];
    dst=localGoto(model,tag,[-445 460+34*k -330 478+34*k]);
    add_line(model,rd.Outport(map(k)),dst,'autorouting','on');
    tag=['cmpD_' corners{k}];
    dst=localGoto(model,tag,[-445 295+34*k -330 313+34*k]);
    add_line(model,d.Outport(map(k)),dst,'autorouting','on');
end

% Reuse the existing 1 kHz clock via the interface's exposed t_s output.
ip=get_param(iface,'PortHandles');
dst=localGoto(model,'clock',[-245 630 -155 648]);
add_line(model,ip.Outport(9),dst,'autorouting','on');

% The existing internal constant is NLCSNN.temp (=42.5 C at present),
% not the requested 30 C. Provide an independent 30 C source for the
% manually inserted v17 blocks, leaving the old model untouched.
source=[model '/NLCSNN_Manual_Temper_30C'];
add_block('simulink/Sources/Constant',source, ...
    'Value','30','Position',[-285 690 -240 712]);
cp=get_param(source,'PortHandles');
dst=localGoto(model,'Temper',[-195 692 -105 710]);
add_line(model,cp.Outport(1),dst,'autorouting','on');

set_param(model,'SimulationCommand','update');
save_system(model);
close_system(model,0);
fprintf('Saved 12 corner Gotos plus [clock] and [Temper] in pmpc_mil.slx.\n');
end

function port=localGoto(model,tag,position)
path=[model '/' tag '_Goto'];
add_block('simulink/Signal Routing/Goto',path, ...
    'GotoTag',tag,'TagVisibility','global','Position',position);
h=get_param(path,'PortHandles');
port=h.Inport(1);
end

function localDemux(model,name,position)
add_block('simulink/Signal Routing/Demux',[model '/' name], ...
    'Outputs','4','Position',position);
end

function localTerminate(model,name,src,position)
path=[model '/' name];
add_block('simulink/Sinks/Terminator',path,'Position',position);
h=get_param(path,'PortHandles');
add_line(model,src,h.Inport(1),'autorouting','on');
end
