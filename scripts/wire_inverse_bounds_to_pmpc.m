function wire_inverse_bounds_to_pmpc()
%WIRE_INVERSE_BOUNDS_TO_PMPC Connect four v17 force envelopes to PMPC.
% Controller corner order: [L1;R1;L2;R2], CarSim force sign, newtons.
startup_pmpc();
model='pmpc_mil';
assert(~bdIsLoaded(model), ...
    'Close loaded pmpc_mil before updating the saved SLX.');
mil_init_PMPC();
load_system(model);

controller=get_param([model '/PMPC_MF'],'PortHandles');
assert(numel(controller.Inport)>=3, 'PMPC_MF needs u, F_lo, F_hi inputs.');
assert(get_param(controller.Inport(2),'Line')==-1 && ...
       get_param(controller.Inport(3),'Line')==-1, ...
       'F_lo or F_hi is already wired; preserve the existing connection.');

corners={'L1','R1','L2','R2'};
loMux=[model '/PMPC_Flo_4corners'];
hiMux=[model '/PMPC_Fhi_4corners'];
add_block('simulink/Signal Routing/Mux',loMux, ...
    'Inputs','4','Position',[-195 235 -190 345]);
add_block('simulink/Signal Routing/Mux',hiMux, ...
    'Inputs','4','Position',[-195 370 -190 480]);
loPorts=get_param(loMux,'PortHandles');
hiPorts=get_param(hiMux,'PortHandles');

for k=1:4
    corner=corners{k};
    loTag=['Flo_' corner];
    hiTag=['Fhhi_' corner]; % Existing user Goto spelling for Fhi.
    assert(~isempty(find_system(model,'SearchDepth',1, ...
        'BlockType','Goto','GotoTag',loTag)), ...
        'Missing inverse lower-bound Goto %s',loTag);
    assert(~isempty(find_system(model,'SearchDepth',1, ...
        'BlockType','Goto','GotoTag',hiTag)), ...
        'Missing inverse upper-bound Goto %s',hiTag);
    loFrom=[model '/PMPC_From_' loTag];
    hiFrom=[model '/PMPC_From_Fhi_' corner];
    add_block('simulink/Signal Routing/From',loFrom, ...
        'GotoTag',loTag,'Position',[-340 240+25*k -260 258+25*k]);
    add_block('simulink/Signal Routing/From',hiFrom, ...
        'GotoTag',hiTag,'Position',[-340 375+25*k -260 393+25*k]);
    lp=get_param(loFrom,'PortHandles');
    hp=get_param(hiFrom,'PortHandles');
    add_line(model,lp.Outport(1),loPorts.Inport(k),'autorouting','on');
    add_line(model,hp.Outport(1),hiPorts.Inport(k),'autorouting','on');
end
% Flo/Fhi are returned by the same stateful inverse blocks that consume
% this tick's F_des. One controller-tick delay breaks the otherwise direct
% PMPC -> F_des -> inverse bounds -> PMPC algebraic loop.
loDelay=[model '/PMPC_Flo_previous_tick'];
hiDelay=[model '/PMPC_Fhi_previous_tick'];
add_block('simulink/Discrete/Unit Delay',loDelay, ...
    'SampleTime','0.01','InitialCondition','zeros(4,1)', ...
    'Position',[-130 275 -95 305]);
add_block('simulink/Discrete/Unit Delay',hiDelay, ...
    'SampleTime','0.01','InitialCondition','zeros(4,1)', ...
    'Position',[-130 410 -95 440]);
ld=get_param(loDelay,'PortHandles');
hd=get_param(hiDelay,'PortHandles');
add_line(model,loPorts.Outport(1),ld.Inport(1),'autorouting','on');
add_line(model,hiPorts.Outport(1),hd.Inport(1),'autorouting','on');
add_line(model,ld.Outport(1),controller.Inport(2),'autorouting','on');
add_line(model,hd.Outport(1),controller.Inport(3),'autorouting','on');

set_param(model,'SimulationCommand','update');
save_system(model);
close_system(model,0);
fprintf('Connected [Flo_L1;R1;L2;R2] and [Fhi_L1;R1;L2;R2] to PMPC_MF.\n');
end
