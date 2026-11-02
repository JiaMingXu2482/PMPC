function revert_pmpc_carsim_to_54()
%REVERT_PMPC_CARSIM_TO_54 Keep manual twin taps without CmpAD channels.
% Only top-level CarSim-vector adaptation and clock tap are changed.
startup_pmpc();
model='pmpc_mil';
assert(~bdIsLoaded(model), ...
    'Close loaded pmpc_mil before editing its saved SLX.');
mil_init_PMPC();
load_system(model);

spec=[model '/CarSim_Output_58'];
assert(getSimulinkBlockHandle(spec)~=-1,'Missing 58-channel signal specification.');
set_param(spec,'Dimensions','54','Name','CarSim_Output_54');
spec=[model '/CarSim_Output_54'];

% Remove only the obsolete [54 4] splitter. The controller's original
% 54 measurements now branch directly from the CarSim signal specification.
oldSplit=[model '/CarSim_58_to_54'];
ctrl=get_param([model '/PMPC_MF'],'PortHandles');
line=get_param(ctrl.Inport(1),'Line');
if line~=-1, delete_line(line); end
delete_block(oldSplit);
term=[model '/TERM_CmpAD_58'];
if getSimulinkBlockHandle(term)~=-1, delete_block(term); end
source=get_param(spec,'PortHandles');
add_line(model,source.Outport(1),ctrl.Inport(1),'autorouting','on');

% Existing Gotos for CmpRD (channels 1:4) and CmpD (51:54) retain their
% corner mapping [L1 R1 L2 R2]. Drop only the nonexistent 55:58 group.
fields=[model '/NLCSNN_u58_fields'];
assert(getSimulinkBlockHandle(fields)~=-1,'Missing damper signal splitter.');
term=[model '/TERM_u58_accel'];
if getSimulinkBlockHandle(term)~=-1, delete_block(term); end
set_param(fields,'Outputs','[4 46 4]','Name','NLCSNN_u54_fields');

% The old interface, which supplied the clock, has been removed by the
% manual migration. Restore the requested global clock tag independently.
clockTag=[model '/clock_Goto'];
assert(getSimulinkBlockHandle(clockTag)~=-1,'Missing [clock] Goto.');
clockIn=get_param(clockTag,'PortHandles');
line=get_param(clockIn.Inport(1),'Line');
if line~=-1, delete_line(line); end
clock=[model '/NLCSNN_Manual_Clock_1kHz'];
add_block('simulink/Sources/Digital Clock',clock, ...
    'SampleTime','0.001','Position',[-285 635 -235 665]);
set_param(clockTag,'Position',[-185 640 -95 658]);
clockOut=get_param(clock,'PortHandles');
add_line(model,clockOut.Outport(1),clockIn.Inport(1),'autorouting','on');

% Save the requested interface edit even if the user has not yet connected
% the new forward/inverse models to PMPC bounds and the plant-force port.
save_system(model);
try
    set_param(model,'SimulationCommand','update');
    fprintf('54-channel pmpc_mil model update compiled successfully.\n');
catch ME
    fprintf(2,'54-channel wiring saved; model update remains incomplete: %s\n',ME.message);
    close_system(model,0);
    rethrow(ME);
end
close_system(model,0);
fprintf('Saved pmpc_mil.slx with 54 CarSim outputs and 12 signal Gotos.\n');
end
