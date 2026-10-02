function [targetKmh,FxDem] = func_CombinedSpeedReference( ...
    course,x,v,vTargetKmh,m)
%FUNC_COMBINEDSPEEDREFERENCE Common longitudinal schedule for all variants.
% Smoothly reduce the 90 km/h DLC request to 80 km/h on the recovery
% straight. FxDem is a positive, bounded service-brake request in newtons.
u = min(max((x-course.dlc_end_x)/course.speed_ramp_m,0),1);
blend = u*u*(3-2*u);
targetKmh = min(vTargetKmh, ...
    course.target_kmh+(course.turn_target_kmh-course.target_kmh)*blend);
speedGap = max(v-targetKmh/3.6,0);
FxDem = m*min(speedGap/0.4,1.5)*blend;
end
