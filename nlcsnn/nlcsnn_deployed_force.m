function F = nlcsnn_deployed_force(net, F_raw, v)
%NLCSNN_DEPLOYED_FORCE Optional dissipative envelope for vehicle coupling.
% Physical NLCSNN units: N and mm/s. This does not modify recurrent state.
% The same map is used for plant force, attainable bounds and inverse trials.
%#codegen
F = F_raw;
if isfield(net,'force_guard_enabled') && net.force_guard_enabled
    floor_force = net.force_guard_c_min*v;
    excess_power = (F_raw-floor_force)*v;
    positive_power = 0.5*(excess_power + sqrt(excess_power*excess_power + ...
        net.force_guard_power_eps*net.force_guard_power_eps));
    F = floor_force + positive_power*v/(v*v + ...
        net.force_guard_v_eps*net.force_guard_v_eps);
end
end
