function bounds = nlcsnn_v14_force_bounds_lut(lut, x, v, a)
%NLCSNN_V14_FORCE_BOUNDS_LUT  Reachable force interval from fixed table.
%#codegen
[curve, outside] = nlcsnn_v14_curve_lut(lut,x,v,a);
bounds = struct('lower',min(curve,[],1)', ...
    'upper',max(curve,[],1)', 'out_of_domain',outside);
end
