function [curve, out_of_domain] = nlcsnn_v14_curve_lut(lut, x, v, a)
%NLCSNN_V14_CURVE_LUT  Trilinear force-current curves for four corners.
% Table force uses CarSim polarity. Kinematics outside axes are clamped.
%#codegen
curve = zeros(numel(lut.i_axis_A), 4, 'single');
out_of_domain = false(4,1);
for corner = 1:4
    [ix, tx, ox] = bracket(lut.x_axis_mm, single(x(corner)));
    [iv, tv, ov] = bracket(lut.v_axis_mmps, single(v(corner)));
    [ia, ta, oa] = bracket(lut.a_axis_mmps2, single(a(corner)));
    out_of_domain(corner) = ox || ov || oa;
    for dx = 0:1
        wx = (1-tx)*(1-dx) + tx*dx;
        for dv = 0:1
            wv = (1-tv)*(1-dv) + tv*dv;
            for da = 0:1
                wa = (1-ta)*(1-da) + ta*da;
                values = reshape(lut.force_N(ix+dx,iv+dv,ia+da,:),[],1);
                curve(:,corner) = curve(:,corner) + wx*wv*wa*values;
            end
        end
    end
end
end

function [idx, t, outside] = bracket(axis, value)
n = numel(axis);
outside = ~isfinite(value) || value < axis(1) || value > axis(n);
value = min(max(value,axis(1)),axis(n));
idx = n-1;
for j = 1:n-1
    if value <= axis(j+1)
        idx = j;
        break
    end
end
t = (value-axis(idx))/(axis(idx+1)-axis(idx));
end
