function [i_cmd, F_est] = nlcsnn_v14_inverse_lut(lut,x,v,a,F_des,i_prev)
%NLCSNN_V14_INVERSE_LUT  Invert a fixed force-current table, no NN calls.
% Current endpoints are 0 and 1.6 A; low authority holds the last command.
%#codegen
[curve,~] = nlcsnn_v14_curve_lut(lut,x,v,a);
i_cmd = zeros(4,1,'single');
F_est = zeros(4,1,'single');
axis = lut.i_axis_A(:);
for corner = 1:4
    y = curve(:,corner);
    previous = min(max(single(i_prev(corner)),axis(1)),axis(end));
    [lo,lo_idx] = min(y);
    [hi,hi_idx] = max(y);
    request = single(F_des(corner));
    if ~isfinite(request) || hi-lo < lut.authority_tol_N
        i_cmd(corner) = previous;
        F_est(corner) = force_at_current(axis,y,previous);
    elseif request <= lo
        i_cmd(corner) = axis(lo_idx);
        F_est(corner) = lo;
    elseif request >= hi
        i_cmd(corner) = axis(hi_idx);
        F_est(corner) = hi;
    else
        best_distance = single(inf);
        best_i = previous;
        best_f = force_at_current(axis,y,previous);
        for j = 1:numel(axis)-1
            y0 = y(j); y1 = y(j+1);
            if request >= min(y0,y1) && request <= max(y0,y1) && abs(y1-y0)>1e-6
                alpha = (request-y0)/(y1-y0);
                candidate = axis(j) + alpha*(axis(j+1)-axis(j));
                distance = abs(candidate-previous);
                if distance < best_distance
                    best_distance = distance;
                    best_i = candidate;
                    best_f = request;
                end
            end
        end
        i_cmd(corner) = best_i;
        F_est(corner) = best_f;
    end
end
end

function force = force_at_current(axis,y,current)
force = y(end);
for j = 1:numel(axis)-1
    if current <= axis(j+1)
        t = (current-axis(j))/(axis(j+1)-axis(j));
        force = y(j) + t*(y(j+1)-y(j));
        return
    end
end
end
