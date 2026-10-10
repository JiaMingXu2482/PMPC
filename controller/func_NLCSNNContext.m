function ctx = func_NLCSNNContext(net, x, v, a, h, cfg)
%FUNC_NLCSNNCONTEXT  100 Hz controller side: reachable force box from plant state.
%
%   In the multirate architecture the 1 kHz plant (func_NLCSNNPlant4) owns
%   the hidden state h and the x/v/a kinematics. The controller samples
%   (x, v, a, h) at the tick and only evaluates the reachable force box
%   here; it never integrates h.
%
%   Corner order is [L1; R1; L2; R2] for all inputs (controller order).
%   Force bounds use CarSim polarity (F_CarSim = -F_NLCSNN).
%
%   V14 scans nine currents because its dynamic force map need not be
%   monotone. The samples are reused by the inverse solver at this tick.

%#codegen

x = reshape(x, 4, 1);
v = reshape(v, 4, 1);
a = reshape(a, 4, 1);

F_lo = zeros(4,1);
F_hi = zeros(4,1);
Fgrid = zeros(9,4);
for j = 1:4
    xj = x(j); vj = v(j); aj = a(j);
    if ~isfinite(xj), xj = cfg.x_ref; end
    if ~isfinite(vj), vj = 0; end
    if ~isfinite(aj), aj = 0; end
    hj = h(:,j);
    if isfield(net, 'lag_kmax')
        for q = 1:9
            Fgrid(q,j) = nlcsnn_v17_force_at_current(net,xj,vj,aj, ...
                (q-1)*cfg.i_max/8,cfg.temp,hj);
        end
        F0 = -max(Fgrid(:,j));
        Fmax = -min(Fgrid(:,j));
    elseif isfield(net, 'weights')
        for q = 1:9
            Fgrid(q,j) = nlcsnn_v14_force_at_current(net,xj,vj,aj, ...
                (q-1)*cfg.i_max/8,cfg.temp,hj);
        end
        F0 = -max(Fgrid(:,j));
        Fmax = -min(Fgrid(:,j));
    else
        F0 = -nlcsnn_predict_force(net,xj,vj,aj,0,cfg.temp,hj);
        Fmax = -nlcsnn_predict_force(net,xj,vj,aj, ...
            cfg.i_max,cfg.temp,hj);
    end
    if ~isfinite(F0),   F0 = 0; end
    if ~isfinite(Fmax), Fmax = 0; end
    F_lo(j) = min(F0, Fmax);
    F_hi(j) = max(F0, Fmax);
end

ctx = struct('x', x, 'v', v, 'a', a, 'h', h, ...
             'F_lo', F_lo, 'F_hi', F_hi, ...
             'F_center', 0.5*(F_lo + F_hi), 'Fgrid', Fgrid);
end
