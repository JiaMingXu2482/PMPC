function [i_cmd, F_pred] = nlcsnn_v14_inverse(net, x, v, a, F_des, ...
    temp, h, i_max, i_prev, Fgrid)
%NLCSNN_V14_INVERSE Invert the full v14 one-step force map at fixed state.
% Nine samples locate a reachable branch, including nonmonotone curves.
% Two safeguarded secant steps refine it. Within 1 N force equivalence,
% prefer the preceding current rather than chasing numerical differences.
%#codegen
Igrid = zeros(9,1);
for k = 1:9
    Igrid(k) = (k-1)*i_max/8;
end
i_cmd = min(max(i_prev,0),i_max);
force_tol = 1.0; % N; matches the legacy inverse's weak-authority threshold
F_prev = nlcsnn_v14_force_at_current(net,x,v,a,i_cmd,temp,h);
F_pred = F_prev;
target = min(max(F_des,min(Fgrid)),max(Fgrid));
if max(Fgrid)-min(Fgrid) < force_tol || abs(F_prev-target) <= force_tol
    return
end
best = abs(F_prev-target);
for k = 1:9
    e = abs(Fgrid(k)-target);
    if e < best-force_tol || ...
            (e <= best && abs(Igrid(k)-i_prev) < abs(i_cmd-i_prev))
        best = e; i_cmd = Igrid(k); F_pred = Fgrid(k);
    end
end
% Prefer the attainable crossing nearest the preceding command.
bracket = 0;
distance = inf;
for k = 1:8
    if (Fgrid(k)-target)*(Fgrid(k+1)-target) <= 0
        d = abs(0.5*(Igrid(k)+Igrid(k+1))-i_prev);
        if d < distance
            bracket = k; distance = d;
        end
    end
end
if bracket == 0
    return
end
lo = Igrid(bracket); hi = Igrid(bracket+1);
Flo = Fgrid(bracket); Fhi = Fgrid(bracket+1);
for k = 1:2
    den = Fhi-Flo;
    if abs(den) < 1e-9
        trial = 0.5*(lo+hi);
    else
        trial = lo + (target-Flo)*(hi-lo)/den;
        trial = min(max(trial,lo),hi);
    end
    Ft = nlcsnn_v14_force_at_current(net,x,v,a,trial,temp,h);
    e = abs(Ft-target);
    if e < best-force_tol || ...
            (e <= best+force_tol && abs(trial-i_prev) < abs(i_cmd-i_prev))
        best = e; i_cmd = trial; F_pred = Ft;
    end
    if (Flo-target)*(Ft-target) <= 0
        hi = trial; Fhi = Ft;
    else
        lo = trial; Flo = Ft;
    end
end
if abs(F_prev-target) <= best+force_tol
    i_cmd = min(max(i_prev,0),i_max);
    F_pred = F_prev;
end
end
