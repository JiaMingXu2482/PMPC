function [i_cmd,F_pred] = nlcsnn_v17_inverse(net,x,v,a,F_des, ...
    temp,h,i_max,i_prev,Fgrid)
%NLCSNN_V17_INVERSE Invert the same guarded v17 map as the 1 kHz plant.
% The packaged fast bisection masks current and omits CurrentLag; using it
% here would make the attainable box, prediction, and plant inconsistent.
%#codegen
Igrid = (0:8)'*(i_max/8);
i_prev = min(max(i_prev,0),i_max);
force_tol = 1; % N; avoid current chatter for nearly equivalent forces
F_prev = nlcsnn_v17_force_at_current(net,x,v,a,i_prev,temp,h);
i_cmd = i_prev;
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
% A full v17 F(I) need not be monotone even though its two G heads are.
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
for k = 1:4
    den = Fhi-Flo;
    if abs(den) < 1e-9
        trial = 0.5*(lo+hi);
    else
        trial = min(max(lo+(target-Flo)*(hi-lo)/den,lo),hi);
    end
    Ft = nlcsnn_v17_force_at_current(net,x,v,a,trial,temp,h);
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
    i_cmd = i_prev;
    F_pred = F_prev;
end
end
