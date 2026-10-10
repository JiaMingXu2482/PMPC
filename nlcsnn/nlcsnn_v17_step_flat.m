function [F, h_new] = nlcsnn_v17_step_flat(net,x,v,a,curr,temp,dt,h)
%NLCSNN_V17_STEP_FLAT One v17 step with the Simulink 217-vector state.
% The first 32 values are the 2x16 GRU state; the remainder is CurrentLag.
%#codegen
% CarSim/controller signals can be single. The v17 weights and state are
% double; keep every recurrent input double to avoid codegen type changes.
hd = double(h);
st = struct('h',reshape(hd(1:32),2,16), ...
    'ie_buf',reshape(hd(33:217),185,1));
[F, st_new] = v17_damper_step(net,double(x),double(v),double(a), ...
    double(curr),0,double(temp),double(dt),st);
h_new = [reshape(st_new.h,32,1);st_new.ie_buf];
end
