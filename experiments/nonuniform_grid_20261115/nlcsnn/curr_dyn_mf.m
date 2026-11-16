function I_act = curr_dyn_mf(t, I_cmd, cid)
%#codegen
I_act = curr_dyn_impl(t, I_cmd, cid);
