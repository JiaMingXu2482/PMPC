function F = fwd_mf(t, Temp_C, cmpD, cmpRD, I_cmd, cid)
%#codegen
F = fwd_mf_impl(t, Temp_C, cmpD, cmpRD, I_cmd, cid);
