function [I_cmd, Flo, Fhi, F_clipped, F_ach] = inv_mf(Fd_N, t, Temp_C, cmpD, cmpRD, cid)
%#codegen
[I_cmd, Flo, Fhi, F_clipped, F_ach] = inv_mf_impl_10ms(Fd_N, t, Temp_C, cmpD, cmpRD, cid);
