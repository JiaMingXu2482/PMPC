function [F_plant,h_new,a_filt_new,v_prev_new,init_new, ...
          x_plant,v_plant,a_plant,F_raw] = ...
    func_NLCSNNPlant4State(net,x_mm,v_mmps,a_g,i_cmd,h,a_filt, ...
                           v_prev,init_done,cfg)
%FUNC_NLCSNNPLANT4STATE Feed the 1 kHz twin from shared physical states.
% States are in [L1;R1;L2;R2] order, rebound-positive, with acceleration g.
% The legacy plant adapter converts CarSim compression signs internally.
%#codegen
o=[1 3 2 4];
cmpD=zeros(4,1);
cmpRD=zeros(4,1);
cmpAD=zeros(4,1);
cmpD(o)=cfg.x_ref-reshape(x_mm,4,1);
cmpRD(o)=-reshape(v_mmps,4,1);
cmpAD(o)=-reshape(a_g,4,1);
[F_plant,h_new,a_filt_new,v_prev_new,init_new, ...
 x_plant,v_plant,a_plant,F_raw] = ...
    func_NLCSNNPlant4(net,cmpD,cmpRD,i_cmd,h,a_filt, ...
                      v_prev,init_done,cfg,cmpAD);
end
