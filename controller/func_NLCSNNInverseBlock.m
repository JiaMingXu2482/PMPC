function [i_cmd,F_pred] = func_NLCSNNInverseBlock( ...
    net,F_des,x,v,a,h,Fgrid,cfg)
%FUNC_NLCSNNINVERSEBLOCK 10 ms inverse in the replaceable twin interface.
% Inputs/outputs use controller corner order [L1;R1;L2;R2]. F_des and
% F_pred use CarSim force polarity; the conversion to v17 is inside Apply.
% This state belongs to the inverse, not to the PMPC optimizer.
%#codegen
persistent i_prev
if isempty(i_prev)
    i_prev = zeros(4,1);
end
ctx = struct('x',reshape(x,4,1),'v',reshape(v,4,1), ...
    'a',reshape(a,4,1),'h',reshape(h,217,4), ...
    'Fgrid',reshape(Fgrid,9,4));
[i_cmd,F_pred,i_prev] = func_NLCSNNApply( ...
    net,ctx,reshape(F_des,4,1),i_prev,cfg);
end
