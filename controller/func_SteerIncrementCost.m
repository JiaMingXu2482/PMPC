function [H,f,c]=func_SteerIncrementCost(nu,nc,nvar,duMax,previous,weight) %#codegen
% Convex L3 penalty on changes in planned steering increments, including
% the discontinuity from the last EXECUTED increment to the first new one.
% No output filter: the optimized first control is the executed command.
D=zeros(nc,nvar);
for j=1:nc
    D(j,(j-1)*nu+1)=1;
    if j>1, D(j,(j-2)*nu+1)=-1; end
end
b=zeros(nc,1); b(1)=previous;
w=max(weight,0)/max(duMax^2,1e-16);
H=2*w*(D'*D); f=-2*w*(D'*b); c=w*(b'*b);
end
