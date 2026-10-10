function [x,status] = func_QPInteriorPriority(H,f,A,b,lb,ub,x0) %#codegen
% R2024b MPC Toolbox primal-dual solver for nearly-linear priority stages.
% Avoid the enormous unconstrained point H\f of an active-set LP surrogate.
n=numel(f);
G=[A;eye(n);-eye(n)]; h=[b;ub;-lb];
rs=max(sqrt(sum(G.^2,2)),1e-12);
G=bsxfun(@rdivide,G,rs); h=h./rs;
options=mpcInteriorPointOptions;
options.MaxIterations=100;
options.ConstraintTolerance=1e-9;
options.OptimalityTolerance=1e-9;
options.ComplementarityTolerance=1e-10;
options.StepTolerance=1e-12;
options.IntegrityChecks=false;
[x,st,feasible]=mpcInteriorPointSolver(H,f,G,h,zeros(0,n),zeros(0,1),x0,options);
status=double(st);
if ~feasible && status>0, status=-3; end
end
