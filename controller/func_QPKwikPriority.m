function [x,status,lam] = func_QPKwikPriority(H,f,A,b,lb,ub,maxiter,feasTol) %#codegen
% PMPC-only reduced QP; does not change the other controllers' solver.
% Multipliers retain [-A; I; -I] order for existing diagnostics.
n=numel(f);
if nargin<8, feasTol=1e-6; end
lbf=lb; ubf=ub;
lbf(~isfinite(lbf))=-1e12;
ubf(~isfinite(ubf))=1e12;
Am=[-A;eye(n);-eye(n)]; bm=[-b;lbf;-ubf];
% Jacobi scaling normalizes variables, not constraint rows. Normalize rows
% too; recover the original multipliers below (the feasible set is unchanged).
rowScale=max(sqrt(sum(Am.^2,2)),1e-12);
Am=bsxfun(@rdivide,Am,rowScale); bm=bm./rowScale;
m=numel(bm);
persistent active
if isempty(active), active=false(m,1); end
if numel(active)~=m, active=false(m,1); end
[L,p]=chol(H,'lower');
if p~=0
    x=zeros(n,1); status=-2; lam=zeros(m,1); active=false(m,1); return;
end
Linv=L\eye(n);
opt=mpcqpsolverOptions;
opt.MaxIter=maxiter;
opt.FeasibilityTol=feasTol;
opt.IntegrityChecks=false;
[x,st,ia,mult]=mpcqpsolver(Linv,f,Am,bm,zeros(0,n),zeros(0,1),active,opt);
if st<=0 && any(active)
    [x,st,ia,mult]=mpcqpsolver(Linv,f,Am,bm,zeros(0,n),zeros(0,1),false(m,1),opt);
end
status=double(st); lam=mult.ineqlin./rowScale;
if st>0
    active=ia;
else
    active=false(m,1);
end
end
