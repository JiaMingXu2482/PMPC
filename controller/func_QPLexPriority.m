function [x,status] = func_QPLexPriority(H,f,A,b,lb,ub,nDU,slackPhysicalScale,tol) %#codegen
% Three tolerance-qualified priority stages, on the SAME reduced variables.
% Inputs use the outer solver's Jacobi coordinates; physical slack=sc*x.
% Small bounded regularization makes the first two LPs KWIK-compatible QPs.
% Its worst-case effect on each physical slack is bounded below tol/8.
n=numel(f); js=nDU+(1:2);
scale=max(max(abs(lb),abs(ub)),1e-8);
scale(js)=1;
lo=lb; hi=ub;
% Bound each slack by the worst violation over the finite input/gamma box.
% This cannot exclude an optimum: a larger slack only worsens every stage.
otherHi=hi; otherLo=lo; otherHi(js)=0; otherLo(js)=0;
worst=max(A,0)*otherHi+min(A,0)*otherLo-b;
for k=1:2
    cap=0;
    for r=1:size(A,1)
        if A(r,js(k)) < -1e-12
            cap=max(cap,worst(r)/(-A(r,js(k))));
        end
    end
    hi(js(k))=min(hi(js(k)),max(cap,0)+1/slackPhysicalScale(k));
    scale(js(k))=max(hi(js(k)),1e-8);
end
An=bsxfun(@times,A,scale'); ln=lo./scale; un=hi./scale;
physical=scale(js).*slackPhysicalScale(:);
reg=min(1e-5,tol/(4*n*max(max(physical),1)));
Hp=reg*eye(n); fp=zeros(n,1); fp(js(1))=1;
[y,st]=func_QPInteriorPriority(Hp,fp,An,b,ln,un,(ln+un)/2);
x=zeros(n,1); status=st;
if st<=0, return; end
% Substitute each locked slack rather than keeping almost coincident bounds.
% For monotone softened inequalities, fixing the slack at its allowed cap
% preserves exactly the admissible controls of a slack upper-bound lock.
c1=needed_(An,b,y,js(1))+tol/physical(1);
c1=min(un(js(1)),max(0,c1));
free2=[1:nDU,nDU+2,nDU+3];
b2=b-An(:,js(1))*c1;
f2=zeros(n-1,1); f2(nDU+1)=1;
[v2,st]=func_QPInteriorPriority(reg*eye(n-1),f2,An(:,free2),b2,ln(free2),un(free2),y(free2));
status=st;
if st<=0, status=st-100; return; end
y(js(1))=c1; y(free2)=v2;
c2=needed_(An,b,y,js(2))+tol/physical(2);
c2=min(un(js(2)),max(0,c2));
un(js)=[c1;c2];
free3=[1:nDU,nDU+3];
fixed=zeros(n,1); fixed(js)=[c1;c2];
bf=b-An*fixed;
if st<=0, return; end
% Final tracking/effort objective; huge safety weights are not used.
f3=f; f3(js)=0;
H3=(scale*scale').*H; f3=scale.*f3;
f3=f3(free3)+H3(free3,:)*fixed;
H3=H3(free3,free3);
sc=1./sqrt(max(diag(H3),1e-14));
H3=(sc*sc').*H3; H3=(H3+H3')/2;
A3=bsxfun(@times,An(:,free3),sc');
[v,st,~]=func_QPKwikPriority(H3,sc.*f3,A3,bf,ln(free3)./sc,un(free3)./sc,2500,1e-9);
status=st;
if st<=0, status=st-200; return; end
if st>0
    y=fixed; y(free3)=sc.*v;
    % Do not accept a solver flag alone when checking priority locks.
    if max(An*y-b)>2e-5 || max(y-un)>2e-6 || max(ln-y)>2e-6 || ...
            max((y(js)-un(js)).*physical)>tol/10
        status=-4; return;
    end
    x=scale.*y;
end
end

function needed=needed_(A,b,x,j)
needed=max(x(j),0);
r=A*x-b;
for i=1:size(A,1)
    if A(i,j)<-1e-12
        needed=max(needed,x(j)+r(i)/(-A(i,j)));
    end
end
end
