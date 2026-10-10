function [T,proof] = func_TerminalRecoverabilityPreview(M,C,E,Pred,AI,zeta,Lim,umin,umax) %#codegen
% Affine backup-recoverability inner sets for the FROZEN terminal model.
% No added QP variables: backup increments are a fixed feedback of zeta_N.
% An invariant Lyapunov ellipsoid is inscribed in the physical constraints;
% an inscribed polytope gives linear terminal rows. Finite backup preimages
% provide recovery trajectories to that polytope. This does NOT certify nonlinear/variable-speed safety
% or recursive feasibility of the changing, softened online QP.
[n1,n2]=func_TerminalRowCounts(M);
nx=M.Nx; nu=M.Nu; na=nx+nu; nd=nu*M.Nc;
T=struct('A1',zeros(n1,nd),'b1',ones(n1,1), ...
    'A2',zeros(n2,nd),'b2',ones(n2,1),'diag',zeros(8,1));
proof=struct('P',eye(na),'Acl',eye(na),'K',zeros(nu,na), ...
    'alpha',zeros(2,1),'scale',ones(na,1),'center',zeros(na,1));
T.diag(1)=1; T.diag(4:5)=NaN;
Aa=Pred.Aterminal; Ba=Pred.Bterminal; d=Pred.dterminal;
du=[Lim.dFyfmax;Lim.dMFxmax;Lim.dMdmax];
us=max((umax-umin)/2,[.01;100;100]);
sx=[max(C.Vymax,.1); max(E.Gsh(1),.05); max(C.phimax,.01); ...
    max(C.dphimax,.01); max(C.emax,.1); max(C.epsimax,.01); ...
    max(max(abs(umin(3)),abs(umax(3))),100);us];
S=diag(sx); U=diag(us);
% Base equilibrium: ey=0, zero DB, CDC at its current admissible midpoint.
% A one-dimensional equilibrium family below can select nonzero steady DB.
% epsi is NOT forced to zero: steady sideslip requires a heading offset.
Eq=zeros(na); Eq(1:nx,:)=[eye(nx)-Aa(1:nx,1:nx),-Aa(1:nx,nx+(1:nu))];
Eq(nx+1,5)=1; Eq(nx+2,7)=1; Eq(nx+3,nx+2)=1;
rhs=[d(1:nx);0;(umin(3)+umax(3))/2;0];
Es=Eq*S; rs=max(sqrt(sum(Es.^2,2)),1e-12);
Es=bsxfun(@rdivide,Es,rs); rr=rhs./rs;
if rcond(Es)<1e-12, T.diag(8)=-1; return; end
ze=S*(Es\rr);
res=norm(Aa*ze+d-ze,inf);
T.diag(7)=res;
if ~all(isfinite(ze)) || res>1e-7*(1+norm(ze,inf))
    T.diag(8)=-2; return;
end
An=bsxfun(@rdivide,Aa*S,sx); Bn=bsxfun(@rdivide,Ba*U,sx);
fixed=(umax-umin<1e-8) | (du<1e-10);
Kfix=zeros(nu,na); Bfree=Bn;
for j=1:nu
    if fixed(j), Kfix(j,nx+j)=1; Bfree(:,j)=0; end
end
A0=An-Bn*Kfix;
% Penalize increments relative to their permitted step, not full amplitude.
% This avoids designing an unrealistically aggressive backup and then
% shrinking its invariant core to accommodate tiny rate limits.
Rbackup=diag(min((us./max(du,1e-6)).^2,1e8));
P=eye(na);
for j=1:120
    Kfree=(Rbackup+Bfree'*P*Bfree)\(Bfree'*P*A0);
    Pnew=eye(na)+A0'*P*A0-A0'*P*Bfree*Kfree;
    P=(Pnew+Pnew')/2;
end
Kfree=(Rbackup+Bfree'*P*Bfree)\(Bfree'*P*A0);
K=Kfix+Kfree; Ac=An-Bn*K;
[L,flag]=chol(P);
dec=(P-Ac'*P*Ac); dec=(dec+dec')/2;
[~,flagDec]=chol(dec-1e-9*eye(na));
if flag~=0 || flagDec~=0 || ~all(isfinite(P(:)))
    T.diag(8)=-3; return;
end
% Support covariance on the invariant subspace of any fixed input channel.
Pinv=P\eye(na);
for j=1:nu
    q=nx+j;
    if fixed(j)
        v=Pinv(:,q); Pinv=Pinv-(v*v')/max(Pinv(q,q),1e-16);
    end
end
Zu=[zeros(nu,nx),eye(nu)];
H1=[E.Hr E.Or;E.Henv zeros(4,nu)];
kap_core=Lim.kap(end);
if isfield(Pred,'kap_tail'), kap_core=Pred.kap_tail(end); end
g1=[E.Gr;E.Genv+kap_core*E.gkap];
H2=[E.Hsh zeros(4,nu)]; g2=E.Gsh;
sc1=[E.sc_r;E.sc_env]; sc2=E.sc_sh;
H1n=bsxfun(@rdivide,H1*S,sc1); h1=(g1-H1*ze)./sc1;
H2n=bsxfun(@rdivide,H2*S,sc2); h2=(g2-H2*ze)./sc2;
Hu=bsxfun(@rdivide,[Zu;-Zu]*S,[us;us]);
hu=[umax-ze(nx+(1:nu));ze(nx+(1:nu))-umin]./[us;us];
% K acts on normalized states and generates normalized input increments.
Hd=[-K;K]; hd=[du;du]./[us;us];
% Maximize the common admissible Lyapunov radius over steady DB equilibria.
% sqrt(alpha) is a concave piecewise-affine function of this scalar, so its
% maximum is attained at an endpoint or an intersection of two support lines.
% There is no rule prescribing the sign or activation of the optimized DB.
rhsDB=zeros(na,1); rhsDB(end)=us(2)/rs(end);
zeDB=S*(Es\rhsDB);
Hset=[H1n;H2n;Hu;Hd]; hset=[h1;h2;hu;hd];
dh=[-[H1n;H2n;Hu]*(zeDB./sx);zeros(2*nu,1)];
support=sqrt(max(sum((Hset*Pinv).*Hset,2),1e-20));
aa=hset./support; bb=dh./support;
v0=min(max(0,umin(2)/us(2)),umax(2)/us(2));
vbest=v0; score=min(aa+bb*v0);
for j=1:numel(aa)+2
    if j==numel(aa)+1
        v=umin(2)/us(2);
        q=min(aa+bb*v);
        if q>score+1e-10, score=q; vbest=v; end
    elseif j==numel(aa)+2
        v=umax(2)/us(2);
        q=min(aa+bb*v);
        if q>score+1e-10, score=q; vbest=v; end
    else
        for k=j+1:numel(aa)
            den=bb(j)-bb(k);
            if abs(den)>1e-12
                v=(aa(k)-aa(j))/den;
                if v>=umin(2)/us(2) && v<=umax(2)/us(2)
                    q=min(aa+bb*v);
                    if q>score+1e-10 || (abs(q-score)<=1e-10 && abs(v)<abs(vbest))
                        score=q; vbest=v;
                    end
                end
            end
        end
    end
end
ze=ze+zeDB*vbest;
h1=(g1-H1*ze)./sc1; h2=(g2-H2*ze)./sc2;
hu=[umax-ze(nx+(1:nu));ze(nx+(1:nu))-umin]./[us;us];
T.diag(7)=norm(Aa*ze+d-ze,inf);
[a1,ok1]=radius_([H1n;Hu;Hd],[h1;hu;hd],Pinv);
[a2,ok2]=radius_([H2n;Hu;Hd],[h2;hu;hd],Pinv);
proof.P=P; proof.Acl=Ac; proof.K=K; proof.alpha=[a1;a2];
proof.scale=sx; proof.center=ze;
T.diag(2:3)=[double(ok1);double(ok2)];
T.diag(6)=max(svd(L*Ac/L)); % <1 is the verified Lyapunov contraction.
if ~ok1 && ~ok2, T.diag(8)=-4; return; end
% Terminal augmented state = [predicted xi_N; last planned absolute input].
rows=(M.Np-1)*M.Ny+(1:nx);
zN0=[Pred.PSI(rows,:)*zeta+Pred.PHI(rows,:)*Pred.GAMMA;zeta(nx+(1:nu))];
zNU=[Pred.THETA(rows,:);AI(end-nu+1:end,:)];
e0=(zN0-ze)./sx; eU=bsxfun(@rdivide,zNU,sx);
F=eye(na); affine=zeros(na,1);
for j=1:M.terminal_steps
    r1=(j-1)*18+(1:18); r2=(j-1)*16+(1:16);
    % Preview-following affine backup; endpoint matrices/force offsets stay
    % frozen, but known road curvature is NOT held throughout the tail.
    dj=d; target=ze; kap_j=kap_core;
    if isfield(Pred,'d_tail')
        dj=Pred.d_tail(:,j); kap_j=Pred.kap_tail(j);
        dnode=Pred.Dterminal*[Pred.kap_tail(j);Pred.GAMMA(end-1:end)];
        rj=[dnode(1:nx);0;(umin(3)+umax(3))/2;ze(nx+2)];
        target=S*(Es\(rj./rs));
    end
    feed=K*((target-ze)./sx);
    h1j=([E.Gr;E.Genv+kap_j*E.gkap]-H1*ze)./sc1;
    % Delta u/U=-K*e+feed, hence the two rate bounds shift oppositely.
    hdj=hd+[-feed;feed];
    if ok1
        G=[H1n;Hu;Hd]; H=G*F; h=[h1j;hu;hdj];
        T.A1(r1,:)=H*eU; T.b1(r1)=h-H*e0-G*affine;
    end
    if ok2
        G=[H2n;Hu;Hd]; H=G*F; h=[h2;hu;hdj];
        T.A2(r2,:)=H*eU; T.b2(r2)=h-H*e0-G*affine;
    end
    affine=Ac*affine+Bn*feed+(dj-d)./sx;
    F=Ac*F;
end
% |L e|_inf <= sqrt(alpha/na) implies e'P e <= alpha.
% The polytope need not itself be invariant; its enclosing ellipsoid is.
if ok1
    % Unit-normal rows in physically normalized state coordinates. Dividing
    % by a nearly-zero core radius would make the shared slack ill-scaled.
    % Positive row scaling preserves EXACTLY the same zero-slack set.
    Hcore=[L;-L]; scale=max(sqrt(sum(Hcore.^2,2)),1e-12);
    H=bsxfun(@rdivide,Hcore,scale)*F; r=18*M.terminal_steps+(1:2*na);
    T.A1(r,:)=H*eU; T.b1(r)=sqrt(a1/na)./scale-H*e0-(Hcore*affine)./scale;
end
if ok2
    Hcore=[L;-L]; scale=max(sqrt(sum(Hcore.^2,2)),1e-12);
    H=bsxfun(@rdivide,Hcore,scale)*F; r=16*M.terminal_steps+(1:2*na);
    T.A2(r,:)=H*eU; T.b2(r)=sqrt(a2/na)./scale-H*e0-(Hcore*affine)./scale;
end
T.diag(8)=1;
end

function [alpha,valid]=radius_(H,h,Pinv)
alpha=1e12; valid=true;
for j=1:size(H,1)
    v=max(H(j,:)*Pinv*H(j,:)',0);
    if h(j)<-1e-8
        valid=false;
    elseif v>1e-14
        alpha=min(alpha,max(h(j),0)^2/v);
    end
end
valid=valid && isfinite(alpha) && alpha>1e-14 && alpha<1e12;
if ~valid, alpha=0; end
end
