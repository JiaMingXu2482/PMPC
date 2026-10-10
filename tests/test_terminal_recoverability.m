function tests=test_terminal_recoverability
tests=functiontests(localfunctions);
end
function setupOnce(~)
startup_pmpc();
end
function testFrozenModelInvariantCore(t)
[M,C,E,P,AI,z,L,lo,hi]=fixture_();
[T,p]=func_TerminalRecoverability(M,C,E,P,AI,z,L,lo,hi);
assertEqual(t,T.diag(2:3),[1;1],sprintf('Construction diagnostics: %s',mat2str(T.diag')));
verifyLessThan(t,T.diag(6),1);
verifySize(t,T.A1,[74,15]); verifySize(t,T.A2,[68,15]);
verifyGreaterThan(t,min(T.b1),-1e-10);
verifyGreaterThan(t,norm(T.A1(:,2:3:end),'fro'),1e-8);
rng(7); U=chol(p.P);
for j=1:80
    v=randn(10,1); v=v/norm(v)*sqrt(min(p.alpha));
    e=U\v;
    verifyLessThanOrEqual(t,(p.Acl*e)'*p.P*(p.Acl*e),e'*p.P*e+1e-9);
    x=p.center+p.scale.*e;
    verifyLessThanOrEqual(t,max(E.Henv*x(1:7)-E.Genv),1e-7);
    verifyLessThanOrEqual(t,max(E.Hsh*x(1:7)-E.Gsh),1e-7);
    verifyLessThanOrEqual(t,max(abs(x(8:10))-hi),1e-7);
    du=-diag(p.scale(8:10))*p.K*e;
    verifyLessThanOrEqual(t,max(abs(du)-[L.dFyfmax;L.dMFxmax;L.dMdmax]),1e-7);
end
end
function testInvalidCenterDoesNotClaimCertificate(t)
[M,C,E,P,AI,z,L,lo,hi]=fixture_();
E.Genv=-ones(4,1);
T=func_TerminalRecoverability(M,C,E,P,AI,z,L,lo,hi);
verifyEqual(t,T.diag(2),0);
verifyEqual(t,T.A1,zeros(size(T.A1)));
end
function testPredictsRiskWhileCurrentlyInside(t)
[M,C,E,P,AI,z,L,lo,hi]=fixture_();
z(1)=.05; z(5)=.2;
verifyLessThan(t,max(E.Henv*z(1:7)-E.Genv),0);
Y=reshape(P.PSI*z+P.PHI*P.GAMMA,7,[]);
verifyLessThan(t,max(max(E.Henv*Y-repmat(E.Genv,1,M.Np))),0);
verifyLessThan(t,max(max(E.Hsh*Y-repmat(E.Gsh,1,M.Np))),0);
verifyLessThan(t,max(max(E.Hr*Y-repmat(E.Gr,1,M.Np))),0);
T=func_TerminalRecoverability(M,C,E,P,AI,z,L,lo,hi);
verifyEqual(t,T.diag(2),1);
verifyLessThan(t,min(T.b1),0);
end
function testDiagnosticsAndDisabledDimensions(t)
verifyEqual(t,numel(func_PriorityDiagnostics((1:44)')),20);
verifyEqual(t,numel(func_PriorityDiagnostics((1:36)')),12);
M=struct('terminal_on',false);
[n1,n2]=func_TerminalRowCounts(M);
verifyEqual(t,[n1 n2],[0 0]);
end
function testSmallCoreDoesNotBlowUpRowCoefficients(t)
[M,C,E,P,AI,z,L,lo,hi]=fixture_();
T=func_TerminalRecoverability(M,C,E,P,AI,z,L,lo,hi);
L.dMFxmax=.01;
Tiny=func_TerminalRecoverability(M,C,E,P,AI,z,L,lo,hi);
assertEqual(t,Tiny.diag(2:3),[1;1]);
verifyTrue(t,all(isfinite(Tiny.A1(:))) && all(isfinite(Tiny.A2(:))));
% Rate-aware K now changes with the limit; do not require identical normals.
verifyLessThan(t,norm(Tiny.A1,'fro'),100*max(1,norm(T.A1,'fro')));
end
function testCurvaturePreviewCondensationMatchesRollout(t)
[M,C,E,P,AI,z,L,lo,hi]=fixture_();
P.Dterminal=zeros(10,3); P.Dterminal(6,1)=-2;
P.kap_tail=[.01;-.01;.005;0];
P.d_tail=P.Dterminal*[.005 -.002 .003;zeros(2,3)];
[T,p]=func_TerminalRecoverabilityPreview(M,C,E,P,AI,z,L,lo,hi);
assertEqual(t,T.diag(2:3),[1;1]);
duPlan=repmat([.002;10;2],5,1);
xx=[P.PSI(end-6:end,:)*z+P.THETA(end-6:end,:)*duPlan;AI(end-2:end,:)*duPlan];
us=p.scale(8:10); limits=[L.dFyfmax;L.dMFxmax;L.dMdmax];
Eq=zeros(10); Eq(1:7,:)=[eye(7)-P.Aterminal(1:7,1:7),-P.Aterminal(1:7,8:10)];
Eq(8,5)=1; Eq(9,7)=1; Eq(10,9)=1;
for j=1:3
    dn=P.Dterminal*[P.kap_tail(j);0;0]; target=Eq\[dn(1:7);0;0;p.center(9)];
    du=-us.*(p.K*((xx-target)./p.scale));
    v1=[(E.Hr*xx(1:7)+E.Or*xx(8:10)-E.Gr)./E.sc_r; ...
        (E.Henv*xx(1:7)-E.Genv-P.kap_tail(j)*E.gkap)./E.sc_env; ...
        (xx(8:10)-hi)./us;(lo-xx(8:10))./us; ...
        (du-limits)./us;(-du-limits)./us];
    rows=(j-1)*18+(1:18);
    verifyEqual(t,T.A1(rows,:)*duPlan-T.b1(rows),v1,'AbsTol',1e-9);
    xx=P.Aterminal*xx+P.Bterminal*du+P.d_tail(:,j);
end
Ch=chol(p.P); Hc=[Ch;-Ch]; sc=sqrt(sum(Hc.^2,2));
direct=(Hc*((xx-p.center)./p.scale)-sqrt(p.alpha(1)/10))./sc;
verifyEqual(t,T.A1(end-19:end,:)*duPlan-T.b1(end-19:end),direct,'AbsTol',1e-9);
end
function testNonzeroSteadyDbRecoversAdmissibleCenter(t)
[M,C,E,P,AI,z,L,lo,hi]=fixture_();
P.dterminal(6)=-.02; hi(1)=.05; lo(1)=-.05;
[T,p]=func_TerminalRecoverabilityPreview(M,C,E,P,AI,z,L,lo,hi);
assertEqual(t,T.diag(2:3),[1;1]);
verifyGreaterThan(t,abs(p.center(9)),10);
verifyLessThanOrEqual(t,max(p.center(8:10)-hi),1e-7);
verifyLessThanOrEqual(t,max(lo-p.center(8:10)),1e-7);
verifyLessThan(t,norm(P.Aterminal*p.center+P.dterminal-p.center,inf),1e-8);
end
function testPhysicalTailMatchesDirectPrediction(t)
[M,~,E,P,AI,z,~,~,~]=fixture_();
M.terminal_mode=2;
P.kap_tail=[.01;.02;-.01;0];
P.d_tail=zeros(10,3); P.d_tail(6,:)=[-.02 -.01 0];
z(5)=.2; z(1)=.4; z(9)=30;
u=repmat([.003;20;5],5,1);
T=func_FutureRiskTail(M,E,P,AI,z);
verifySize(t,T.A1,[18,15]); verifySize(t,T.A2,[12,15]);
x=[P.PSI(end-6:end,:)*z+P.PHI(end-6:end,:)*P.GAMMA+P.THETA(end-6:end,:)*u;z(8:10)+AI(end-2:end,:)*u];
for j=1:3
    x=P.Aterminal*x+P.d_tail(:,j);
    v1=[(E.Hr*x(1:7)+E.Or*x(8:10)-E.Gr)./E.sc_r; ...
        (E.Henv*x(1:7)-E.Genv-P.kap_tail(j+1)*E.gkap)./E.sc_env];
    verifyEqual(t,T.A1((j-1)*6+(1:6),:)*u-T.b1((j-1)*6+(1:6)),v1,'AbsTol',1e-9);
    verifyEqual(t,T.A2((j-1)*4+(1:4),:)*u-T.b2((j-1)*4+(1:4)), ...
        (E.Hsh*x(1:7)-E.Gsh)./E.sc_sh,'AbsTol',1e-9);
end
verifyGreaterThan(t,norm(T.A1(:,2:3:end),'fro'),0);
verifyEqual(t,T.diag(8),2);
end
function testPhysicalTailDoesNotSwitchOffAtViolation(t)
[M,~,E,P,AI,z,~,~,~]=fixture_(); M.terminal_mode=2;
P.kap_tail=zeros(4,1); P.d_tail=zeros(10,3);
T=func_FutureRiskTail(M,E,P,AI,z);
z(5)=2; U=func_FutureRiskTail(M,E,P,AI,z);
verifyEqual(t,T.A1,U.A1);
verifyEqual(t,U.diag(2:3),[1;1]);
verifyLessThan(t,min(U.b1),0);
end
function [M,C,E,P,AI,z,L,lo,hi]=fixture_()
M=struct('Nx',7,'Ny',7,'Nu',3,'Np',10,'Nc',5,'ControllerVariant',3, ...
    'abl',0,'terminal_on',true,'terminal_steps',3);
C=struct('Vymax',3,'phimax',.15,'dphimax',.5,'emax',.55,'epsimax',.14);
A=diag([.9,.8,.9,.7,1,1,.1]); A(3,4)=.1; A(5,1)=.1; A(5,6)=2;
A(6,2)=.1; A(4,7)=-1e-4; A(2,1)=.02;
B=zeros(7,3); B(1,1)=.5; B(2,2)=.001; B(7,3)=.9;
Aa=[A B;zeros(3,7) eye(3)]; Ba=[B;eye(3)];
S=struct('A_aug',repmat(Aa,1,1,10),'B_aug',repmat(Ba,1,1,10), ...
    'D_aug',zeros(10,3,10),'C_aug',[eye(7) zeros(7,3)]);
[ps,th,ga,ph]=func_SystemFurture(M,S,zeros(10,1),zeros(2,1));
P=struct('PSI',ps,'THETA',th,'GAMMA',ga,'PHI',ph, ...
    'Aterminal',Aa,'Bterminal',Ba,'dterminal',zeros(10,1));
E=struct('Henv',[0 0 0 0 1 2 0;0 0 0 0 1 -2 0; ...
    0 0 0 0 -1 -2 0;0 0 0 0 -1 2 0], 'Genv',.55*ones(4,1), ...
    'gkap',zeros(4,1),'sc_env',.55*ones(4,1), ...
    'Hsh',[0 1 0 0 0 0 0;0 -1 0 0 0 0 0;1/20 -2/20 0 0 0 0 0; ...
    -1/20 2/20 0 0 0 0 0],'Gsh',[.5;.5;.12;.12],'sc_sh',[.5;.5;.12;.12], ...
    'Hr',[0 0 2 0 0 0 0;0 0 -2 0 0 0 0],'Or',zeros(2,3), ...
    'Gr',[.8;.8],'sc_r',[.8;.8]);
AI=kron(tril(ones(5)),eye(3)); z=zeros(10,1);
L=struct('kap',zeros(10,1),'dFyfmax',.03,'dMFxmax',200,'dMdmax',100);
hi=[.3;2000;1000]; lo=-hi;
end
