function T=func_FutureRiskTail(M,E,Pred,AI,zeta) %#codegen
% Physical state constraints beyond Np, with the last planned input HELD.
% Affine in the existing DeltaU; no equilibrium, Riccati/core switch, or new
% decision variables. A finite frozen-model risk preview, NOT an invariant set.
[n1,n2]=func_TerminalRowCounts(M);
nx=M.Nx; nu=M.Nu; nd=nu*M.Nc;
T=struct('A1',zeros(n1,nd),'b1',zeros(n1,1), ...
    'A2',zeros(n2,nd),'b2',zeros(n2,1),'diag',zeros(8,1));
% diag 2:3 denote active row groups, not invariant-core validity. Status 2
% distinguishes this path; contraction/equilibrium diagnostics are inapplicable.
T.diag=[1;1;1;NaN;NaN;NaN;NaN;2];
rows=(M.Np-1)*M.Ny+(1:nx);
x0=[Pred.PSI(rows,:)*zeta+Pred.PHI(rows,:)*Pred.GAMMA;zeta(nx+(1:nu))];
X=[Pred.THETA(rows,:);AI(end-nu+1:end,:)];
H1=[E.Hr E.Or;E.Henv zeros(4,nu)];
H2=[E.Hsh zeros(4,nu)];
sc1=[E.sc_r;E.sc_env];
for j=1:M.terminal_steps
    % d_tail includes the previewed road curvature for this future interval.
    x0=Pred.Aterminal*x0+Pred.d_tail(:,j);
    X=Pred.Aterminal*X;
    g1=[E.Gr;E.Genv+Pred.kap_tail(j+1)*E.gkap];
    r1=(j-1)*6+(1:6); r2=(j-1)*4+(1:4);
    T.A1(r1,:)=bsxfun(@rdivide,H1*X,sc1);
    T.b1(r1)=(g1-H1*x0)./sc1;
    T.A2(r2,:)=bsxfun(@rdivide,H2*X,E.sc_sh);
    T.b2(r2)=(E.Gsh-H2*x0)./E.sc_sh;
end
end
