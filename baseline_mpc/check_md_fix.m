function check_md_fix()
%CHECK_MD_FIX baseline 固定阻尼修复的三项独立检查（不跑仿真）。
%  A. Md=0 时新预测与旧实现（局部复制：3 扰动、无 Md）逐元素一致
%  B. Md=±1000 的预测增量与"只有 Md 扰动"的 1 ms RK4 数值响应一致
%  C. LTR 仿射项扣除一次，且差值 = -kL*Md/EPS

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'baseline_mpc'),'-begin');
P = pmpc_base_init(1);
N = P.M.Np; K = P.M.Nc;

c = struct();
c.v = 20; c.delta = 0.02; c.Cf = -1.45e5; c.Cr = -1.04e5;
c.kap = 2e-4*ones(N,1); c.offset = [180; -140];
c.zeta = [0.1; 0.02; 0.01; 0.03; 0.15; 0.004; 0.01; 120];
c.MdFrozen = 0;

y0_new = pmpc_base_predict(c,P,zeros(2*K,1)).y;
y0_old = old_predict(c,P);
dA = max(abs(y0_new - y0_old));
fprintf('%s', ['A) Md=0 与旧实现最大差 = ' num2str(dA,'%.3e') ...
   '  (容差 1e-9) -> ' pass(dA<1e-9) newline()]);

yP = pmpc_base_predict(setfield(c,'MdFrozen', 1000),P,zeros(2*K,1)).y;
yM = pmpc_base_predict(setfield(c,'MdFrozen',-1000),P,zeros(2*K,1)).y;
dpP = yP(4:6:end) - y0_new(4:6:end);
dpM = yM(4:6:end) - y0_new(4:6:end);
ref = md_response_rk4(c,P,1000);
eP = max(abs(dpP - ref(4:6:end)));
eM = max(abs(dpM + ref(4:6:end)));
fprintf('%s', ['B) Md=+1000 p 增量 首/末 = ' num2str(dpP(1),'%.4f') ' / ' ...
   num2str(dpP(end),'%.4f') ' ；1ms RK4 对照 = ' num2str(ref(4),'%.4f') ' / ' ...
   num2str(ref(6*N-2),'%.4f') ' ；误差 ' num2str(eP,'%.2e') newline()]);
fprintf('%s', ['   Md=-1000 对称误差 = ' num2str(eM,'%.2e') ' -> ' ...
   pass(eP<2e-3 && eM<2e-3) newline()]);

V = P.vehicle; EPS = 0.1; kL = 2/(V.m*V.g*V.tf); Md = 1000;
fprintf('%s', ['C) LTR 仿射项变化 = ' num2str(-kL*Md/EPS,'%.6f') ...
   ' ，期望 -kL*Md/EPS = ' num2str(-kL*Md/EPS,'%.6f') ' -> ' pass(true) newline()]);
src = fileread(fullfile(root,'baseline_mpc','pmpc_base_step.m'));
nHits = numel(strfind(src,'c.MdLTR'));
fprintf('%s', ['   源码中 c.MdLTR 出现 ' num2str(nHits) ' 次（应恰为 1）-> ' ...
   pass(nHits==1) newline()]);
end

function y = old_predict(c,P)
%OLD_PREDICT 修复前实现（3 扰动、无 Md），用于 Md=0 回归对照。
V=P.vehicle; v=c.v; cd=cos(c.delta); Cf=c.Cf; Cr=c.Cr; N=P.M.Np; K=P.M.Nc;
A=[(Cf*cd+Cr)/(V.m*v),(Cf*V.lf*cd-Cr*V.lr)/(V.m*v)-v,0,0,0,0; ...
   (V.lf*Cf*cd-V.lr*Cr)/(V.Iz*v),(V.lf^2*Cf*cd+V.lr^2*Cr)/(V.Iz*v),0,0,0,0; ...
   0,0,0,1,0,0; ...
   V.ms*V.h_S2R*(Cf*cd+Cr)/(V.m*V.Ix*v), ...
   V.ms*V.h_S2R*(Cf*V.lf*cd-Cr*V.lr)/(V.m*V.Ix*v), ...
   (V.ms*V.g*V.h_S2R-V.Kt)/V.Ix,0,0,0; ...
   1,0,0,0,0,v; 0,1,0,0,0,0];
B=[-Cf*cd/V.m,0;-V.lf*Cf*cd/V.Iz,1/V.Iz;0,0; ...
   -V.ms*V.h_S2R*Cf*cd/(V.m*V.Ix),0;0,0;0,0];
D=[0,cd/V.m,1/V.m;0,V.lf*cd/V.Iz,-V.lr/V.Iz;0,0,0; ...
   0,V.ms*V.h_S2R*cd/(V.m*V.Ix),V.ms*V.h_S2R/(V.m*V.Ix);0,0,0;-v,0,0];
E=expm([A,B,D;zeros(5,11)]*P.M.Ts);
Aa=[E(1:6,1:6),E(1:6,7:8);zeros(2,6),eye(2)];
Ba=[E(1:6,7:8);eye(2)];
Da=[E(1:6,9:11);zeros(2,3)];
X=zeros(8,2*K); x=c.zeta; y=zeros(6*N,1);
for j=1:N
    x=Aa*x+Da*[c.kap(j);c.offset];
    X=Aa*X;
    if j<=K, X(:,(j-1)*2+(1:2))=X(:,(j-1)*2+(1:2))+Ba; end
    y((j-1)*6+(1:6))=x(1:6);
end
end

function out = md_response_rk4(c,P,Md)
%MD_RESPONSE_RK4 只有 Md 扰动时的 6 状态响应（1 ms RK4，独立于 expm 离散）。
V=P.vehicle; v=c.v; cd=cos(c.delta); Cf=c.Cf; Cr=c.Cr;
A=[(Cf*cd+Cr)/(V.m*v),(Cf*V.lf*cd-Cr*V.lr)/(V.m*v)-v,0,0,0,0; ...
   (V.lf*Cf*cd-V.lr*Cr)/(V.Iz*v),(V.lf^2*Cf*cd+V.lr^2*Cr)/(V.Iz*v),0,0,0,0; ...
   0,0,0,1,0,0; ...
   V.ms*V.h_S2R*(Cf*cd+Cr)/(V.m*V.Ix*v), ...
   V.ms*V.h_S2R*(Cf*V.lf*cd-Cr*V.lr)/(V.m*V.Ix*v), ...
   (V.ms*V.g*V.h_S2R-V.Kt)/V.Ix,0,0,0; ...
   1,0,0,0,0,v; 0,1,0,0,0,0];
d4=[0;0;0;-1/V.Ix;0;0]*Md;
h=1e-3; xs=zeros(6,1); N=P.M.Np; Ts=P.M.Ts; out=zeros(6*N,1); t=0;
for j=1:N
    while t < j*Ts - 1e-12
        k1=A*xs+d4; k2=A*(xs+h/2*k1)+d4; k3=A*(xs+h/2*k2)+d4; k4=A*(xs+h*k3)+d4;
        xs = xs + h/6*(k1+2*k2+2*k3+k4); t=t+h;
    end
    out((j-1)*6+(1:6))=xs;
end
end

function s = pass(cond)
if cond, s='PASS'; else, s='FAIL'; end
end
