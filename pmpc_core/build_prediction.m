function p=build_prediction(c,P,plan) %#codegen
%BUILD_PREDICTION 线性化 + 精确 ZOH + 控制增量预测。
% x=[Vy;r;phi;p;ey;epsi;Md_actual], u=[delta;Mz;Md_command]。
% c 为本拍工作点，plan 提供固定控制时域尺寸；N=Np，K=Nc。

N=coder.const(size(c.kap,1)); 
K=coder.const(numel(plan)/3);
V=P.vehicle; 
M=P.M; 
v=c.v; 
cd=cos(c.delta); 
Cf=c.Cf;
Cr=c.Cr;

% 连续六状态：横向、横摆、侧倾角/角速度、横向/航向跟踪误差。
A=[ (Cf*cd+Cr)/(V.m*v),             (Cf*V.lf*cd-Cr*V.lr)/(V.m*v)-v,0,0,0,0; ...
    (V.lf*Cf*cd-V.lr*Cr)/(V.Iz*v),  (V.lf^2*Cf*cd+V.lr^2*Cr)/(V.Iz*v),0,0,0,0; ...
    0,0,0,1,0,0; ...
    V.ms*V.h_S2R*(Cf*cd+Cr)/(V.m*V.Ix*v), ...
    V.ms*V.h_S2R*(Cf*V.lf*cd-Cr*V.lr)/(V.m*V.Ix*v), ...
    (V.ms*V.g*V.h_S2R-V.Kt)/V.Ix,0,0,0; ...
    1,0,0,0,0,v; 0,1,0,0,0,0];

B=[-Cf*cd/V.m,0,0;-V.lf*Cf*cd/V.Iz,1/V.Iz,0;0,0,0; ...
    -V.ms*V.h_S2R*Cf*cd/(V.m*V.Ix),0,-1/V.Ix;0,0,0;0,0,0];

% 扰动向量为 [道路曲率;前轴轮胎偏置;后轴轮胎偏置]。
D=[0,cd/V.m,1/V.m;0,V.lf*cd/V.Iz,-V.lr/V.Iz;0,0,0; ...
    0,V.ms*V.h_S2R*cd/(V.m*V.Ix),V.ms*V.h_S2R/(V.m*V.Ix); ...
    0,0,0;-v,0,0];

% 增加实际 Md 状态，用 tau_d 描述力矩命令到实际力矩的一阶响应。
A=[A,B(:,3);zeros(1,6),-1/M.tau_d];
B(:,3)=0; B=[B;0,0,1/M.tau_d]; D=[D;zeros(1,3)];

% 矩阵指数同时求 Ad/Bd/Dd，输入与扰动在预测步内零阶保持。
E=expm([A,B,D;zeros(6,13)]*M.Ts);

% 增广上一拍输入，改用控制增量为决策量：zeta=[x;u_previous]。
A=[E(1:7,1:7),E(1:7,8:10);zeros(3,7),eye(3)];
B=[E(1:7,8:10);eye(3)]; D=[E(1:7,11:13);zeros(3)];

% x/y 是零增量自由响应，X/Y 是控制增量映射；控制时域后保持末端输入。
X=zeros(10,3*K); x=c.zeta;
Y=zeros(7*N,3*K); y=zeros(7*N,1);
for j=1:N
    x=A*x+D*[c.kap(j);c.offset];
    X=A*X;
    if j<=K, X(:,(j-1)*3+(1:3))=X(:,(j-1)*3+(1:3))+B; end
    y((j-1)*7+(1:7))=x(1:7);
    Y((j-1)*7+(1:7),:)=X(1:7,:);
end

% 预测状态=y+Y*DeltaU；绝对输入序列=Ut+AI*DeltaU。
p=struct('Y',Y,'y',y,'AI',kron(tril(ones(K)),eye(3)), ...
    'Ut',repmat(c.zeta(8:10),K,1));
end
