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
% tau_d->0 表示阻尼力瞬时建立（no-comp 基准）；取下限避免 1/tau_d 发散。
% 时变时滞（M.tau_d_mode=2）：由四角压缩速度幅值决定，标定自力阶跃响应
% 的 t10-90（50/100/200/400 mm/s -> 17/19/22/22 ms）。分段线性、末段饱和。
% M.tau_d_mode=1（默认）用常数 M.tau_d。
if isfield(M,'tau_d_mode') && (M.tau_d_mode==3 || M.tau_d_mode==4)
    % 3 = 四角加权聚合，4 = 主导角；两者都在 prepare_step 里算好放进 c
    if M.tau_d_mode==3, tau_d = c.tau_d_eff; else, tau_d = c.tau_d_dom; end
elseif isfield(M,'tau_d_mode') && M.tau_d_mode==2
    vmag=1000*mean(abs(c.vd));            % [mm/s]
    if vmag<=50
        tau_d=0.017;
    elseif vmag<=100
        tau_d=0.017+0.002*(vmag-50)/50;
    elseif vmag<=200
        tau_d=0.019+0.003*(vmag-100)/100;
    else
        tau_d=0.022;
    end
else
    tau_d=M.tau_d;
end
tau_d=max(tau_d,1e-4);
A=[A,B(:,3);zeros(1,6),-1/tau_d];
B(:,3)=0; B=[B;0,0,1/tau_d]; D=[D;zeros(1,3)];

% 非均匀网格：以最小步长 dtmin 做一次 expm，其余节点用矩阵幂得到精确 ZOH。
% 均匀网格（全部 dt=0.1）时 kmul 全为 1，E0 与旧版 expm(A*Ts) 完全一致。
E0 = expm([A,B,D;zeros(6,13)]*c.dtmin);

% zeta=[x;u_previous] 的增广递推，每节点矩阵不同（Aaug_j / Baug_j / Daug_j）。
X=zeros(10,3*K); x=c.zeta; prevblk=0;
Y=zeros(7*N,3*K); y=zeros(7*N,1);
for j=1:N
    Ej = E0;
    for r=2:c.kmul(j)
        Ej = Ej*E0;
    end
    Aaug=[Ej(1:7,1:7), Ej(1:7,8:10); zeros(3,7), eye(3)];
    Baug=[Ej(1:7,8:10); eye(3)];
    Daug=[Ej(1:7,11:13); zeros(3,3)];
    x = Aaug*x + Daug*[c.kap(j); c.offset];
    X = Aaug*X;
    bj = c.blk(j);
    if bj ~= prevblk
        X(:,3*(bj-1)+(1:3)) = X(:,3*(bj-1)+(1:3)) + Baug;
        prevblk = bj;
    end
    y((j-1)*7+(1:7)) = x(1:7);
    Y((j-1)*7+(1:7),:) = X(1:7,:);
end

% 预测状态=y+Y*DeltaU；绝对输入序列=Ut+AI*DeltaU。
p=struct('Y',Y,'y',y,'AI',kron(tril(ones(K)),eye(3)), ...
    'Ut',repmat(c.zeta(8:10),K,1));
end
