function [SW,Fdes,Tb,stateNext,Xlocal,Ylocal,logVec] = pmpc_step(meas,Reftraj,params,Flo,Fhi,state,prediction,delta_d) %#codegen
%PMPC_STEP 100 Hz 控制入口。Reftraj/params 来自控制块的 Parameter 数据。
% 新分段接法增加prediction输入，直接接本拍pmpc_preview输出；不得加延迟。
% 保留6参调用供旧接线/离线使用；7参调用复用预测矩阵。stateNext由外部Unit Delay保存。
% delta_d 为驾驶员前轮转角，u(1)=Delta delta 为 AFS 附加角；总转角
% delta=delta_d+u(1)。默认 delta_d=0，兼容 J-turn/DLC 的7参调用。
% Flo/Fhi 为4x1或4x(Np+1)，行顺序 [L1;L2;R1;R2]，CarSim 力符号。
% 输出 Fdes/Tb：[L1;L2;R1;R2]，对应当前 SLX 的执行器端口。
% SW [deg]；Tb 为非负制动力矩 [Nm]；Fdes [N]。
% Xlocal/Ylocal 为全局坐标系下的局部参考路径，尺寸为 (Np+1)x1，单位 m。
% state 保存上一拍 AFS/Mz/Md 命令、备用增量序列和三个 QP 的活动集；先给输出赋保持值。
% params/Reftraj/state 与测量、力边界均使用 double；活动集保持 logical。
% logVec 为25x1诊断日志，不参与控制计算。
% [ey,epsi,phi,p,LTR,Md_actual,Md_cmd,Md_min,Md_max,
%  Fd_L1,Fd_L2,Fd_R1,Fd_R2,Fdes_L1,Fdes_L2,Fdes_R1,Fdes_R2,
%  Mz_cmd,SW,Vx,r,deltaMd,deltaMd_max,QP_OK,candidate_used]。

if nargin<8, delta_d=0; end

% 初始化
stateNext=state;
logVec=nan(25,1);
logVec(24:25)=-1; % -1表示本拍尚未执行主QP

Np=coder.const((numel(state.active_main)-4*numel(state.plan)-10)/12);
% 无效测量或参考路径时输出 NaN，避免 XY Graph 保留上一拍的旧路径。
Xlocal=nan(Np+1,1);
Ylocal=nan(Np+1,1);
SW=rad2deg(min(max(params.vehicle.isw*(delta_d+state.u_prev(1)),-deg2rad(720)),deg2rad(720)));
Tb=state.brake_prev;
% 两路必须同尺寸；各自用编译期常量索引，避免旧4x1特化时引用第2列。
assert(size(Flo,1)==4 && isequal(size(Flo),size(Fhi)), ...
    'Flo/Fhi must have identical sizes: 4x1 or 4x(Np+1).');
assert(size(Flo,2)==1 || size(Flo,2)==Np+1);
colLo=coder.const(min(2,size(Flo,2)));
colHi=coder.const(min(2,size(Fhi,2)));
lower = min(Flo(:,colLo), Fhi(:,colHi));
upper = max(Flo(:,colLo), Fhi(:,colHi));
Fdes  = min(max(state.damper_prev, lower), upper);

% 先记录无需prepare_step即可获得的测量量，便于诊断提前返回的情况。
logVec(3)=meas.phi;
logVec(4)=meas.p;
logVec(10:13)=meas.Fd(:);
logVec(14:17)=Fdes(:);
logVec(18)=state.u_prev(2);
logVec(19)=SW;
logVec(20)=meas.Vx;
logVec(21)=meas.r;
logVec(7)=state.u_prev(3);
FzLog=max(meas.Fz(:),0);
FzTotal=sum(FzLog);
if numel(FzLog)==4 && all(isfinite(FzLog)) && FzTotal>1e-6
    % 正值表示左侧垂向载荷大于右侧。
    logVec(5)=(FzLog(1)+FzLog(2)-FzLog(3)-FzLog(4))/FzTotal;
end
if numel(meas.Fd)==4 && all(isfinite(meas.Fd(:)))
    BdLog=.5*[-params.vehicle.ldf;-params.vehicle.ldr; ...
               params.vehicle.ldf; params.vehicle.ldr];
    logVec(6)=BdLog'*meas.Fd(:);
end

% 检查所有测量数据和悬架力边界中，是否存在 NaN 或 Inf。
if meas.Vx<=0 || any(~isfinite([meas.X;meas.Y;meas.Vx;meas.Vy; ...
                    meas.yaw;meas.r;meas.phi;meas.p;meas.delta_front(:); ...
                    meas.steering_wheel;meas.Fx(:);meas.Fy(:);meas.Fz(:); ...
                    meas.Fd(:);meas.Td(:);Flo(:);Fhi(:)]))
    Fdes=state.damper_prev;
    return
end
% 1. 更新局部参考、轮胎工作点与执行器边界。
[c,index,valid]=prepare_step(meas,Reftraj,params,Flo,Fhi,state,delta_d);
if ~valid, return; end
% 输出当前时刻及 Np 个预测节点的局部参考路径坐标。
Xlocal=c.Xlocal;
Ylocal=c.Ylocal;
stateNext.path_index=index;

% 当前控制工作点及首个预测步的侧倾力矩边界。
logVec(1)=c.zeta(5);
logVec(2)=c.zeta(6);
logVec(6)=c.zeta(7);
logVec(8)=c.MdloPlan(1);
logVec(9)=c.MdhiPlan(1);
logVec(23)=c.duMax(3);
% 2. 新接法复用本拍预测关系；只更新参考/约束工作点，不重复矩阵指数。
% 两块必须使用同一拍meas、同一Reftraj/params和同一个旧state。
if nargin>=7
    if ~prediction.valid, return; end
    p=struct('Y',prediction.Y,'y',prediction.y, ...
        'AI',prediction.AI,'Ut',prediction.Ut);
else
    p=build_prediction(c,params,state.plan);
end
% 3. 构造目标函数和约束，并生成符合条件时可回退使用的候选计划。
[q,candidate,useCandidate,mdFeasible]=build_qp(c,p,params,state);
% 4. 先使用逐步预瞄界；已知Md区间不相容时直接跳过该QP。
z=zeros(size(q.f)); ok=false;
if mdFeasible
    [z,ok,stateNext.active_main]=solve_qp(q,state.active_main);
end
if ~ok && ~useCandidate
    % 远期NN条件边界是近似；不能让其不可行性同时锁死转向和DB。
    % 退回首步边界在全时域保持的QP，首步预瞄和四角分配硬界均不变。
    % 不放松道路/LTR/稳定性定义，不改变Q/R/目标优先级计算规则。
    local=c;
    local.MdloPlan(:)=c.MdloPlan(1);
    local.MdhiPlan(:)=c.MdhiPlan(1);
    [q,candidate,useCandidate,~]=build_qp(local,p,params,state);
    [z,ok,stateNext.active_main]=solve_qp(q,false(size(state.active_main)));
end
% 5. 本拍执行第一项增量；仅将后续候选计划按实际执行周期对齐。
% 求解与候选均不可用时保持命令，清零旧计划，避免重复使用过期增量。
du=zeros(3,1); nd=coder.const(numel(state.plan)); plan=zeros(nd,1);
candidateUsed=false;
if ok || useCandidate
    if ok
        plan=z(1:nd);
    else
        plan=candidate;
        candidateUsed=true;
    end
    du=plan(1:3);
end
logVec(22)=du(3);
logVec(24)=double(ok);
logVec(25)=double(candidateUsed);
stateNext.u_prev=c.zeta(8:10)+du;
% 旧绝对输入为分段常值，平移Ts_exec后按Ts区间取平均，末端保持。
% 对0<Ts_exec<=Ts，Unew_j=(1-a)*U_j+a*U_{j+1}，a=Ts_exec/Ts。
% 先扣除已执行的第一项，再转换为相对新u_prev的增量，固定尺寸不变。
% 这只对齐下一拍候选，不滤波本拍命令，也不改变预测模型的ZOH离散化。
plan(1:3)=0;
a=params.M.Ts_exec/params.M.Ts;
stateNext.plan=(1-a)*plan+a*[plan(4:end);zeros(3,1)];
% 6. 将前轮转角、横摆力矩和侧倾力矩分配为执行器命令。
[SW,Fdes,Tb,stateNext]=allocate_actuators(stateNext.u_prev,c,params,stateNext);

% 最终下发命令及执行器分配结果。
logVec(7)=stateNext.u_prev(3);
logVec(14:17)=Fdes(:);
logVec(18)=stateNext.u_prev(2);
logVec(19)=SW;
end
%% 执行器分配
function [SW,Fdes,Tb,s]=allocate_actuators(u,c,P,s)
% 内部和输出四角顺序均为 [L1;L2;R1;R2]。
% u(1) 是 AFS 附加角 Delta delta；本拍总前轮转角取 c.delta_d+u(1)。
V=P.vehicle;
delta_cmd=c.delta_d+u(1);
SW=rad2deg(min(max(V.isw*delta_cmd,-deg2rad(720)),deg2rad(720)));
% 制动映射 Mz=B*T，四列为 L1/L2/R1/R2；T 是非负轮端制动力矩。
B=[V.tf/2*cos(delta_cmd)-V.lf*sin(delta_cmd),V.tr/2, ...
   -V.tf/2*cos(delta_cmd)-V.lf*sin(delta_cmd),-V.tr/2]/V.rt;
% G 惩罚轮胎附着利用，kd 惩罚相对上一拍制动力矩的变化。
G=diag((1./(V.rt*V.mu*max(c.Fz,1))).^2);
previous=min(max(s.brake_prev,0),c.brakeMax);
kd=0.003*(max(V.mu*sum(c.Fz)*V.tf/4,1)/ ...
    max(V.rt*V.mu*sum(c.Fz)/4,1))^2;
% 横摆需求接近零时逐步释放制动，跳过制动分配 QP。
if abs(u(2))<=1
    s.brake_prev=max(previous-2e4*P.M.Ts_exec,0);
    s.active_brake=false(8,1);
else
    q=struct('H',2*(B'*B+.1*G+kd*eye(4)), ...
        'f',-2*(B'*u(2)+kd*previous), ...
        'A',zeros(0,4),'b',zeros(0,1),'lb',zeros(4,1),'ub',c.brakeMax);
    [T,ok,s.active_brake]=solve_qp(q,s.active_brake);
    if ok, s.brake_prev=T; else, s.brake_prev=zeros(4,1); end
end
% 悬架映射 Md=B*F：左侧为负、右侧为正，前后使用各自力臂。
B=.5*[-V.ldf,-V.ldr,V.ldf,V.ldr];
previous=min(max(s.damper_prev,c.Flo),c.Fhi);
% 力区间越窄，该角力幅值惩罚越大；同时惩罚相对上一拍力的变化。
span=max(c.Fhi-c.Flo,1); w=(mean(span)./span).^2; w=w/mean(w);
q=struct('H',2*(B'*B+.005*diag(w)+.05*eye(4)), ...
    'f',-2*(B'*u(3)+.05*previous), ...
    'A',zeros(0,4),'b',zeros(0,1),'lb',c.Flo,'ub',c.Fhi);
% 悬架分配失败时保留已裁剪到当前上下界内的上一拍力。
[F,ok,s.active_damper]=solve_qp(q,s.active_damper);
if ok, s.damper_prev=F; else, s.damper_prev=previous; end
Tb=s.brake_prev;
Fdes=s.damper_prev;
end

%% QP 目标与约束组装
function [q,candidate,useCandidate,mdFeasible]=build_qp(c,p,P,s) %#codegen
%BUILD_QP z=[DeltaU;s_r;s_alpha;s_LTR;s_env;gamma_DB]，A*z<=b。
% 4 个松弛分别对应横摆、后轴侧偏、LTR、道路；gamma 只缩放 DB 边界。
% 采用固定预测矩阵的尺寸，不用可调参数值分配求解器数组。
N=coder.const(size(p.Y,1)/7); K=coder.const(size(p.Y,2)/3);
nd=3*K; n=nd+5;
% 固定参考尺度（SI，与工况无关），三种控制器共用。
VY_REF=1.0; R_REF=0.2; PHI_REF=deg2rad(2); P_REF=deg2rad(10);
EY_REF=0.3; EPSI_REF=deg2rad(1);
DFYF_REF=deg2rad(1); MFX_REF=500; MD_REF=500;
EPS_R_REF=deg2rad(1); EPS_A_REF=deg2rad(1);
EPS_LTR_REF=0.1; EPS_E_REF=0.1;
% 状态顺序 Vy/r/phi/p/ey/epsi/Md_actual；控制 delta/Mz/Md。
X_REF=[VY_REF;R_REF;PHI_REF;P_REF;EY_REF;EPSI_REF;1];
Q=diag(P.M.Q_norm./X_REF.^2);
Q=kron(eye(N),Q);
% 第一项是本拍下发的增量（10 ms）；其后是预测节点间增量（100 ms）。
% 同一组物理变化率分别按各自时间间隔换算，并统一用于约束与候选计划。
duLimit=repmat(c.duMax*(P.M.Ts/P.M.Ts_exec),K,1);
duLimit(1:3)=c.duMax;
% 力界随速度移动时，上一Md命令可能已在首步区间外。
% 只增加回到首步区间所必需的Md增量额度，不放宽力界或转向/DB速率。
% R仍使用原c.duMax归一化，下面的修正不减弱平滑惩罚。
duLimit(3)=max(duLimit(3),max([0; ...
    c.MdloPlan(1)-c.zeta(10);c.zeta(10)-c.MdhiPlan(1)]));
% 标量区间传播：Nc以后同一保持命令必须位于所有尾部区间的交集。
% 同时核对每个Md增量限制，避免把结构上不可行的边界交给QP反复求解。
mdFeasible=true; reachLo=c.zeta(10); reachHi=reachLo;
for j=1:K
    lo=c.MdloPlan(j); hi=c.MdhiPlan(j);
    if j==K
        lo=max(c.MdloPlan(K:N)); hi=min(c.MdhiPlan(K:N));
    end
    reachLo=max(lo,reachLo-duLimit(3*j));
    reachHi=min(hi,reachHi+duLimit(3*j));
    if reachLo>reachHi, mdFeasible=false; end
end
% 控制增量按固定参考尺度归一化；不再使用 c.duMax 作为分母。
R=kron(eye(K),diag(P.M.R_norm./[DFYF_REF;MFX_REF;MD_REF].^2));
Wb=0;                          % 只有 Proposed 对 DB 激活量定价
if P.M.controller==3, Wb=P.M.Wb; end
rho_g=1e-4*Wb;
% 标准目标为 0.5*z'*H*z+f'*z；小对角项用于数值正定化。
H=zeros(n); f=zeros(n,1);
H(1:nd,1:nd)=2*(p.Y'*Q*p.Y+R);
H(end,end)=2*rho_g;            % gamma 的二次正则
H=.5*(H+H')+1e-8*eye(n);
e=p.y-c.yr;
f(1:nd)=2*p.Y'*Q*e; f(end)=Wb;

% 固定行次序：输入框、稳定性、侧倾、道路。
% 输入约束使用累计增量；gamma_DB 在 [0,1] 内缩放横摆力矩边界。
A=zeros(6*K+12*N,n); b=zeros(6*K+12*N,1);
for j=1:K
    for k=1:3
        row=(j-1)*3+k;
        % Md 改在末尾逐预测步约束，原位置保留 0<=1，避免重复活动行。
        if k==3
            b(row)=1; b(3*K+row)=1;
            continue
        end
        A(row,1:nd)=p.AI(row,:); A(3*K+row,1:nd)=-p.AI(row,:);
        if k==2
            A(row,end)=-c.umax(2); b(row)=-p.Ut(row);
            A(3*K+row,end)=c.umin(2); b(3*K+row)=p.Ut(row);
        else
            b(row)=c.umax(k)-p.Ut(row);
            b(3*K+row)=p.Ut(row)-c.umin(k);
        end
    end
end
% 每节点：2 行横摆 -> s_r，2 行侧偏 -> s_alpha，
% 2 行 LTR -> s_LTR，4 行道路 -> s_env；Md 保持硬约束。
for j=1:N
    rows=(j-1)*7+(1:7);
    r=6*K+(j-1)*4+(1:2);
    A(r,1:nd)=c.Hsh(1:2,:)*p.Y(rows,:)/EPS_R_REF;
    A(r,nd+1)=-1;
    b(r)=(c.Gsh(1:2)-c.Hsh(1:2,:)*p.y(rows))/EPS_R_REF;
    r=6*K+(j-1)*4+2+(1:2);
    A(r,1:nd)=c.Hsh(3:4,:)*p.Y(rows,:)/EPS_A_REF;
    A(r,nd+2)=-1;
    b(r)=(c.Gsh(3:4)-c.Hsh(3:4,:)*p.y(rows))/EPS_A_REF;
    r=6*K+4*N+(j-1)*2+(1:2);
    ur=(min(j,K)-1)*3+(1:3);
    A(r,1:nd)=(c.Hr*p.Y(rows,:)+c.Or*p.AI(ur,:))/EPS_LTR_REF;
    A(r,nd+3)=-1;
    b(r)=(c.Gr-c.Hr*p.y(rows)-c.Or*p.Ut(ur))/EPS_LTR_REF;
    r=6*K+6*N+(j-1)*4+(1:4);
    A(r,1:nd)=c.Henv*p.Y(rows,:)/EPS_E_REF;
    A(r,nd+4)=-1;
    b(r)=(c.Genv+c.kapNode(j)*c.gkap-c.Henv*p.y(rows))/EPS_E_REF;
    % j>Nc 时仍约束保持的最后一项 Md 命令，决策变量数不增加。
    row=(min(j,K)-1)*3+3;
    r=6*K+10*N+2*j-1;
    A(r,1:nd)=p.AI(row,:); b(r)=c.MdhiPlan(j)-p.Ut(row);
    A(r+1,1:nd)=-p.AI(row,:); b(r+1)=p.Ut(row)-c.MdloPlan(j);
end

% 投影已按执行周期对齐的上一拍候选计划，用预测安全余量计算目标优先级权重。
candidate=zeros(nd,1); u=c.zeta(8:10); target=u; gamma=0;
for j=1:K
    k=(j-1)*3+(1:3);
    target=target+s.plan(k);
    lower=c.umin; upper=c.umax;
    lower(3)=c.MdloPlan(j); upper(3)=c.MdhiPlan(j);
    next=min(max(target,max(lower,u-duLimit(k))),min(upper,u+duLimit(k)));
    candidate(k)=next-u; u=next;
    if u(2)>0
        gamma=max(gamma,u(2)/max(c.umax(2),1e-9));
    elseif u(2)<0
        gamma=max(gamma,u(2)/min(c.umin(2),-1e-9));
    end
end
% 候选零松弛残差：正值表示违反约束，负值表示安全余量。
gamma=min(max(gamma,0),1);
residual=A*[candidate;0;0;0;0;gamma]-b;
% margin(1) 为道路/侧倾最小余量，margin(2) 为稳定性最小余量。
margin=[-max(residual(6*K+4*N+1:6*K+10*N));-max(residual(6*K+(1:4*N)))];
F=max(.5*candidate'*H(1:nd,1:nd)*candidate+f(1:nd)'*candidate+e'*Q*e,0) ...
    +Wb*gamma+rho_g*gamma^2;
% 松弛权重：MPC 固定；ZENG 固定但稳定性松弛乘 rho；Proposed 按余量在线。
if P.M.controller==3
    w2=P.M.w2_max;
    if margin(2)>0
        w2=min(max(P.M.w2_gain*F/margin(2),1),P.M.w2_max);
    end
    w1=P.M.w1_max;
    if margin(1)>0
        slack=max(-margin(2),0);
        w1=min(max(P.M.w1_gain*(F+w2*slack+rho_g*slack^2)/margin(1),1),P.M.w1_max);
    end
    wr=w2; wa=w2; wl=w1; we=w1;
else
    wr=P.M.slack_w(1); wa=P.M.slack_w(2);
    wl=P.M.slack_w(3); we=P.M.slack_w(4);
    if P.M.controller==2
        rho_z=func_ZengRho(c.v,P.vehicle.mu,c.delta, ...
            atan2(c.zeta(1),max(c.v,0.5)),c.zeta(2),P.vehicle.g, ...
            P.M.rho_I0,P.M.rho_dI);
        % Zeng 2025: rho 乘在稳定性松弛权重上
        wr=wr*rho_z;
        wa=wa*rho_z;
    end
end
f(nd+(1:4))=[wr;wa;wl;we];
lbQ=[-duLimit;0;0;0;0;0]; ubQ=[duLimit;Inf;Inf;Inf;Inf;1];
if P.M.controller~=3
    % MPC/ZENG 不做 DB 激活量缩放，等价于 gamma=1。
    lbQ(end)=1; ubQ(end)=1;
end
q=struct('H',H,'f',f,'A',A,'b',b, ...
    'lb',lbQ,'ub',ubQ);
% 仅在输入硬约束满足且两层安全余量均为正时，允许候选回退。
useCandidate=mdFeasible && all(margin>0) && max(residual(1:6*K))<=1e-6 ...
    && max(residual(6*K+10*N+1:end))<=1e-6 ...
    && all(abs(candidate)<=duLimit+1e-6);
end

%% 主控制与分配共用的 QP 求解器
function [x,ok,activeNext]=solve_qp(q,activePrev) %#codegen
%SOLVE_QP 官方 active-set 求解器；缩放、热启动和一次冷启动重试。
% 全部 QP 数组及容差为 double；活动集为 logical。
% ok 仅用于控制失败回退，不输出诊断数据。
% 主 QP 和四轮分配 QP 分别内联，保持各自的固定尺寸。
coder.inline('always');
n=coder.const(size(q.H,1)); m=coder.const(size(q.A,1));
x=zeros(n,1); ok=false; activeNext=false(m+2*n,1);
if any(~isfinite(q.H(:))) || any(~isfinite(q.f)) || ...
        any(~isfinite(q.A(:))) || any(~isfinite(q.b)) || ...
        any(isnan(q.lb)) || any(isnan(q.ub)) || any(q.lb>q.ub)
    return
end
% 相等上下界用 base 固定变量，保留占位变量以保持代码生成尺寸。
fixed=isfinite(q.lb)&isfinite(q.ub)&(q.lb==q.ub);
base=zeros(n,1); base(fixed)=q.lb(fixed);
H=.5*(q.H+q.H');
% 缩放 x=base+map.*y，减小物理量尺度差异。
scale=1./sqrt(max(diag(H),eps*max(1,max(diag(H)))));
map=scale.*double(~fixed);
f=map.*(q.f+H*base);
H=(map*map').*H+diag(double(fixed)); H=.5*(H+H');
% 追加有限上下界；无效或固定边界用 0<=1 占位，保持固定尺寸。
A=zeros(m+2*n,n); b=ones(m+2*n,1);
A(1:m,:)=bsxfun(@times,q.A,map'); b(1:m)=q.b-q.A*base;
for j=1:n
    if ~fixed(j) && isfinite(q.lb(j))
        A(m+j,j)=-1; b(m+j)=(base(j)-q.lb(j))/scale(j);
    end
    if ~fixed(j) && isfinite(q.ub(j))
        A(m+n+j,j)=1; b(m+n+j)=(q.ub(j)-base(j))/scale(j);
    end
end
% 约束逐行归一化，后续可行性容差用于缩放后的问题。
norms=sqrt(sum(A.^2,2)); rowscale=max(norms,1e-12); rowscale(norms==0)=1;
A=bsxfun(@rdivide,A,rowscale); b=b./rowscale;
if any(~isfinite(H(:))) || any(~isfinite(f)) || ...
        any(~isfinite(A(:))) || any(~isfinite(b)), return; end
[~,notPD]=chol(H);
if notPD~=0, return; end
% 剔除占位活动约束；活动集过多或线性相关时冷启动。
active=logical(activePrev(:)); active(norms==0)=false;
if nnz(active)>n || (any(active) && rank(A(active,:))<nnz(active))
    active(:)=false;
end
options=mpcActiveSetOptions('DataType','double');
options.MaxIterations=200;
options.ConstraintTolerance=1e-6;
options.UseHessianAsInput=true;
options.IntegrityChecks=false;
% 只接受成功状态、有限解和满足缩放后约束容差的解。
[y,status,act]=mpcActiveSetSolver(H,f,A,b,zeros(0,n),zeros(0,1),active,options);
ok=status>0 && all(isfinite(y)) && max([0;A*y-b])<=1e-5;
% 热启动失败后，从空活动集重试一次。
if ~ok && any(active)
    [y,status,act]=mpcActiveSetSolver(H,f,A,b,zeros(0,n),zeros(0,1),false(m+2*n,1),options);
    ok=status>0 && all(isfinite(y)) && max([0;A*y-b])<=1e-5;
end
if ok
    % 恢复原物理单位，保存成功的活动集。
    x=base+map.*y;
    ok=all(isfinite(x));
    if ok, activeNext=act; end
end
end
