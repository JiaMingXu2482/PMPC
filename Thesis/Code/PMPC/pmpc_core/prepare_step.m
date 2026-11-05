function [c,index,valid]=prepare_step(meas,Reftraj,params,Flo,Fhi,state) %#codegen
%PREPARE_STEP 局部参考、轮胎切线、执行器边界及安全包络。
% 四角向量统一为 [L1;L2;R1;R2]，前轴 [1,3]，后轴 [2,4]。
% m 为 SI 测量，P 为参数，s 为跨拍状态，c 汇集本拍预测与约束数据。

V=params.vehicle; 
vx=max(meas.Vx,.5); 
delta=mean(meas.delta_front);
if V.k_cs>0, delta=meas.steering_wheel/V.isw; end%转向柔性
Fx = meas.Fx;Fy = meas.Fy;Fz = meas.Fz;Fd = meas.Fd;

% 按各角力臂符号，由四角力边界计算可实现的总侧倾力矩范围。
% 旧接口 4x1：当前界。新接口 4x(Np+1)：当前界、各步预瞄界。
% 当前界用于原增量归一化；首个预瞄界用于本拍分配。
Bd=.5*[-V.ldf;-V.ldr;V.ldf;V.ldr];
lo0=min(Flo(:,1),Fhi(:,1)); hi0=max(Flo(:,1),Fhi(:,1));
Mdlo0=sum(min(Bd.*lo0,Bd.*hi0)); Mdhi0=sum(max(Bd.*lo0,Bd.*hi0));
assert(size(Flo,1)==4 && isequal(size(Flo),size(Fhi)), ...
    'Flo/Fhi must have identical sizes: 4x1 or 4x(Np+1).');
colLo=coder.const(min(2,size(Flo,2)));
colHi=coder.const(min(2,size(Fhi,2)));
lo=min(Flo(:,colLo),Fhi(:,colHi)); hi=max(Flo(:,colLo),Fhi(:,colHi));
Mdlo=sum(min(Bd.*lo,Bd.*hi)); Mdhi=sum(max(Bd.*lo,Bd.*hi));

% 计算前后轴侧偏角，前后轴的横向轮胎力和等效侧偏刚度。
alpha=[(meas.Vy+V.lf*meas.r)/max(meas.Vx,1)-delta; ...
       (meas.Vy-V.lr*meas.r)/max(meas.Vx,1)];%前后轮侧偏角向量
% 前轴使用 [1,3] 载荷，后轴使用 [2,4] 载荷。
[Ff,Cf]=tire(params.tireF,alpha(1),sum(Fz([1,3])));
[Fr,Cr]=tire(params.tireR,alpha(2),sum(Fz([2,4])));

% 先根据纵向力消耗情况计算每个车轮剩余的横向附着能力，
% 再用前后轴加权剩余比例修正横向力和侧偏刚度，
% 从而体现纵横向轮胎力之间的摩擦圆耦合。
Fz=max(Fz,1); 
xi=sqrt(max(1-(Fx./(V.mu*Fz)).^2,0));% 摩擦圆剩余比例 xi
xf=sum(Fz([1,3]).*xi([1,3]))/sum(Fz([1,3]));
xr=sum(Fz([2,4]).*xi([2,4]))/sum(Fz([2,4]));
Ff=xf*Ff;           Fr=xr*Fr; 
Cf=-abs(xf*Cf);     Cr=-abs(xr*Cr);

% 轮胎切线偏置：Fy ≈ C*alpha + offset。
offset=[Ff-Cf*alpha(1);Fr-Cr*alpha(2)];
% active_main 的固定长度 = 4*numel(plan)+12*Np+6。
% 时域是结构尺寸；修改 Np/Nc 后必须重新生成 state0 和状态 Bus。
Np=coder.const((numel(state.active_main)-4*numel(state.plan)-6)/12);%等价于Np = coder.const(params.M.Np);

% 全预测时域的 Md 命令界，包括 Nc 以后末端命令保持的区间。
assert(size(Flo,1)==4 && isequal(size(Flo),size(Fhi)));
assert(size(Flo,2)==1 || size(Flo,2)==Np+1);
MdloPlan=zeros(Np,1); MdhiPlan=zeros(Np,1);
for j=1:Np
    % 预测块内部仍用4x1占位界；在该特化中不编译第2列及后续列访问。
    if coder.const(size(Flo,2)==1)
        lf=Flo(:,1);
    else
        lf=Flo(:,j+1);
    end
    if coder.const(size(Fhi,2)==1)
        hf=Fhi(:,1);
    else
        hf=Fhi(:,j+1);
    end
    l=min(lf,hf); h=max(lf,hf);
    MdloPlan(j)=sum(min(Bd.*l,Bd.*h));
    MdhiPlan(j)=sum(max(Bd.*l,Bd.*h));
end

% 局部轨迹规划
[ey,epsi,kap,kapNode,yr,index,valid,Xlocal,Ylocal]= ...
    reference(meas,Reftraj,params,state.path_index,Np);

% zeta=[Vy;r;phi;p;ey;epsi;实际Md;上一拍delta;上一拍Mz;上一拍Md命令]。
zeta=[  meas.Vy; 
        meas.r; 
        meas.phi; 
        meas.p; 
        ey; 
        epsi; 
        Bd'*Fd; % 实际力矩由测量决定，不裁剪到目标力边界 
        state.u_prev];
% 上一拍 Md 命令也保留实际下发值，避免边界变化造成虚假的输入跳变。

% 转向界由轮胎力余量与机械转角界共同决定。
flim=.95*V.mu_eff*xf*sum(Fz([1,3]));%当前前轴允许的最大横向力
deltaMax=max(min( V.delta_sw_max/V.isw, delta+max(flim-Ff,0)/max(abs(Cf),5e3)),0);
deltaMin=min(max(-V.delta_sw_max/V.isw, delta-max(flim+Ff,0)/max(abs(Cf),5e3)),0);

% 轮胎摩擦圆允许的纵向力，换算为轮端制动扭矩；
% 扣除已有的正向驱动扭矩，并保证非负。
% 左侧 [1,2] 估算正横摆能力，右侧 [3,4] 估算负横摆能力。
brakeMax=max(V.rt*sqrt(max((V.mu*Fz).^2-Fy.^2,0))-max(meas.Td(:),0),0);
if abs(delta)<.15
    mzpos=V.tf/(2*V.rt)*(brakeMax(1)+brakeMax(2));
    mzneg=-V.tf/(2*V.rt)*(brakeMax(3)+brakeMax(4));
else
    mzpos=(V.tf/2*(brakeMax(1)*cos(delta)+brakeMax(2))-V.lf*sin(delta)*brakeMax(1))/V.rt;
    mzneg=(-V.tf/2*(brakeMax(3)*cos(delta)+brakeMax(4))-V.lf*sin(delta)*brakeMax(3))/V.rt;
end
% 横摆、侧倾力矩直接用可达范围，不做额外收缩。
% 左/右侧制动能力不同，正负横摆界各自取用，并保号以确保区间有序。
mzpos=max(mzpos,0);
mzneg=min(mzneg,0);
umax=[deltaMax;mzpos;Mdhi];
umin=[deltaMin;mzneg;Mdlo];

% duMax 只表示第一拍 Ts_exec 的命令增量上限。
% 后续预测节点按 Ts 折算；这是命令变化限制，不是实际执行器响应模型。
duDelta = V.delta_sw_rate_max/V.isw*params.M.Ts_exec;
% 单侧制动打满可产生的横摆力矩，按 tau_mz 建立。
mzref = V.mu*V.m*V.g/2*(V.tf+V.tr)/4;
duMz = mzref*params.M.Ts_exec/params.M.tau_mz;
% tau_md 控制命令平滑；实际 Md 的响应仍由预测模型 tau_d 描述。
duMd=max(Mdhi0-Mdlo0,1e-6)*params.M.Ts_exec/params.M.tau_md;
duMax=[duDelta;duMz;duMd];
% 必要时放宽绝对输入界，使其与上一拍输入的增量可达区间相交。
umax(1:2)=max(umax(1:2),zeta(8:9)-duMax(1:2));
umin(1:2)=min(umin(1:2),zeta(8:9)+duMax(1:2));
% 预瞄开启时，Md 不放宽到力界之外；冲突走已有失败回退，分配仍受首步力界约束。
% 旧4x1接口或alpha=0保留原恒定Md框及相交放宽规则，便于基线对照。
if size(Flo,2)==1 || params.M.preview_alpha<=0
    umax(3)=max(umax(3),zeta(10)-duMax(3));
    umin(3)=min(umin(3),zeta(10)+duMax(3));
    MdloPlan(:)=umin(3); MdhiPlan(:)=umax(3);
end

% 统一归一化后，道路/侧倾共享 s1，横摆/后轴侧偏共享 s2。
ar=deg2rad(5); rmax=-V.CarHat*ar*(1+V.lr/V.lf)/(V.m*vx);
% 正负横摆角速度和后轴侧偏角约束。
Hsh=[0 1 0 0 0 0 0;0 -1 0 0 0 0 0; ...
    1/vx -V.lr/vx 0 0 0 0 0;-1/vx V.lr/vx 0 0 0 0 0];
Gsh=[rmax;rmax;ar;ar];
% 道路包络采用 ey+a*epsi 小角度近似，扣除半车宽及安全余量。
af=.860+V.lf; rear=3.972-V.lf;
Henv=[0 0 0 0 1 af 0;0 0 0 0 1 -rear 0; ...
      0 0 0 0 -1 -af 0;0 0 0 0 -1 rear 0];
Genv=(params.road.width/2-1.96/2-.2)*ones(4,1);
% Hr/Or/Gr 构成双侧 LTR 约束，含状态、转向输入及轮胎偏置。
ay=[(Cf*cos(delta)+Cr)/(V.m*vx),(Cf*V.lf*cos(delta)-Cr*V.lr)/(V.m*vx),zeros(1,5)];
kg=V.m*V.h_TL-V.ms*V.h_S2R; k=2/(V.m*V.g*V.tf);
Hr=k*(kg*ay+[0 0 V.Kt 0 0 0 1]);
Or=k*kg*[-Cf*cos(delta)/V.m,0,0];
Gr=[.8;.8]-k*kg*(offset(1)*cos(delta)+offset(2))/V.m*[1;-1];
c=struct('zeta',zeta,'ey',ey,'Cf',Cf,'Cr',Cr,'offset',offset,'v',vx,'delta',delta, ...
    'kap',kap,'kapNode',kapNode,'yr',yr,'Fz',Fz,'Flo',lo,'Fhi',hi, ...
    'brakeMax',brakeMax,'umin',umin,'umax',umax,'duMax',duMax, ...
    'MdloPlan',MdloPlan,'MdhiPlan',MdhiPlan, ...
    'Hsh',Hsh,'Gsh',Gsh,'Hr',[Hr;-Hr],'Gr',Gr,'Or',[Or;-Or], ...
    'Henv',Henv,'Genv',Genv,'gkap',[af^2;rear^2;-af^2;-rear^2]/2, ...
    'Xlocal',Xlocal,'Ylocal',Ylocal);
end

function [ey,epsi,kap,kn,yr,index,valid,Xlocal,Ylocal]=reference(m,W,P,previous,N)
% 保留原局部样条投影与弧长预瞄。航向差在 +/-pi 内。
V=P.vehicle; N=coder.const(N);
ey=0; epsi=0; kap=zeros(N,1); kn=zeros(N,1); yr=zeros(7*N,1);
Xlocal=zeros(N+1,1); Ylocal=zeros(N+1,1);
index=previous; valid=false;
if size(W,1)<6 || size(W,2)<7, return; end
% 将前轴参考坐标转换到质心；沿程索引用于保持路径进度。
x=m.X-V.lf*cos(m.yaw); y=m.Y-V.lf*sin(m.yaw);
start=max(1,min(previous,size(W,1)));
[~,j]=min((W(start:end,2)-x).^2+(W(start:end,3)-y).^2);
index=start+j-1;
if index>=size(W,1), return; end
[~,j]=min((W(:,2)-x).^2+(W(:,3)-y).^2);
% 邻近路径点作局部样条，求横向误差和包裹到 +/-pi 的航向误差。
a=max(1,j-7); b=min(size(W,1),a+15);
if b-a<5, a=max(1,b-5); end
t=linspace(1,b-a+1,1500);
xx=spline(1:b-a+1,W(a:b,2),t); yy=spline(1:b-a+1,W(a:b,3),t);
psi=spline(1:b-a+1,W(a:b,4),t);
[~,j]=min((xx-x).^2+(yy-y).^2);
ey=(xx(j)-x)*sin(m.yaw)-(yy(j)-y)*cos(m.yaw);
epsi=-(mod(psi(j)-m.yaw+pi,2*pi)-pi);
% 投影到相邻折线段得到弧长位置 station。
station=W(index,7); distance=Inf;
for segment=max(index-1,1):min(index,size(W,1)-1)
    dx=W(segment+1,2)-W(segment,2); dy=W(segment+1,3)-W(segment,3);
    len=dx^2+dy^2;
    if len>0
        u=((x-W(segment,2))*dx+(y-W(segment,3))*dy)/len;
        if segment>1, u=max(u,0); end
        if segment<size(W,1)-1, u=min(u,1); end
        d=(x-W(segment,2)-u*dx)^2+(y-W(segment,3)-u*dy)^2;
        if d<distance
            distance=d; station=W(segment,7)+u*(W(segment+1,7)-W(segment,7));
        end
    end
end
% 将当前投影点作为第一个点，后续 N 个点按恒速预瞄采样。
% Xlocal/Ylocal 均为全局车辆坐标系下的路径坐标，尺寸固定为 (N+1)x1。
sCurrent=min(max(station,W(1,7)),W(end,7));
Xlocal(1)=interp1(W(:,7),W(:,2),sCurrent,'linear');
Ylocal(1)=interp1(W(:,7),W(:,3),sCurrent,'linear');
% 恒速预瞄：kap 为预测扰动曲率，kn 为节点曲率；yr 按七状态逐节点堆叠。
K=-V.m/(2*V.L^2)*(V.lf/V.CarHat-V.lr/V.CafHat);
for j=1:N
    next=station+m.Vx*P.M.Ts;
    sCurrent=min(max(station,W(1,7)),W(end,7));
    sQuery=min(max(next,W(1,7)),W(end,7));
    Xlocal(j+1)=interp1(W(:,7),W(:,2),sQuery,'linear');
    Ylocal(j+1)=interp1(W(:,7),W(:,3),sQuery,'linear');
    ps0=interp1(W(:,7),W(:,4),sCurrent,'linear');
    ps1=interp1(W(:,7),W(:,4),sQuery,'linear');
    kap(j)=(ps1-ps0)/max(next-station,1e-6);
    kn(j)=interp1(W(:,7),W(:,5),sQuery,'linear');
    station=next;
    delta=V.L*kn(j)*(1+K*m.Vx^2);
    den=max(V.L*(1+K*m.Vx^2),1e-6);
    vy=(1-V.m*V.lf*m.Vx^2/(2*V.L*V.CafHat*V.lr))*V.lr*m.Vx*delta/den;
    r=m.Vx*delta/den;
    yr((j-1)*7+1)=min(abs(vy),m.Vx*atan(.02*V.mu*V.g))*sign(delta);
    yr((j-1)*7+2)=min(abs(r),V.mu*V.g/max(m.Vx,1e-3))*sign(delta);
end
valid=true;
end

function [F,C]=tire(T,alpha,Fz)
% 载荷相关参数与 Fiala 型轮胎力；C 输出当前侧偏角处的导数。
z=min(max(Fz,T.fzlo),T.fzhi);
C=max(Fz,0)*polyval(T.cq,z); peak=max(max(Fz,0)*polyval(T.mq,z),1);
C=max(C/(1+T.kcs*C),1); t=tan(min(max(alpha,-1.4),1.4));
if abs(t)>=3*peak/C
    F=-peak*sign(t); C=0;
else
    F=-C*t+C^2/(3*peak)*abs(t)*t-C^3/(27*peak^2)*t^3;
    C=(-C+2*C^2/(3*peak)*abs(t)-C^3/(9*peak^2)*t^2)*(1+t^2);
end
end
