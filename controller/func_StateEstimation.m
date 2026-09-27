function [VehStatemeasured, HATParameter] = func_StateEstimation(ModelInput,VehiclePara)
%***************************************************************%
% we should do state estimation, but for simplicity we deem that the
% measurements are accurate
% Update the state vector according to the input of the S function,
%           usually do State Estimation from measured Vehicle Configuration
%***************************************************************%
%   ModelInput : 50x1, **顺序就是 CarSim 的 I/O Channels: Export 列表**
%                (1 CmpRD_L1 .. 4 CmpRD_R2, 5 Alpha_L1 .. 8, 9 Fx .. 12,
%                 13 Fy .. 16, 17 Fz .. 20, 21 Fd .. 24, 25 AVy .. 28,
%                 29 My_Dr .. 32, 33 AVx, 34 AVz, 35 Ax, 36 Ay, 37 Beta,
%                 38 Roll, 39 Yaw, 40 Xo, 41 Yo, 42 Vx, 43 Vy, 44 AAx,
%                 45 AAz, 46 Steer_L1, 47 Steer_R1, 48 Steer_SW,
%                 49 VxTarget, 50 LTR)   —— 完整对照见 func_PortMap
%
%   ⚠️ 2026-09-18 从 62 路改成 50 路: CarSim 的 Export 去掉了 CmpD_* / Zgnd_* /
%   Z_* 共 12 路(旧编号 5-16), 这里所有下标跟着重编 —— 旧的 n(>=17) 对应新的 n-12。
%   改法是脚本批量改 + 逐字节回归验证, 不是手工数。改之前先证过: 旧代码里生效的
%   下标集合恰好是 {1..4} ∪ {17..62}, 没有一处落在被删掉的 5-16 上。
    %******输入接口转换***%        
    g = 9.81;
%% ---- codegen: 结构体被读取后不能再加新字段, 先声明全部字段 ----
VehStatemeasured = struct('V_l1',0, 'V_l2',0, 'V_r1',0, 'V_r2',0, 'X',0, 'Y',0, 'x_dot',0, 'y_dot',0, 'Yaw',0, 'Yawrate',0, 'Yawaccel',0, 'beta',0, 'delta_l',0, 'delta_r',0, 'Steer_SW',0, 'Ax',0, 'Ay',0, 'omega_L1',0, 'omega_L2',0, 'omega_R1',0, 'omega_R2',0, 'VxTarget',0, 'delta_f',0);
HATParameter = struct('Roll',0, 'Rollrate',0, 'Rollaccel',0, 'alpha_l1',0, 'alpha_l2',0, 'alpha_r1',0, 'alpha_r2',0, 'Fx_l1',0, 'Fx_l2',0, 'Fx_r1',0, 'Fx_r2',0, 'Fy_l1',0, 'Fy_l2',0, 'Fy_r1',0, 'Fy_r2',0, 'Fz_l1',0, 'Fz_l2',0, 'Fz_r1',0, 'Fz_r2',0, 'Fd_L1',0, 'Fd_L2',0, 'Fd_R1',0, 'Fd_R2',0, 'Td_L1',0, 'Td_L2',0, 'Td_R1',0, 'Td_R2',0, 'LTR',0, 'MFx',0, 'Fyf',0, 'Fyr',0, 'Md',0, 'alphaf',0, 'alphar',0, 'Fzf',0, 'Fzr',0, 'Fxf',0, 'Fxr',0);
    VehStatemeasured.V_l1       = ModelInput(1)/1000;
    VehStatemeasured.V_l2       = ModelInput(2)/1000;
    VehStatemeasured.V_r1       = ModelInput(3)/1000;
    VehStatemeasured.V_r2       = ModelInput(4)/1000;
    VehStatemeasured.X       = ModelInput(40);%单位为m, 保留2位小数
    VehStatemeasured.Y       = ModelInput(41);%单位为m, 保留2位小数    
    VehStatemeasured.x_dot   = ModelInput(42)/3.6; %Unit:km/h-->m/s，保留1位小数  
    VehStatemeasured.y_dot   = ModelInput(43)/3.6; %Unit:km/h-->m/s，保留1位小数   
    VehStatemeasured.Yaw     = ModelInput(39)*pi/180; %航向角，Unit：deg-->rad，保留1位小数    
    VehStatemeasured.Yawrate = ModelInput(34)*pi/180; %Unit：deg/s-->rad/s，保留1位小数      
    VehStatemeasured.Yawaccel= ModelInput(45); %rad/s^2
    HATParameter.Roll        = ModelInput(38)*pi/180;% deg-->rad 
    HATParameter.Rollrate    = ModelInput(33)*pi/180;% deg/s-->rad/s
    HATParameter.Rollaccel   = ModelInput(44); % rad/s^2
    VehStatemeasured.beta    = ModelInput(37)*pi/180;% side slip, Unit:deg-->rad，保留1位小数    
    VehStatemeasured.delta_l = ModelInput(46)*pi/180;
    VehStatemeasured.delta_r = ModelInput(47)*pi/180;
    VehStatemeasured.Steer_SW= ModelInput(48); %deg
    VehStatemeasured.Ax      = g*ModelInput(35);%单位为m/s^2, 保留2位小数
    VehStatemeasured.Ay      = g*ModelInput(36);%单位为m/s^2, 保留2位小数
    HATParameter.alpha_l1   = ModelInput(5)*pi/180;   
    HATParameter.alpha_l2   = ModelInput(6)*pi/180;  
    HATParameter.alpha_r1   = ModelInput(7)*pi/180; 
    HATParameter.alpha_r2   = ModelInput(8)*pi/180; 
    HATParameter.Fx_l1      = round(1000*ModelInput(9))/1000; % N 
    HATParameter.Fx_l2      = round(1000*ModelInput(10))/1000; % N 
    HATParameter.Fx_r1      = round(1000*ModelInput(11))/1000; % N 
    HATParameter.Fx_r2      = round(1000*ModelInput(12))/1000; % N
    HATParameter.Fy_l1      = round(1000*ModelInput(13))/1000; % N 
    HATParameter.Fy_l2      = round(1000*ModelInput(14))/1000; % N 
    HATParameter.Fy_r1      = round(1000*ModelInput(15))/1000; % N 
    HATParameter.Fy_r2      = round(1000*ModelInput(16))/1000; % N
    HATParameter.Fz_l1      = round(1000*ModelInput(17))/1000;
    HATParameter.Fz_l2      = round(1000*ModelInput(18))/1000;
    HATParameter.Fz_r1      = round(1000*ModelInput(19))/1000;
    HATParameter.Fz_r2      = round(1000*ModelInput(20))/1000;
    HATParameter.Fd_L1      = round(1000*ModelInput(21))/1000;
    HATParameter.Fd_L2      = round(1000*ModelInput(22))/1000;
    HATParameter.Fd_R1      = round(1000*ModelInput(23))/1000;
    HATParameter.Fd_R2      = round(1000*ModelInput(24))/1000;
    HATParameter.Td_L1      = ModelInput(29);
    HATParameter.Td_L2      = ModelInput(30);
    HATParameter.Td_R1      = ModelInput(31);
    HATParameter.Td_R2      = ModelInput(32);
    VehStatemeasured.omega_L1   = ModelInput(25)*pi/30;
    VehStatemeasured.omega_L2   = ModelInput(26)*pi/30;
    VehStatemeasured.omega_R1   = ModelInput(27)*pi/30;
    VehStatemeasured.omega_R2   = ModelInput(28)*pi/30;
    VehStatemeasured.VxTarget    = ModelInput(49); %km/h
    HATParameter.LTR            = ModelInput(50);
    % [alpha] = func_lamda_alpha(VehStatemeasured,VehiclePara); 
    % HATParameter.alpha_l1  = alpha(1);  HATParameter.alpha_l2  = alpha(2);   
    % HATParameter.alpha_r1  = alpha(3);  HATParameter.alpha_r2  = alpha(4);
%% 计算等效值
    VehStatemeasured.delta_f = 0.5*(ModelInput(46)+ ModelInput(47))*pi/180; % deg-->rad
    if VehiclePara.k_cs > 0
        % 建模柔性时, 控制器的前轮转角取运动学转角 (方向盘角/传动比), 与前轴有效特性同口径
        VehStatemeasured.delta_f = ModelInput(48)*pi/180/VehiclePara.isw;
    end
    HATParameter.MFx  = HATParameter.Fx_r1 + HATParameter.Fx_r2 - HATParameter.Fx_l1 - HATParameter.Fx_l2;%忽略前轴纵向滚动阻力
    HATParameter.Fyf  = HATParameter.Fy_l1 + HATParameter.Fy_r1;
    HATParameter.Fyr  = HATParameter.Fy_l2 + HATParameter.Fy_r2;
    %  ldf/ldr 是左右减振器间距, 力臂是一半 (与 func_QPA_CDC 的 Bd = 0.5*[-df df -dr dr] 同式)
    HATParameter.Md   = 0.5*((HATParameter.Fd_R1- HATParameter.Fd_L1)*VehiclePara.ldf + (HATParameter.Fd_R2  - HATParameter.Fd_L2 )*VehiclePara.ldr);
    HATParameter.alphaf   = (HATParameter.alpha_l1 + HATParameter.alpha_r1)/2;
    HATParameter.alphar   = (HATParameter.alpha_l2 + HATParameter.alpha_r2)/2;
    HATParameter.Fzf = HATParameter.Fz_l1+HATParameter.Fz_r1;
    HATParameter.Fzr = HATParameter.Fz_l2+HATParameter.Fz_r2;
    HATParameter.Fxf = HATParameter.Fx_l1+HATParameter.Fx_r1;
    HATParameter.Fxr = HATParameter.Fx_l2+HATParameter.Fx_r2;
%     HATParameter.alphaf     =(ModelInput(14)+ ModelInput(16))/2;   % 死代码: 下标是更早以前的编号, 与现在的 50 路无关
%     if     HATParameter.alpha_r1 == 0
%         HATParameter.alphar = HATParameter.alpha_r2;
%     elseif HATParameter.alpha_r2 == 0
%         HATParameter.alphar = HATParameter.alpha_r1;
%     end %end of HATParameter.alphar,防止一侧后两轴中一侧车轮抬起导致alpha_r没有值

% 估计
  

    
%     [Fz] = func_Fz(VehStatemeasured,VehiclePara);
%     HATParameter.Fz_l1 = Fz(1);  HATParameter.Fz_l2 = Fz(2);  HATParameter.Fz_l3 = Fz(3);
%     HATParameter.Fz_r1 = Fz(4);  HATParameter.Fz_r2 = Fz(5);  HATParameter.Fz_r3 = Fz(6);
    
%     [Fx_fl, Fy_fl] = func_Tire_force(lambda(1), alpha(1), Fz(1));
%     [Fx_ml, Fy_ml] = func_Tire_force(lambda(2), alpha(2), Fz(2));
%     [Fx_rl, Fy_rl] = func_Tire_force(lambda(3), alpha(3), Fz(3));
%     [Fx_fr, Fy_fr] = func_Tire_force(lambda(4), alpha(4), Fz(4));
%     [Fx_mr, Fy_mr] = func_Tire_force(lambda(5), alpha(5), Fz(5));
%     [Fx_rr, Fy_rr] = func_Tire_force(lambda(6), alpha(6), Fz(6));
%     HATParameter.Fx_l1 = Fx_fl;  HATParameter.Fx_l2  = Fx_ml;  HATParameter.Fx_l3 = Fx_rl;
%     HATParameter.Fx_r1 = Fx_fr;  HATParameter.Fx_r2  = Fx_mr;  HATParameter.Fx_r3 = Fx_rr;
%     HATParameter.Fy_l1 = Fy_fl;  HATParameter.Fy_l2  = Fy_ml;   HATParameter.Fy_l3 = Fy_rl;
%     HATParameter.Fy_r1 = Fy_fr;  HATParameter.Fy_r2  = Fy_mr;   HATParameter.Fy_r3 = Fy_rr;



end % end of func_StateEstimation

function [alpha] = func_lamda_alpha(VehStatemeasured,VehiclePara)

    vx          = VehStatemeasured.x_dot;
    vy          = VehStatemeasured.y_dot;
    psidot      = VehStatemeasured.phi_dot;
    delta_l     = VehStatemeasured.delta_l;
    delta_r     = VehStatemeasured.delta_r;
%     omega_fl    = VehStatemeasured.omega_L1;
%     omega_ml    = VehStatemeasured.omega_L2;
%     omega_rl    = VehStatemeasured.omega_L3;
%     omega_fr    = VehStatemeasured.omega_R1;
%     omega_mr    = VehStatemeasured.omega_R2;
%     omega_rr    = VehStatemeasured.omega_R3;
    g           = VehiclePara.g;
    lf          = VehiclePara.lf;
    tf          = VehiclePara.tf;
%     tm          = VehiclePara.tm;
    tr          = VehiclePara.tr;
%     lm          = VehiclePara.lm;
    lr          = VehiclePara.lr;
    rt          = VehiclePara.rt;

    % 车轮旋转中心速度
%     Vx_fl = (vx - tf/2*psidot)*cos(delta_l) + (vy + lf*psidot)*sin(delta_l);
%     Vx_fr = (vx + tf/2*psidot)*cos(delta_r) + (vy + lf*psidot)*sin(delta_r);
%     Vx_ml = vx - tm/2*psidot;
%     Vx_mr = vx + tm/2*psidot;
%     Vx_rl = vx - tr/2*psidot;
%     Vx_rr = vx + tr/2*psidot;
    %轮胎线速度
%     Vw_fl = omega_fl*rt; Vw_fr = omega_fr*rt;
%     Vw_ml = omega_ml*rt; Vw_mr = omega_mr*rt;
%     Vw_rl = omega_rl*rt; Vw_rr = omega_rr*rt;
    %轮胎侧偏角
    alpha_fl = atan((vy + psidot * lf) / (vx - tf * psidot / 2)) - delta_l;
    alpha_rl = atan((vy - psidot * lr) / (vx - tr * psidot / 2));
    alpha_fr = atan((vy + psidot * lf) / (vx + tf * psidot / 2)) - delta_r;
    alpha_rr = atan((vy - psidot * lr) / (vx + tr * psidot / 2));%rad
    alpha(1) = alpha_fl; alpha(2) = alpha_rl; 
    alpha(4) = alpha_fr; alpha(5) = alpha_rr; 
    %纵向滑移率
%     lambda_fl = (max(Vw_fl,Vx_fl)-min(Vw_fl,Vx_fl))/max(Vw_fl, Vx_fl)*sign(Vw_fl-Vx_fl);
%     lambda_fr = (max(Vw_fr,Vx_fr)-min(Vw_fr,Vx_fr))/max(Vw_fr, Vx_fr)*sign(Vw_fr-Vx_fr);
%     lambda_ml = (max(Vw_ml,Vx_ml)-min(Vw_ml,Vx_ml))/max(Vw_ml, Vx_ml)*sign(Vw_ml-Vx_ml);
%     lambda_mr = (max(Vw_mr,Vx_mr)-min(Vw_mr,Vx_mr))/max(Vw_mr, Vx_mr)*sign(Vw_mr-Vx_mr);
%     lambda_rl = (max(Vw_rl,Vx_rl)-min(Vw_rl,Vx_rl))/max(Vw_rl, Vx_rl)*sign(Vw_rl-Vx_rl);
%     lambda_rr = (max(Vw_rr,Vx_rr)-min(Vw_rr,Vx_rr))/max(Vw_rr, Vx_rr)*sign(Vw_rr-Vx_rr);
%     lambda(1) = lambda_fl; lambda(2) = lambda_ml; lambda(3) = lambda_rl;
%     lambda(4) = lambda_fr; lambda(5) = lambda_mr; lambda(6) = lambda_rr;

end

function [Fz] = func_Fz(VehStatemeasured,VehiclePara)
    %% 计算载荷转移和垂直载荷
    ax      = VehStatemeasured.Ax;
    ay      = VehStatemeasured.Ay;
    m       = VehiclePara.m;
    ms      = VehiclePara.ms;
    g       = VehiclePara.g;
    lf      = VehiclePara.lf;
    L       = VehiclePara.L;
    lr1     = VehiclePara.lr1;
    lm      = VehiclePara.lm;
    lr      = VehiclePara.lr;
    hg      = VehiclePara.hg;
    rt      = VehiclePara.rt;
    Kf      = VehiclePara.Kt;
    kf      = VehiclePara.kf;
    km      = VehiclePara.km;
    kr      = VehiclePara.kr;      % 悬架弹簧线刚度(N/m)
    kw      = VehiclePara.kw;
    cf      = VehiclePara.cf; 
    cm      = VehiclePara.cm; 
    cr      = VehiclePara.cr;      % 前轴线阻尼(N*s/m) 
    lsf     = VehiclePara.lsf;       %前轴减振器间距(m)
    lsm     = VehiclePara.lsm;
    lsr     = VehiclePara.lsr;
    ldf     = VehiclePara.ldf;        %L_DAMPERS(1)1轴两减震器之间的距离Distance between dampers on axle 1
    ldm     = VehiclePara.ldm;
    ldr     = VehiclePara.ldr;
    omega   = VehiclePara.omega;

    h_a1    = 0.432;
    h_a2    = 0.725;
    h_a3    = h_a2;      % 530+195，同上 
    h_ar    = (h_a2+h_a3)/2;         % 组合轴质心到地面的高度
    hr      = lf/L*(h_ar-h_a1)+h_a1;
    hs      = hg-hr;
    tf      = 2.07;
    tm      = 2.07;
    tr      = 2.07;

    kef=(kw*cf^2*omega^2+kf*kw*(kw+kf))/(cf^2*omega^2+(kf+kw)^2);
    kem=(kw*cm^2*omega^2+km*kw*(kw+km))/(cm^2*omega^2+(km+kw)^2);
    ker=(kw*cr^2*omega^2+kr*kw*(kw+kr))/(cr^2*omega^2+(kr+kw)^2);

    Kf=kef*lsf^2;
    Km=kem*lsm^2;
    Kr=ker*lsr^2;% 等效侧倾角刚度 (N·m/rad)   
    Kt=Kf+Km+Kr;
    Cf=cf*ldf^2;
    Cm=cm*ldm^2;
    Cr=cr*ldr^2;
    Ct=Cf+Cm+Cr;
  
    Fz_fl_static = m*g*lr1/(2*L);
    Fz_fr_static = Fz_fl_static;
    Fz_ml_static = m*g*lf/(4*L);
    Fz_mr_static = Fz_ml_static;
    Fz_rl_static = m*g*lf/(4*L); %fprintf('Fz_ml_static: %.1f s\n', Fz_ml_static);
    Fz_rr_static = Fz_rl_static;
        
    delta_Fz_x_fl = -m*ax*hg/(2*L);
    delta_Fz_x_fr =  delta_Fz_x_fl;
    delta_Fz_x_ml =  m*ax*hg/(4*L);
    delta_Fz_x_mr =  delta_Fz_x_ml;
    delta_Fz_x_rl =  m*ax*hg/(4*L);
    delta_Fz_x_rr =  delta_Fz_x_rl;
    
    delta_Fz_y_fl = -ay*m/tf*(Kf*hs/(Kt-m*g*hs)+lr1*h_a1/L);
    delta_Fz_y_fr = -delta_Fz_y_fl;
    delta_Fz_y_ml = -(lf*m*ay*h_a2/L+(2*Km*ms*ay*hr)/(Kt-ms*g*hr))/tm;
    delta_Fz_y_mr = -delta_Fz_y_ml;
    delta_Fz_y_rl = -(lf*m*ay*h_a2/L+(2*Kr*ms*ay*hr)/(Kt-ms*g*hr))/tr;
    delta_Fz_y_rr = -delta_Fz_y_rl;
    
    Fz(1) = Fz_fl_static+delta_Fz_x_fl+delta_Fz_y_fl;
    Fz(2) = Fz_fr_static+delta_Fz_x_fr+delta_Fz_y_fr;
    Fz(3) = Fz_ml_static+delta_Fz_x_ml+delta_Fz_y_ml;
    Fz(4) = Fz_mr_static+delta_Fz_x_mr+delta_Fz_y_mr;
    Fz(5) = Fz_rl_static+delta_Fz_x_rl+delta_Fz_y_rl;
    Fz(6) = Fz_rr_static+delta_Fz_x_rr+delta_Fz_y_rr;
    for i=1:6
        if Fz(i)<0
            Fz(i)=0;
        end
    end
        
end

function [fx, fy] = func_Tire_force(lambda, alpha, fz)
    % 函数：计算轮胎纵向力 Fx 和侧向力 Fy
    % 输入：
    %   lambda - 纵向滑移率数组，无单位，范围 [-1, 1]
    %   alpha  - 侧偏角数组，单位：度，范围 [-40, 40]
    %   fz     - 垂向载荷数组，单位：N，正值
    % 输出：
    %   fx     - 纵向力数组，单位：N，正负均可
    %   fy     - 侧向力数组，单位：N，正负均可
    % 备注：当 lambda 或 alpha 为负时，Fx 和 Fy 关于原点对称

    % 将垂向载荷从 N 转换为 kN
    fz = fz / 1000; % kN

    % === 纵向力参数 (单位：kN) ===
    b0  =   1.6859;  	%C 参数 (形状因子)
    b1  =  -0.0017;  	%D1参数 (刚度因子系数, 1/kN^2)
    b2  =   0.7591;  	%D2参数 (刚度因子系数, 1/kN)
    b3  =   0.0126;  	%B1参数 (曲率因子系数, 1/kN^2)
    b4  =   6.4557;  	%B2参数 (曲率因子系数, 1/kN)
    b5  =   0.0017;  	%B3参数 (曲率衰减系数, 1/kN)
    b6  =  -0.0000;  	%E1参数 (曲率偏移系数, 1/kN^2)
    b7  =   0.0126;  	%E2参数 (曲率偏移系数, 1/kN)
    b8  =  -2.4489;  	%E3参数 (曲率偏移常数)
    b9  =   0;  	%S_h1参数 (滑移率偏移系数, 1/kN)
    b10 =   0;  	%S_h2参数 (滑移率偏移常数)
    % === 侧向力参数 (单位：kN) ===
    A0  =   1.6791;  	%C 参数 (形状因子)
    A1  =  -0.0030;  	%D1参数 (峰值二次系数, 1/kN^2)
    A2  =   0.7194;  	%D2参数 (峰值线性系数, 1/kN)
    A3  = 358.6829;  	%B1参数 (刚度因子)
    A4  =  98.1203;  	%B2参数 (参考载荷, kN)
    A5  =  -0.2264;  	%B3参数 (外倾角衰减因子)
    A6  =  -0.0002;  	%E1参数 (曲率二次系数, 1/kN^2)
    A7  =   0.3548;  	%E2参数 (曲率常数)
    A8 = 0;  % S_h1参数 (外倾角偏移系数)
    A9 = 0;   % S_h2参数 (载荷偏移系数, 1/kN)
    A10 = 0; % S_h3参数 (偏移常数)
    A11 = 0;%0.0595;  % S_v1参数 (外倾角交互系数, 1/kN)
    A12 = 0;%0.0674;  % S_v2参数 (载荷垂直偏移系数, 1/kN)

    % === 矢量化计算纵向力 F_x (单位：kN) ===
    % 魔术公式参数
    C_x = b0 * ones(size(fz));
    D_x = b1 * fz.^2 + b2 * fz;
    B_numerator = b3 * fz.^2 + b4 * fz;
    B_denominator = C_x .* D_x .* exp(b5 * fz);
    B_x = B_numerator ./ B_denominator;
    E_x = b6 * fz.^2 + b7 * fz + b8;
    S_h_x = 0;%b9 * (fz-1) + b10;
    S_v_x = -0.03; % 纵向力无垂直偏移
    x_x = abs(lambda) + S_h_x; % 使用绝对值计算正向力

    % 计算 F_x (kN)
    fx_kN_pos = D_x .* sin(C_x .* atan(B_x .* x_x - E_x .* (B_x .* x_x - atan(B_x .* x_x)))) + S_v_x;
    fx_kN = fx_kN_pos .* sign(lambda); % 调整符号以满足对称性

    % === 矢量化计算侧向力 F_y (单位：kN) ===
    % 外倾角 gamma (2 度，转换为弧度)
    gamma_rad = 2 * pi / 180 * ones(size(fz));

    % 侧偏角 alpha 转换为弧度
    alpha_rad = abs(alpha) * pi / 180; % 使用绝对值计算正向力

    % 魔术公式参数
    C_y = A0 * ones(size(fz));
    D_y = A1 * fz.^2 + A2 * fz;
    B_y = A3 * sin(2 * atan(fz ./ A4)) .* (1 - A5 * abs(gamma_rad)) ./ (C_y .* D_y);
    E_y = A6 * fz.^2 + A7;
    S_h_y = 0;%A8 * gamma_rad + A9 * fz + A10;
    S_v_y =0; %A11 * fz .* gamma_rad + A12;
    x_y = alpha_rad + S_h_y;

    % 计算 F_y (kN)
    fy_kN_pos = D_y .* sin(C_y .* atan(B_y .* x_y - E_y .* (B_y .* x_y - atan(B_y .* x_y)))) + S_v_y;
    fy_kN = -fy_kN_pos .* sign(alpha); % 调整符号以满足对称性

    % === 单位转换：kN 转为 N ===
    fx = fx_kN * 1000;
    fy = fy_kN * 1000;
end