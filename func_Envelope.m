function [Envelope, r_ssmax] = func_Envelope(VehiclePara, Constraints,VehStateMeasured)

    a   = VehiclePara.lf;
    b   = VehiclePara.lr;
    lr  = VehiclePara.lr;
    M   = VehiclePara.m;
    g   = VehiclePara.g;
    tf  = VehiclePara.tf;      % 轨距
    Kt  = VehiclePara.Kt;
    % Ct = VehiclePara.Ct;     % 论文口径下 LTR 不用阻尼项
    CarHat = VehiclePara.CarHat;   % 名义(线性区)刚度 -> 只给 r_ssmax
    Vel    = VehStateMeasured.x_dot;

    Alphar_lim = Constraints.arlim;
    LTR_lim    = Constraints.LTR_lim;
    es         = Constraints.es;

    % ---- 后轴侧偏/横摆稳态上界（论文式(20)）----
    r_ssmax = -CarHat * Alphar_lim * (1 + b/a) / (M * Vel);  % [1](https://mailscuteducn-my.sharepoint.com/personal/202420100352_mail_scut_edu_cn/Documents/Microsoft%20Copilot%20Chat%20%E6%96%87%E4%BB%B6/XJM_Coordinated_Control_of_Multiple_Actuators_Under_Extreme_Conditions_Based_on_Priority_Strategy.pdf)

    % ---- 道路（左右边界）约束（论文式(24)）----
    LM_middle = 0;
    Yroad_L   = Constraints.Roadwidth/2;      % 单车道半宽，可按工况改
    Eymin = LM_middle - Yroad_L;
    Eymax = Eymin + 2*Yroad_L;

    % ---- 车身四角点 (论文 III-B, 2026-09-25) ----
    %  原来只约束质心 e_y, 余量取半轮距 tf/2 + es, 没有车长和航向。
    %  现在约束矩形车身的四个角: 横向偏移 e_y + a*e_psi +- W_v/2 (a = a_f 或 -a_r),
    %  左边界只需看左前/左后角, 右边界只需看右前/右后角 -> 4 行。
    %  状态顺序 x = [Vy r phi dphi ey epsi]。
    %  松弛映射 Eenv: 上界两行共用 eps_ey+, 下界两行共用 eps_ey- (松弛仍 8 维)。
    af  = Constraints.env_af;   ar = Constraints.env_ar;
    Ddv = Constraints.env_Wv/2 + Constraints.env_es;
    Envelope.Henv = [ 0 0 0 0  1  af 0;
                      0 0 0 0  1 -ar 0;
                      0 0 0 0 -1 -af 0;
                      0 0 0 0 -1  ar 0 ];   % 第 7 列 = Md_a
    lane_big = 1e3*double(Constraints.lane_off);   % 诊断: 1 = 车道角点约束右端 +1000 m, 永不起作用 (改进.md 18z)
    Envelope.Genv = [ Eymax - Ddv;
                      Eymax - Ddv;
                     -Eymin - Ddv;
                     -Eymin - Ddv ] + lane_big;
    Envelope.Eenv = [ zeros(4,6), [1 0; 1 0; 0 1; 0 1] ];
    %  弯道修正: 车身轴线上距质心 a 处的点, 相对弯曲路径的横向偏移为
    %      e_y + a*e_psi - kappa*a^2/2      (前后都减; 左弯 kappa>0)
    %  小曲率近似丢掉的正是这一项。DLC 回正段 kappa ~ 0.035, 车尾 a_r = 2.7 m
    %  -> 0.13 m, 与余量同量级。实测: 不加时沿一条"精确几何下不越界"的轨迹
    %  误报 5.8%, 加上后 0.6%, 公式误差 0.13 -> 0.04 m (2026-09-25)。
    %  kappa 逐预测步已知(预测模型本来就用它), 只改约束右端, 仍是线性约束:
    %      b_env(i) = Genv + kappa_i * gkap
    Envelope.gkap = [ af^2/2; ar^2/2; -af^2/2; -ar^2/2 ];

    % ---- yaw/alpha 软约束 ----
    if isfield(Constraints,'ZengRho_on') && Constraints.ZengRho_on
        % === 先进对比方法: Zeng 2025 的稳定性约束 (原文 Eq.26e) ===
        %  横摆:  |r| <= mu*g/Vx                       (Eq.(12), 原文引 [25])
        %  侧偏:  Eq.(27)  -alpha_r_sat + lr*r/ux <= beta <= alpha_r_sat + lr*r/ux
        %           与 alpha_r = beta - lr*r/ux 合并后即 |alpha_r| <= alpha_r_sat,
        %           alpha_r_sat = atan( mu*m*g*lf / (C_alpha*L) )  后轴饱和侧偏角
        %
        %  【为何不用鞍点查表做约束】Eq.(12) 的 beta 鞍点界是给**指标**
        %  Eq.(13) 用的; 原文 MPC 约束 Eq.(26e) 用的是 Eq.(27) 的解析式。
        %  实测: 本车在 mu=0.5/80km/h 无两侧稳定域, 查表零填充节点参与插值后
        %  得到 beta 走廊仅 0.78 deg, 而实需 5.36 deg —— 差 13 倍, 会把控制器压垮。
        %  C_alpha 取**当前切线刚度**而非名义值 —— 原文 Eq.(7) 自己的定义:
        %  "where C_alpha_r is the **current** rear cornering stiffness"。
        %  理由: Eq.(27) 是线性轮胎假设下推的, 等价于"线性模型下 a_y <= mu*g"。
        %  实测本车在 a_y~0.5g 时 alpha_r 已达 4.70 deg (轮胎峰值在 9.5 deg,
        %  割线刚度仅名义值的 44%), 用名义刚度代入会把界收紧 2.3 倍。
        %  上限用 Fiala 全滑移角 (Eq.(2): alpha_sl = atan(3*mu*Fz/C_alpha)) 防止
        %  切线刚度塔陷时界发散。
        Fzr_st  = M * g * a / (a + b);                      % 后轴静载
        Cr_cur  = max(abs(VehiclePara.CbarR), 1e3);         % 当前后轴切线刚度
        a_sat   = atan( VehiclePara.mu * Fzr_st / Cr_cur );
        a_sl    = atan( 3 * VehiclePara.mu * Fzr_st / abs(CarHat) );   % Fiala 全滑移角
        alpha_r_sat = min(a_sat, a_sl);
        Envelope.Hsh = [  0      1      0 0 0 0 0;
                          0     -1      0 0 0 0 0;
                          1/Vel -lr/Vel 0 0 0 0 0;
                         -1/Vel  lr/Vel 0 0 0 0 0 ];   % 第 7 列 = Md_a
        Gsh_ = [ Constraints.Zeng_rmax; Constraints.Zeng_rmax;
                 alpha_r_sat;           alpha_r_sat ];
    else
        Envelope.Hsh = [  0      1      0 0 0 0 0;
                          0     -1      0 0 0 0 0;
                          1/Vel -lr/Vel 0 0 0 0 0;
                         -1/Vel  lr/Vel 0 0 0 0 0 ];   % 第 7 列 = Md_a
        Gsh_ = [ r_ssmax; r_ssmax; Alphar_lim; Alphar_lim ];
    end
    Envelope.Gsh = Gsh_;

    % ---- LTR 约束（动态式，无准静态假设）----
    ms      = VehiclePara.ms;
    h_TL    = VehiclePara.h_TL;
    h_S2R   = VehiclePara.h_S2R;
    delta_f = VehStateMeasured.delta_f;

    kappa_g = M*h_TL - ms*h_S2R;      % 341.76 kg*m
    bta     = 2/(M*g*tf);             % 6.9617e-5

    % a_y = c_ay*xi + d_ay*u   （= vydot + Vel*r，与 A/B 第一行一致）
    % 仿射轮胎模型下 a_y = (Fyf*cos(delta) + Fyr)/m，前后轴都进 c_ay，
    % Ddelta 进 d_ay；两个仿射偏置的常值部分在主函数里移到 Gr。
    CbF = VehiclePara.CbarF;  CbR = VehiclePara.CbarR;
    cd_f = cos(delta_f);
    c_ay = [ (CbF*cd_f + CbR)/(M*Vel), ...
             (CbF*a*cd_f - CbR*lr)/(M*Vel), 0, 0, 0, 0, 0 ];   % Md_a 不进 a_y
    d_ay = [ -CbF*cd_f/M, 0, 0 ];

    %  2026-09-25: LTR 用**已建立**的阻尼力矩 Md_a(状态第 7 列), 不再用指令 Md_c(输入)。
    %  论文 III-B: 实际载荷转移由阻尼器已经建立的力矩决定, 于是 LTR 约束成为纯状态约束,
    %  指令 Md_c 经时延状态才影响 LTR。
    Hr_row = bta*( kappa_g*c_ay + [0 0 Kt 0 0 0 1] );
    Or_row = bta*( kappa_g*d_ay + [0 0 0] );          % 只剩 AFS 经 a_y 的直通项

    Envelope.Hr = [ Hr_row; -Hr_row ];
    Envelope.Or = [ Or_row; -Or_row ];
    Envelope.Gr = [ LTR_lim; LTR_lim ];

    % ---- 约束行的归一化尺度 (PrioMode, 论文 III-D, 2026-09-26) ----
    %  每行除以其界的大小: 违反量 0.1 即超出界 10%。LTR 与车道行用名义界
    %  (不含 a_y 偏置与弯道修正项), 横摆/后轴侧偏行用本拍的界。
    %  codegen: 结构体读过之后不能再加字段, 故用局部量 Gsh_ 而不读 Envelope.Gsh。
    Envelope.sc_sh  = max(abs(Gsh_), 1e-6);
    Envelope.sc_r   = LTR_lim * [1; 1];
    Envelope.sc_env = max([Eymax - Ddv; Eymax - Ddv; -Eymin - Ddv; -Eymin - Ddv], 1e-3);

end
