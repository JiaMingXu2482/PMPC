function [Tb_L1,Tb_L2,Tb_R1,Tb_R2] = func_QPA_DB(VehiclePara,InitialParams,Constraints,ParaHAT,MFx,delta_wheel,Tb_u,Fx_dem,verbose)
%   verbose=0 时不打印失败警告(失败计数不受影响)。见 setup_pmpc 的 MPCParameters.Verbose
%  Fx_dem (可选): 纵向制动力需求 (N, 正=减速)。为 0 时与原版**逐位等价**。
if nargin < 8 || isempty(Fx_dem), Fx_dem = 0; end

    lf          = VehiclePara.lf;
    mu          = VehiclePara.mu;
    tf          = VehiclePara.tf;
    tr          = VehiclePara.tr;
    rt          = VehiclePara.rt;
    Fz_L1       = ParaHAT.Fz_l1;
    Fz_L2       = ParaHAT.Fz_l2;
    Fz_R1       = ParaHAT.Fz_r1;
    Fz_R2       = ParaHAT.Fz_r2;

 
    rho    =  0.1;%越大跟踪越准确，0.1
    Tb_max = Constraints.Tb_max;%Trucksim数据
    Tb_fl_max = min(Tb_u.Tb_L1,Tb_max);
    Tb_rl_max = min(Tb_u.Tb_L2,Tb_max);
    Tb_fr_max = min(Tb_u.Tb_R1,Tb_max);
    Tb_rr_max = min(Tb_u.Tb_R2,Tb_max);
    % 构造分配矩阵 B (参考公式(84)的矩阵形式)
    Bb = (1/rt) * [   (tf/2)*cos(delta_wheel)-lf*sin(delta_wheel),...
                    -(tf/2)*cos(delta_wheel)-lf*sin(delta_wheel), ...
                      tr/2, -tr/2 ];
    % 轮胎离地时 Fz=0，直接计算 1/Fz^2 会令 H=Inf 并使 quadprog
    % 终止仿真。该轮的制动力上界此时已经是 0，因此只需用一个很小的
    % 正载荷保持正则项有限。
    Fz_eff = max([Fz_L1, Fz_R1, Fz_L2, Fz_R2], 1);
    Gamma_u = diag((1./(rt*mu*Fz_eff)).^2);

    H = 2 * (Bb'*Bb + rho*Gamma_u);
    f = -2 * Bb' * MFx;

    % ---- 分配器增量惩罚 ----
    %  Bb'*Bb 是秩 1 (4 个未知数、只一个横摆目标), 解在 3 维零空间里
    %  不唯一; 原有正则项 rho*Gamma_u ~ 1e-7 相对 Bb'*Bb ~ 5 小七个数量级,
    %  等于没有 —— 逐拍由 active-set 的搜索路径随机挑一个解, 轮缸力矩就乱跳。
    %  这里用上一拍的解钉住零空间分量。归一化沿用本文件的约定:
    %    lam_d = 1  <=>  「用满单轮力矩能力的增量」与「用满差动制动横摆能力
    %    的残差」同价; <1 => 横摆优先。与 Gov_lamx 同一套量纲。
    Fz_tot = Fz_L1 + Fz_R1 + Fz_L2 + Fz_R2;
    T_ref  = max(rt*mu*Fz_tot/4, 1);       % N*m, 单轮制动力矩能力
    M_ref  = max(mu*Fz_tot*tf/4,  1);      % N*m, 差动制动的纯横摆能力(粗略)
    lam_d  = Constraints.QPA_lamd;
    if lam_d > 0
        Tb_pre = InitialParams.prevstate.Tb(:);
        if numel(Tb_pre) ~= 4 || any(~isfinite(Tb_pre)), Tb_pre = zeros(4,1); end
        kd = lam_d * (M_ref/T_ref)^2;
        H  = H + 2*kd*eye(4);
        f  = f - 2*kd*Tb_pre;
    end

    % ---- 纵向减速需求（仅在限速器触发时叠加）----
    %  把“减速”放进同一个分配 QP，而不是另开一路制动：
    %    - 横摆力矩 MFx 仍是主目标，纵向项权重由 lam_x<1 压低；
    %    - 上界 ub 本身就是摩擦圆给的，故不会抢走横向能力；
    %    - Fx_dem=0 时本段不执行，保证对原行为零回归。
    if Fx_dem > 0
        Bx = (1/rt) * [cos(delta_wheel), cos(delta_wheel), 1, 1];   % 总制动力
        F_ref  = max(mu*Fz_tot, 1);            % N,   纯纵向能力 (Fz_tot/M_ref 上面已算)
        lam_x  = Constraints.Gov_lamx;         % <1 => 横摆优先
        kx     = lam_x * (M_ref/F_ref)^2;      % 把纵向残差折算到横摆残差的量纲
        H = H + 2*kx*(Bx'*Bx);
        f = f - 2*kx*(Bx'*Fx_dem);
    end

    ub = [ Tb_fl_max;Tb_fr_max;Tb_rl_max;Tb_rr_max;];  % 上限
    lb = zeros(4, 1);          % 下限
    
    x0 = InitialParams.prevstate.Tb(:);
    if numel(x0) ~= 4 || any(~isfinite(x0))
        x0 = zeros(4,1);
    end
    x0 = min(max(x0, lb), ub);

    % 5. 优化求解设置

    %  (原 'quadprog' options 已删: 改用 KWIK 后用不到, 且会把 quadprog 拖进生成代码)
    
    % 6. 调用quadprog求解
    %  R2018a 的 quadprog 不支持 codegen, 分配 QP 也必须改 KWIK。
    %  本 QP 只有边界约束(A/b 为空), H 正定(含正则项), 满足 KWIK 前提。
    %  实测只要 1~3 次迭代, 故冷启动(usews=false), 避开 persistent 共用问题。
    [Tb, st_, ~] = func_QPKwik(H, f, zeros(0,4), zeros(0,1), lb, ub, 200, false);
    if st_ > 0, exitflag_DB = 1; else, exitflag_DB = st_ - 10; end

    if exitflag_DB <= 0
        if verbose
            fprintf(2, 'func_QPA_DB: brake torque allocation QP failed, exitflag_DB=%.0f\n', exitflag_DB);
        end
        Tb = zeros(4, 1);
    end

    Tb_L1 = Tb(1);  
    Tb_L2 = Tb(3);  
    Tb_R1 = Tb(2);  
    Tb_R2 = Tb(4);  
%     Tb_L1 = 0;  Tb_L2 = 0;  
%     Tb_R1 = 0;  Tb_R2 = 0;  
end

