function [Fd_L1,Fd_L2,Fd_R1,Fd_R2,Md_real,exitflag_Fd] = func_QPA_CDC( ...
    VehiclePara,InitialParams,VehStateMeasured,MFd,Fdu,Fdl,verbose)

    df = VehiclePara.ldf;
    dr = VehiclePara.ldr;

    % --- damper relative velocities (measured; 现仅供调试/记录) ---
    Vd_L1 = VehStateMeasured.V_l1;   % front-left
    Vd_R1 = VehStateMeasured.V_r1;   % front-right
    Vd_L2 = VehStateMeasured.V_l2;   % rear-left
    Vd_R2 = VehStateMeasured.V_r2;   % rear-right

    % --- QP weights ---
    rho = 0.005;

    % Decision vector order: [F_fl; F_fr; F_rl; F_rr]
    Bd = 0.5 * [ -df,  df,  -dr,  dr ];  % Md = Bd * Fd

    % --- bounds (robust: allow any sign ordering) ---
    Fdu = Fdu(:);
    Fdl = Fdl(:);
    lb = min(Fdl, Fdu);
    ub = max(Fdl, Fdu);

    % warm start
    if isfield(InitialParams, 'prevstate') && isfield(InitialParams.prevstate, 'Fd') ...
            && ~isempty(InitialParams.prevstate.Fd)
        x0 = InitialParams.prevstate.Fd(:);
        if numel(x0) ~= 4
            x0 = zeros(4,1);
        end
    else
        x0 = zeros(4,1);
    end
    x0 = min(max(x0, lb), ub);           % warm start 必须在界内

    % --- 正则项 1: 按可达能力归一化 (最小化各角相对利用率) ---
    %  与差动制动按 Fz 分配同一思想: 各角能出的力范围 [lb,ub] 正比于 |v|,
    %  按 1/span^2 加权即最小化 sum (Fd_i/span_i)^2, 负担按能力均摊。
    %  原式 Wd=1/(v^2+1e-2) 形状正确但**量级失控**: 低速角 rho*Wd=0.5 已压过
    %  Bd'*Bd 的 0.3025, 正则项从"在零空间里挑解"变成"和跟踪目标抢权重",
    %  实测丢掉 20% 的请求力矩。这里归一化到均值 1, 保形状不抢权重。
    spn = max(ub - lb, 1);
    Wd  = (mean(spn) ./ spn).^2;
    Wd  = Wd / mean(Wd);                 % 均值归一, rho 的含义与量纲无关
    Gamma_d = diag(Wd);

    % --- 正则项 2: 连续性, 压制零空间游走 ---
    %  Bd*Fd=MFd 有 3 维零空间, 四角力可大幅重排而 Md 不变, active-set 在这些
    %  等价解间跳动就是阻尼力高频抖动的来源(实测 Md 有 60% 能量在 5Hz 以上)。
    %  罚 ||Fd - Fd_prev||^2 直接抑制游走, 不影响可达的 Md。
    rho_c = 0.05;

    H = 2 * (Bd' * Bd + rho * Gamma_d + rho_c * eye(4));
    f = -2 * (Bd' * MFd + rho_c * x0);

    %  (原 'quadprog' options 已删: 改用 KWIK 后用不到, 且会把 quadprog 拖进生成代码)

    %  R2018a 的 quadprog 不支持 codegen, 分配 QP 也必须改 KWIK。
    %  本 QP 只有边界约束(A/b 为空), H 正定(含正则项), 满足 KWIK 前提。
    %  实测只要 1~3 次迭代, 故冷启动(usews=false), 避开 persistent 共用问题。
    [Fd, st_, ~] = func_QPKwik(H, f, zeros(0,4), zeros(0,1), lb, ub, 200, false);
    if st_ > 0, exitflag_Fd = 1; else, exitflag_Fd = st_ - 10; end

    if exitflag_Fd <= 0 || any(~isfinite(Fd))
        if verbose
            fprintf(2, 'func_QPA_CDC: MR damper allocation QP failed; returning zeros\n');
        end
        Fd = zeros(4,1);
        exitflag_Fd = -1;
    end

    % unpack
    Fd_L1 = Fd(1);   % fl
    Fd_R1 = Fd(2);   % fr
    Fd_L2 = Fd(3);   % rl
    Fd_R2 = Fd(4);   % rr

    % --- realized anti-roll moment from allocated forces ---
    Md_real = Bd * Fd;   % scalar

end
