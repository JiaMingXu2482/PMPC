function InitialParams = func_ReportStatus(InitialParams, exitflag, PrjP, Vel, t_Elapsed, verbose)
% func_ReportStatus  统计求解结果、跟踪最大误差并打印运行信息
%   verbose=0 时只统计不打印 (codegen 用, 见 setup_pmpc 的 MPCParameters.Verbose)
% -------------------------------------------------------------------------

% --- 求解结果计数 ---
switch exitflag
    case  1, InitialParams.failed_num.solve      = InitialParams.failed_num.solve      + 1;
    case  0, InitialParams.failed_num.conv       = InitialParams.failed_num.conv       + 1;
    case -2, InitialParams.failed_num.unsolved   = InitialParams.failed_num.unsolved   + 1;
    case -3, InitialParams.failed_num.NaN_or_Inf = InitialParams.failed_num.NaN_or_Inf + 1;
    otherwise
             InitialParams.failed_num.else       = InitialParams.failed_num.else       + 1;
end

% --- 求解耗时记录（R3#9 需要报告 mean/max 求解时间与迭代次数）---
%  codegen: t_solve/n_solve 已在 func_InitialParams 里预分配
InitialParams.n_solve = InitialParams.n_solve + 1;
if InitialParams.n_solve <= numel(InitialParams.t_solve)
    InitialParams.t_solve(InitialParams.n_solve) = t_Elapsed;
end

% --- 跟踪误差时间历程（诊断用）---
%  codegen: ey_hist/epsi_hist 已预分配
if InitialParams.n_solve <= numel(InitialParams.ey_hist)
    InitialParams.ey_hist(InitialParams.n_solve)   = PrjP.ey;
    InitialParams.epsi_hist(InitialParams.n_solve) = PrjP.epsi;
end

% --- 最大跟踪误差 ---
if abs(PrjP.ey)   >= abs(InitialParams.emax.y)
    InitialParams.emax.y   = abs(PrjP.ey);
end
if abs(PrjP.epsi) >= abs(InitialParams.emax.psi)
    InitialParams.emax.psi = abs(PrjP.epsi);
end

if verbose
    fprintf(['Vel: %.0f\nMPC solve %.0f ms, sim %.2f s\n' ...
             'max ey: %.2f m, max epsi: %.2f deg\n' ...
             'solved:%.0f notconv:%.0f infeas:%.0f NaNInf:%.0f other:%.0f\n'], ...
             Vel*3.6, t_Elapsed, InitialParams.InitialGapflag*0.01, ...
             InitialParams.emax.y, rad2deg(InitialParams.emax.psi), ...
             InitialParams.failed_num.solve, InitialParams.failed_num.conv, ...
             InitialParams.failed_num.unsolved, InitialParams.failed_num.NaN_or_Inf, ...
             InitialParams.failed_num.else);
end


end
