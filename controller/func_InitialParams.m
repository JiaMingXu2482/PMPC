function [Pa]=func_InitialParams
    Pa = struct();
    % 初始化各种参数
    Pa.InitialGapflag = 0;  % 初始间隙标志
    Pa.WayPoints_IndexPre = 1;  % 路径点索引
    
    % 失败统计相关
    Pa.failed_num = struct();
    Pa.failed_num.solve = 0;
    Pa.failed_num.conv = 0;
    Pa.failed_num.unsolved = 0;
    Pa.failed_num.NaN_or_Inf = 0;
    Pa.failed_num.else = 0;
    
    % 最大误差相关
    Pa.emax = struct();
    Pa.emax.y = 0;
    Pa.emax.psi = 0;
    
    % 控制输入
    Pa.U = zeros(3,1);
    
    % 先前状态
    Pa.prevstate = struct();
    Pa.prevstate.sw = 0;
    Pa.prevstate.Fyf = 0;
    Pa.prevstate.MFx = 0;
    Pa.prevstate.Md = 0;
    Pa.prevstate.ey = 0;
    Pa.prevstate.epsi = 0;
    Pa.prevstate.Tb = zeros(4,1);
    Pa.prevstate.Fd = zeros(4,1);
    Pa.prevstate.s_act = -ones(4,1);  % 阻尼器执行器状态(func_DamperActuator), <0 = 未初始化
    Pa.prevstate.Vd = zeros(4,1);

    % ---- codegen: 下面这些原来是跑的时候靠 ~isfield 惰性创建的。
    %  代码生成不允许结构体被读取后再加字段, 故改为预声明 +
    %  哨兵值(NaN = 尚未初始化), 调用处的守卫改成 isnan 判定。
    %  数值上与原来逐位等价。
    Pa.dr_prev = NaN;                 % 原: ~isfield(InitialParams,'dr_prev')
    Pa.dr_rate = 0;
    Pa.prevstate.gamma = [0; 0; 1];   % 原: 首拍建立, 初值仅 MR 激活
    Pa.prevstate.Vpid  = NaN;         % 原: ~isfield(...,'Vpid')
    Pa.prevstate.iA    = false(0,1);  % KWIK 活动集热启动
    Pa.t_solve   = zeros(1,20000);    % 诊断缓冲
    Pa.n_solve   = 0;
    Pa.ey_hist   = zeros(1,20000);
    Pa.epsi_hist = zeros(1,20000);
end
