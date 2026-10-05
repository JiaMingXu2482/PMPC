function [WayPoints_Collect] = func_WayPoints(maneuverType, Rjt, saveOutput, turnX)
% func_WayPoints 生成参考路径点
% 输入 maneuverType: 
%   1 - Double Lane Change (DLC) [基于你的拟合参数]
%   2 - Slalom (蛇形穿桩), 与 CarSim X-Y-Station 表逐点一致
%   3 - U-Turn (U型弯) [基于论文描述: 直线40m + R60m]
%   5 - J-turn: 直线 60 m + 半径 Rjt 的 90 度左转圆弧 + 出弯直线 100 m (改进.md 7.1)
%       Rjt 按 R = V^2/(mu_eff*g) 取: 80 km/h 时 mu=0.85 -> 69 m (需求 = 能力),
%       mu=0.5 -> 90 m (需求/能力 = 1.31, 强制不足转向)。
%
% 默认参数为 1 (DLC)
if nargin < 1
    maneuverType = 1;
end
if nargin < 2 || isempty(Rjt)
    Rjt = 69;
end
if nargin < 3
    saveOutput = true;
end
if nargin < 4
    turnX = [];
end
stationInput = [];

%% 1. 路径点生成 (X, Y)
switch maneuverType
    case 1  % ===== DLC (Double Lane Change) =====
        % 你的拟合参数
        % x1  = 50;        % 切入中心
        % x2  = 102.5;     % 切回中心
        % dy1 = 1.75;      % 幅值参数 (3.5/2)
        % dy2 = 1.75; 
        % dx  = 15.2;      % 拟合的特征长度
        % c   = 0;
        %  原来是从 DLC 的 .mat 里取 cfit 对象再求值, 现改成显式公式。
        %  这里保留显式公式，避免重新引入已移除的 cfit 对象依赖。
        %  两个原因:
        %   1. cfit 是 Curve Fitting Toolbox 的类, R2024b 存下来的对象在
        %      R2018a 上**加载不了** —— curvefit.nlsqoptions 类定义不兼容,
        %      部署机上直接报 "Cannot evaluate CFIT model for some reason."
        %   2. 顺带去掉对 Curve Fitting Toolbox 的依赖(部署机不一定有许可)
        %  公式取自 formula(fitresult), 系数取自 coeffvalues(fitresult)。
        %  已验证 X=(0:0.5:200)' 共 401 个点**逐位相同**, max|diff| = 0。
        Afit = 3.5564696255156258;
        Lf1  = 27.85220278516676;
        Lf2  = 24.007795423319223;
        xc1  = 80.058689040522395;
        xc2  = 132.35698740891445;
        X = (0:0.5:200)';
        Y = Afit/2*(1+tanh(4/Lf1*(X-xc1))) - Afit/2*(1+tanh(4/Lf2*(X-xc2)));
        % z1 = (2.4/dx) * (X - x1) + c;
        % z2 = (2.4/dx) * (X - x2) + c;
        
        % 拟合公式
        % Y = dy1 * (1 + tanh(z1)) - dy2 * (1 + tanh(z2));
    case 2  % Slalom: exact CarSim X-Y-Station table, including preview tail.
        xys = dlmread(fullfile(func_ProjectRoot(),'data','Slalom_CarSim_XYS.txt'),',');
        assert(size(xys,2)==3 && size(xys,1)>2 && all(diff(xys(:,3))>0), ...
            'func_WayPoints:InvalidSlalomPath','Slalom X-Y-Station table is invalid.');
        X = xys(:,1);
        Y = xys(:,2);
        stationInput = xys(:,3);

    case 3  % ===== U-Turn (三段：直线40m + R60半圆 + 出弯直线) =====
        ds    = 0.25;   % 采样间距
        Rroad = 60;    % 半径 (论文描述) [1](https://mailscuteducn-my.sharepoint.com/personal/202420100352_mail_scut_edu_cn/Documents/Microsoft%20Copilot%20Chat%20%E6%96%87%E4%BB%B6/NMPC-Based%20Path%20Tracking%20Control%20Strategy%20for%20Autonomous%20Vehicles%20With%20Stable%20Limit%20Handling.pdf)
        L1    = 20;    % 第一段直线长度 (论文描述) [1](https://mailscuteducn-my.sharepoint.com/personal/202420100352_mail_scut_edu_cn/Documents/Microsoft%20Copilot%20Chat%20%E6%96%87%E4%BB%B6/NMPC-Based%20Path%20Tracking%20Control%20Strategy%20for%20Autonomous%20Vehicles%20With%20Stable%20Limit%20Handling.pdf)
        L_exit = 60;   % 第三段直线长度（你可按需要改，比如 20/40/60）
    
        % ---------- 第一段：直线 (0 -> 40m) ----------
        X_str1 = (0:ds:L1)';
        Y_str1 = zeros(size(X_str1));
    
        % ---------- 第二段：U-turn 半圆 ----------
        % 用几何更直观：从(40,0)开始，绕圆心(40, Rroad)左转半圈
        % 参数角theta：-pi/2 -> +pi/2
        arc_len = pi * Rroad;
        n_arc   = ceil(arc_len/ds) + 1;
        theta   = linspace(-pi/2, +pi/2, n_arc)';
    
        centerX = L1;
        centerY = Rroad;
    
        X_arc = centerX + Rroad * cos(theta);
        Y_arc = centerY + Rroad * sin(theta);
    
        % ---------- 第三段：出弯后直线 ----------
        % 半圆结束点在 (L1, 2*Rroad)，航向约为 pi（朝 -X）
        X_end = X_arc(end);
        Y_end = Y_arc(end);
    
        X_str3 = (X_end:-ds:(X_end - L_exit))';
        Y_str3 = Y_end * ones(size(X_str3));
    
        % ---------- 拼接（去除连接处重复点） ----------
        X = [X_str1; X_arc(2:end); X_str3(2:end)];
        Y = [Y_str1; Y_arc(2:end); Y_str3(2:end)];
case 4  % ===== Combined Maneuver: DLC -> U-Turn =====
        % ---------------------------------------------------------
        % Phase A: Double Lane Change (same as Case 1)
        % ---------------------------------------------------------
        x1  = 50;        % 切入中心
        x2  = 102.5;     % 切回中心
        dy1 = 1.75;      % 幅值参数 (3.5/2)
        dy2 = 1.75;
        dx  = 15.2;      % 特征长度
        c   = 0;
    
        ds_dlc = 0.5;
        X_dlc = (0:ds_dlc:140)'; 
        z1 = (2.4/dx) * (X_dlc - x1) + c;
        z2 = (2.4/dx) * (X_dlc - x2) + c;
        Y_dlc = dy1 * (1 + tanh(z1)) - dy2 * (1 + tanh(z2));
    
        % 可选：确保 DLC 起始段严格对齐中心线
        Y_dlc(X_dlc <= 20) = 0;
    
        % DLC 末端（拼接参考点）
        X0 = X_dlc(end);
        Y0 = Y_dlc(end);
    
        % ---------------------------------------------------------
        % Phase B: U-Turn (same geometry idea as Case 3, but offset to DLC end)
        % ---------------------------------------------------------
        ds_ut   = 0.25;   % 采样间距
        Rroad   = 60;     % 半径
        L1      = 100;     % 进入直线长度
        L_exit  = 60;     % 出弯直线长度（可调）
    
        % --- U-Turn local coordinates (start at 0,0) ---
        % 1) 直线：0 -> L1
        X_str1 = (0:ds_ut:L1)';
        Y_str1 = zeros(size(X_str1));
    
        % 2) 半圆：圆心 (L1, Rroad)，theta: -pi/2 -> +pi/2
        arc_len = pi * Rroad;
        n_arc   = ceil(arc_len/ds_ut) + 1;
        theta   = linspace(-pi/2, +pi/2, n_arc)';
    
        centerX = L1;
        centerY = Rroad;
    
        X_arc = centerX + Rroad * cos(theta);
        Y_arc = centerY + Rroad * sin(theta);
    
        % 3) 出弯直线：从半圆终点沿 -X 方向走 L_exit
        X_end_local = X_arc(end);
        Y_end_local = Y_arc(end);
    
        X_str3 = (X_end_local:-ds_ut:(X_end_local - L_exit))';
        Y_str3 = Y_end_local * ones(size(X_str3));
    
        % --- 拼接 U-Turn local（去重） ---
        X_ut_local = [X_str1; X_arc(2:end); X_str3(2:end)];
        Y_ut_local = [Y_str1; Y_arc(2:end); Y_str3(2:end)];
    
        % --- 将 U-Turn 平移到 DLC 末端（全局坐标） ---
        X_ut = X_ut_local + X0;
        Y_ut = Y_ut_local + Y0;
    
        % ---------------------------------------------------------
        % Phase C: Concatenate DLC and U-Turn (remove duplicate joint)
        % ---------------------------------------------------------
        X = [X_dlc; X_ut(2:end)];
        Y = [Y_dlc; Y_ut(2:end)];
    case 5  % ===== J-turn: 直线 + 90 度左转圆弧 + 出弯直线 =====
        %  曲率在进弯处阶跃(0 -> 1/R), 与 J-turn 的转向阶跃同性质; 控制器按弧长取真曲率。
        %  车从 X = 30 m 出发(CarSim SV_XO), 进弯前约 30 m / 1.35 s 直线;
        %  出弯直线 100 m 保证 TSTOP = 10 s 内不跑出路径末端。
        if nargin < 2 || isempty(Rjt), Rjt = 69; end
        ds    = 0.5;
        L1    = 60;
        L3    = 100;
        X_str1 = (0:ds:L1)';
        Y_str1 = zeros(size(X_str1));
        n_arc  = ceil((pi/2*Rjt)/ds) + 1;
        theta  = linspace(-pi/2, 0, n_arc)';          % 圆心 (L1, Rjt), 从 (L1,0) 左转到 (L1+Rjt, Rjt)
        X_arc  = L1 + Rjt*cos(theta);
        Y_arc  = Rjt + Rjt*sin(theta);
        Y_str3 = (Rjt:ds:(Rjt + L3))';                % 出弯后朝 +Y
        X_str3 = (L1 + Rjt)*ones(size(Y_str3));
        X = [X_str1; X_arc(2:end); X_str3(2:end)];
        Y = [Y_str1; Y_arc(2:end); Y_str3(2:end)];
    case 6  % DLC -> recovery straight -> parameterized-radius J-turn, one continuous path
        C = func_CombinedCourse(Rjt,turnX);
        dlc = func_WayPoints(1,C.turn_radius,false);
        X_dlc = dlc(:,2); Y_dlc = dlc(:,3);
        ds = 0.5;
        X_mid = (C.dlc_end_x+ds:ds:C.turn_x)';
        Y_mid = zeros(size(X_mid));
        n_arc = ceil((pi/2*C.turn_radius)/ds)+1;
        theta = linspace(-pi/2,0,n_arc)';
        X_arc = C.turn_x + C.turn_radius*cos(theta);
        Y_arc = C.turn_radius*(1+sin(theta));
        Y_exit = (C.turn_radius+ds:ds:C.turn_radius+C.exit_length)';
        X_exit = (C.turn_x+C.turn_radius)*ones(size(Y_exit));
        X = [X_dlc; X_mid; X_arc(2:end); X_exit];
        Y = [Y_dlc; Y_mid; Y_arc(2:end); Y_exit];
    otherwise
        error('Maneuver Type unknown. Use 1 (DLC), 2 (Slalom), 3 (U-Turn) or 5 (J-turn).');
end


%% 2. 通用后处理 (计算航向角 psi, 距离 s, 曲率 K)
numPoints = length(X);
WayPoints_Collect = zeros(numPoints, 9); 

WayPoints_Collect(:,1) = (1:numPoints)'; % Index
WayPoints_Collect(:,2) = X;              % Global X
WayPoints_Collect(:,3) = Y;              % Global Y

% --- 修正部分：数值微分计算航向角 ---
% 使用 gradient 计算差分
dX = gradient(X);
dY = gradient(Y);

% 使用 atan2 而不是 atan，以处理 U-Turn 垂直路段 (dX=0) 和 180度掉头的情况
psi_ref = atan2(dY, dX); 

% 修正：对于 DLC 起始段，强制设为 0 以消除数值噪音
if maneuverType == 1 || maneuverType == 6
    psi_ref(X <= 20) = 0; 
    Y(X <= 20) = 0;      % 确保起始点完全对齐
    WayPoints_Collect(:,3) = Y;
end

WayPoints_Collect(:,4) = psi_ref;        % Heading Angle (rad)

% --- 计算累计路程 s ---
s = zeros(numPoints, 1);
for i = 2:numPoints
    dist = sqrt((X(i)-X(i-1))^2 + (Y(i)-Y(i-1))^2);
    s(i) = s(i-1) + dist;
end
if ~isempty(stationInput)
    assert(max(abs(s-stationInput))<0.05,'func_WayPoints:SlalomStationMismatch', ...
        'Slalom X-Y and Station columns disagree.');
    s = stationInput;
end
WayPoints_Collect(:,7) = s;              % Station (m)

% --- 计算曲率 K ---
K = zeros(numPoints, 1);
% 航向角解卷绕 (Unwrap) 处理，防止 +/- Pi 跳变导致的曲率尖峰
psi_unwrap = unwrap(psi_ref);

for i = 2:numPoints-1
    d_psi = psi_unwrap(i+1) - psi_unwrap(i-1);
    ds_Local = s(i+1) - s(i-1);
    
    if abs(ds_Local) < 1e-6
        K(i) = 0;
    else
        K(i) = d_psi / ds_Local;
    end
end
% 边界填充
K(1) = K(2);
K(end) = K(end-1);

WayPoints_Collect(:,5) = K;              % Curvature (1/m)

%% 3. 保存与输出
filename = sprintf('WayPoints_Type%d.mat', maneuverType);
if maneuverType == 5, filename = sprintf('WayPoints_Type5_R%g.mat', Rjt); end   % 半径不同的 J-turn 分开存
if maneuverType == 6, filename = sprintf('WayPoints_Type6_R%g.mat', Rjt); end
if maneuverType == 5 || maneuverType == 6
    outdir = fullfile(func_ProjectRoot(), 'data', 'generated');
else
    outdir = fullfile(func_ProjectRoot(), 'data');
end
if saveOutput
    if exist(outdir,'dir') ~= 7, mkdir(outdir); end
    save(fullfile(outdir, filename), 'WayPoints_Collect');
end
% fprintf('路经类型 %d 生成完成，点数: %d，已保存至 %s\n', maneuverType, numPoints, filename);

%% 4. (可选) 绘图验证
% figure(101); clf;
% subplot(2,1,1);
% plot(X, Y, 'b-', 'LineWidth', 1.5); grid on;
% xlabel('X [m]'); ylabel('Y [m]'); title(['Trajectory Type: ' num2str(maneuverType)]);
% subplot(2,1,2);
% plot(s, psi_ref*180/pi, 'r-'); grid on;
% xlabel('s [m]'); ylabel('Heading [deg]');
end
