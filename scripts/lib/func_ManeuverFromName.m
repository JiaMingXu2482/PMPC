function mv = func_ManeuverFromName(dsname)
%FUNC_MANEUVERFROMNAME  从数据集名解析工况类型、名义 J-turn 半径和路面 mu。
% 实际仿真半径由 func_ManeuverFromRun 从 CarSim Run_all.par 覆盖。
%   mv = func_ManeuverFromName('JT80_R69_mu0.85_PMPC')
%     mv.type : 1 = DLC, 2 = Slalom, 5 = J-turn, 6 = DLC+J-turn
%     mv.R    : J-turn 圆弧半径 [m] (DLC 时为 NaN)
%     mv.mu   : 路面附着系数 (须与 CarSim 路面数据集一致)
%     mv.wp   : 参考路径文件名 (func_WayPoints 存的那个)
%   命名约定: <工况><车速>_[R<半径>_]mu<附着>_<控制器>, 例如
%     DLC80_mu0.5_PMPC、JT80_R69_mu0.85_ZENG。
%   解析不到时(部署机没有 simfile.sim 等)退回 DLC、mu = 0.5, 与 2026-09-26 之前写死的值相同。
mv = struct('type', 1, 'R', NaN, 'mu', 0.5, 'wp', 'WayPoints_Type1.mat', ...
    'combined',false, 'mu_high',0.5, 'mu_switch_x',0, 'stop_station',NaN);
if nargin < 1 || isempty(dsname), return; end
U  = upper(char(dsname));
t  = regexp(U, 'MU([0-9]*\.?[0-9]+)', 'tokens', 'once');
if ~isempty(t), mv.mu = str2double(t{1}); end
if strncmp(U,'SLALOM',6)
    mv.type = 2;
    mv.mu = 0.85;  % Current CarSim Slalom_mu1_* runs use the shared 0.85 road.
    mv.mu_high = mv.mu;
    mv.wp = 'WayPoints_Type2.mat';
    mv.stop_station = 160;
    return;
end
if strncmp(U,'COMB',4)
    r = regexp(U,'_JT_R?([0-9]+(?:\.[0-9]+)?)_', 'tokens','once');
    assert(~isempty(r),'func_ManeuverFromName:NoCombinedRadius', ...
        'Combined dataset must include _JT_R69_: %s',dsname);
    C = func_CombinedCourse(str2double(r{1}));
    mv.type = 6;
    mv.R = C.turn_radius;
    mv.turn_x = C.turn_x;
    mv.mu = C.mu_dlc;
    mv.mu_high = C.mu_jturn;
    mv.mu_switch_x = C.mu_switch_x;
    mv.combined = true;
    mv.wp = sprintf('WayPoints_Type6_R%g.mat',mv.R);
    return;
end
if strncmp(U, 'JT', 2)
    r = regexp(U, '_R([0-9]+(\.[0-9]+)?)_', 'tokens', 'once');
    assert(~isempty(r), 'func_ManeuverFromName:NoRadius', ...
           'J-turn 数据集名里必须带半径, 例如 JT80_R69_mu0.85_PMPC, 实际为 %s', dsname);
    mv.type = 5;
    mv.R    = str2double(r{1});
    mv.wp   = sprintf('WayPoints_Type5_R%g.mat', mv.R);
end
end
