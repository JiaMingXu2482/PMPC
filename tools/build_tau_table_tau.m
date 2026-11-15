function build_tau_table_tau()
%BUILD_TAU_TABLE_TAU  t90 -> tau 换算，生成控制器可用的时滞表。
%
%   数据源：MR_NLCSNN/SH_delay/tau_table_cases.csv
%           （build_tau_table.m 对减振器正模型做 180 组恒速电流阶跃辨识的原始结果）
%   输出  ：pmpc_core/data/tau_table_tau.mat
%
% 为什么需要这一步：
%   identify 出来的 t2_ms 是"电流阶跃后，力变化达到 90% 的时刻"，即 t90。
%   预测模型里的 tau 是"命令到实际力矩"一阶环节的时间常数。两者不是一回事：
%
%       t90 = L + 2.3026*tau        (一阶 + 纯滞后 L，ln(10)=2.3026)
%       t10 = L + 0.1054*tau
%       => tau = (t90 - t10)/2.1972,  L = t10 - 0.1054*tau
%
%   直接把 t90 当 tau 用会高估 2.3 倍。本脚本用 t10/t90 双点换算，
%   顺手给出死区 L —— 实测 L 中位约 0.7 ms，接近 0，说明这个执行器就是一阶的。
%
% 条件定义（v>0 为压缩，CarSim 力符号压缩为正）：
%   comp_up 压缩升电流 / comp_dn 压缩降电流 / reb_up 复原升电流 / reb_dn 复原降电流
%   四组的 tau 差 2.4 倍（19.6 vs 8.2 ms），所以不能用固定值时滞。

root = fileparts(fileparts(mfilename('fullpath')));
srcCSV = 'D:\OneDrive - mail.scut.edu.cn\Thesis\MR_NLCSNN\SH_delay\tau_table_cases.csv';
outDir = fullfile(root,'pmpc_core','data');
if ~isfolder(outDir), mkdir(outDir); end
assert(isfile(srcCSV), '找不到辨识原始结果：%s', srcCSV);

T = readtable(srcCSV);
I_grid = [0 0.4 0.8 1.2 1.6];
v_grid = [-400 -300 -200 -100 0 100 200 300 400];
A = 2.302585 - 0.105361;

ok = isfinite(T.t1_ms) & isfinite(T.t2_ms) & (T.t2_ms > T.t1_ms);
tau_case = (T.t2_ms(ok) - T.t1_ms(ok)) / A;
L_case   = T.t1_ms(ok) - 0.105361*tau_case;

tau_tau = nan(numel(I_grid),numel(I_grid),numel(v_grid));
tau_t90 = nan(numel(I_grid),numel(I_grid),numel(v_grid));
for k = find(ok).'
    [~,a] = min(abs(I_grid - T.I_start(k)));
    [~,b] = min(abs(I_grid - T.I_target(k)));
    [~,j] = min(abs(v_grid - T.v_rel(k)));
    tau_tau(a,b,j) = (T.t2_ms(k) - T.t1_ms(k))/A;
    tau_t90(a,b,j) = T.t2_ms(k);
end
% 对角（I_start == I_target，没有阶跃）填全表条件中位数
for i = 1:numel(I_grid)
    tau_tau(i,i,:) = median(tau_case);
    tau_t90(i,i,:) = median(T.t2_ms(ok));
end

% 四种条件 x 九个速度
cond_names = {'comp_up','comp_dn','reb_up','reb_dn'};
cond_tau = nan(4,numel(v_grid));
sv = sign(T.v_rel);
sd = sign(T.I_target - T.I_start);
cid = zeros(height(T),1);
cid(sv> 0 & sd> 0) = 1;
cid(sv> 0 & sd< 0) = 2;
cid(sv< 0 & sd> 0) = 3;
cid(sv< 0 & sd< 0) = 4;
for c = 1:4
    for j = 1:numel(v_grid)
        sel = ok & (cid==c) & (T.v_rel==v_grid(j));
        if any(sel)
            cond_tau(c,j) = median((T.t2_ms(sel)-T.t1_ms(sel))/A);
        end
    end
end

tau_I_grid = I_grid; tau_v_grid = v_grid;
save(fullfile(outDir,'tau_table_tau.mat'),'tau_tau','tau_t90','tau_I_grid','tau_v_grid', ...
     'cond_names','cond_tau','A');
fprintf('tau: %.1f .. %.1f ms, median %.1f ms\n', min(tau_tau(:),[],'omitnan'),max(tau_tau(:),[],'omitnan'),median(tau_tau(:),'omitnan'));
fprintf('死区 L 中位 %.1f ms\n', median(L_case(~isnan(L_case))));
fprintf('saved %s\n', fullfile(outDir,'tau_table_tau.mat'));
end
