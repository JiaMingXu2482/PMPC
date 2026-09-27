function C_table = func_tire_init_Calpha(mu,FitRang)
% func_tire_init_Calpha(mu)
% 基于参考实验矩阵(exp_data@mu_ref)，用 Friction Similarity 转换到目标 mu，
% 再拟合得到 25° 内的轮胎线性侧偏刚度表（N/rad）。
%
% 输入:
%   mu (scalar) - 目标路面摩擦系数，例如 0.4
% 输出:
%   C_table(1x3) - 对应 4 个Fz点的 C_alpha (N/rad)
%
% 备注:
% - exp_data 第一行(第2列起)被当作 Fz(N) 

    if nargin < 1 || isempty(mu)
        mu = 1; % 默认（若你希望默认就是参考摩擦，可保持）
    end

    % ===== 实验矩阵（列1为角度，其余列为不同Fz下的Fy）=====
    exp_data = [0	2000	4000	6000	8000
                0.5	217.209	429.593	636.931	836.561
                1	429.757	850.025	1260.36	1655.53
                1.5	633.31	1252.77	1857.72	2440.49
                2	824.144	1630.48	2418.15	3177.3
                2.5	999.372	1977.45	2933.23	3854.91
                3	1157.07	2289.91	3397.37	4465.95
                3.5	1296.34	2566.04	3807.81	5006.79
                4	1417.2	2805.82	4164.49	5477.26
                4.5	1520.37	3010.7	4469.52	5880.02
                5	1607.17	3183.19	4726.55	6219.78
                5.5	1679.18	3326.45	4940.22	6502.57
                6	1738.18	3443.91	5115.6	6734.98
                6.5	1785.91	3539.05	5257.79	6923.65
                7	1824.04	3615.13	5371.62	7074.93
                7.5	1854.08	3675.14	5461.53	7194.6
                8	1877.36	3721.73	5531.44	7287.83
                8.5	1895.06	3757.2	5584.76	7359.1
                9	1908.16	3783.51	5624.41	7412.26
                9.5	1917.49	3802.31	5652.83	7450.52
                10	1923.74	3814.99	5672.09	7476.62
                10.5	1927.5	3822.68	5683.89	7492.8
                11	1929.24	3826.33	5689.65	7500.95
                11.5	1929.35	3826.74	5690.53	7502.58
                12	1928.16	3824.53	5687.49	7498.99
                12.5	1925.92	3820.23	5681.31	7491.21
                13	1922.86	3814.27	5672.63	7480.08
                13.5	1919.14	3806.99	5661.97	7466.31
                14	1914.92	3798.71	5649.79	7450.47
                14.5	1910.31	3789.64	5636.41	7433.04
                15	1905.4	3779.97	5622.13	7414.39
                15.5	1900.27	3769.85	5607.19	7394.83
                16	1895	3759.44	5591.77	7374.63
                16.5	1889.62	3748.82	5576.03	7353.99
                17	1884.19	3738.07	5560.1	7333.08
                17.5	1878.73	3727.27	5544.09	7312.04
                18	1873.27	3716.47	5528.07	7290.98
                18.5	1867.84	3705.72	5512.12	7269.99
                19	1862.45	3695.05	5496.27	7249.15
                19.5	1857.13	3684.5	5480.59	7228.5
                20	1851.87	3674.07	5465.1	7208.1
                20.5	1846.68	3663.79	5449.82	7187.98
                21	1841.58	3653.68	5434.79	7168.17
                21.5	1836.57	3643.74	5420.01	7148.69
                22	1831.65	3633.98	5405.5	7129.55
                22.5	1826.82	3624.4	5391.25	7110.76
                23	1822.08	3615.01	5377.28	7092.34
                23.5	1817.45	3605.81	5363.59	7074.28
                24	1812.9	3596.79	5350.18	7056.58
                24.5	1808.46	3587.96	5337.04	7039.25
                25	1804.11	3579.32	5324.18	7022.27];

    % ===== 1) 将 exp_data 从 mu_ref 转换到目标 mu（Friction Similarity）=====
    mu_ref = 1; % 参考摩擦：exp_data 所在的路面
    exp_data_mu = friction_similarity_expdata(exp_data, mu_ref, mu);

    % ===== 2) 用 exp_data_mu  拟合 C_alpha =====
    alpha_deg = exp_data_mu(2:end, 1);
    alpha_rad = alpha_deg * pi / 180;

    Fz_values = exp_data_mu(1, 2:end); % 4个载荷点（N）
    mask = (alpha_deg <= FitRang);
    a_sel = alpha_rad(mask);

    % Fiala 模型（幅值形式）
    fiala_mag = @(a, Fz, C) fiala_model_mag(a, Fz, C, mu);

    % 搜索范围（单参数）
    C_lower = 2e3; C_upper = 6e5;
    C_table = zeros(1, numel(Fz_values));
    opts = optimset('Display','off');

    for i = 1:numel(Fz_values)
        Fz = Fz_values(i);
        Fy_exp = exp_data_mu(2:end, i+1);
        Fy_sel = Fy_exp(mask);

        obj = @(C) sqrt(mean( (fiala_mag(a_sel, Fz, C) - Fy_sel).^2 ));
        C_table(i) = fminbnd(obj, C_lower, C_upper, opts);
    end
end

% ===== Friction Similarity：把 exp_data@mu_ref 变换到 exp_data@mu_tgt =====
function exp_data_mu = friction_similarity_expdata(exp_data, mu_ref, mu_tgt)
% 相似性：Fy(mu)= r * Fy_ref( r*alpha )，其中 r = mu_tgt/mu_ref  
    r = mu_tgt / mu_ref;

    alpha_deg = exp_data(2:end, 1);
    Fy_ref = exp_data(2:end, 2:end);   % Nalpha x 4
    Fz_values = exp_data(1, 2:end);    % 1 x 4

    alpha_ref_deg = r * alpha_deg;     % 需要在参考曲线的 (r*alpha) 处取值
    Fy_mu = zeros(size(Fy_ref));

    for j = 1:size(Fy_ref, 2)
        % 在 alpha_deg 网格上插值，求 Fy_ref(alpha_ref_deg)
        Fy_interp = interp1(alpha_deg, Fy_ref(:,j), alpha_ref_deg, 'pchip', 'extrap');
        Fy_mu(:,j) = r * Fy_interp;

        % （可选但推荐）物理限幅：不超过 mu_tgt * Fz
        Fy_mu(:,j) = min(Fy_mu(:,j), mu_tgt * Fz_values(j));
    end

    exp_data_mu = exp_data;
    exp_data_mu(2:end, 2:end) = Fy_mu;
end


% ---- Fiala 模型（幅值）----
function Fy = fiala_model_mag(alpha_rad, Fz, C_alpha, mu)
    a = abs(alpha_rad(:));  % 幅值
    alpha_slip = atan((3*mu*Fz)/C_alpha);

    Fy = zeros(size(a));
    in_lin = (a < alpha_slip);
    ar = a(in_lin);

    Fy(in_lin) = (C_alpha * ar) ...
               - (C_alpha^2/(3*mu*Fz)) .* (ar.^2) ...
               + (C_alpha^3/(27*mu^2*Fz^2)) .* (ar.^3);

    Fy(~in_lin) = mu * Fz;
end
