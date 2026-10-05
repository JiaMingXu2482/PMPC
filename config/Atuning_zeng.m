function T = Atuning_zeng()
%ATUNING_ZENG  Editable tuning defaults for ZENG rho weighting.
T.controller = 'ZENG';
T.Ts = 0.1;
T.Np = 10;
T.Nc = 5;

T.Q1 = 0; T.Q2 = 0; T.Q3 = 100;
T.Q4 = 0; T.Q5 = 2e4; T.Q6 = 2e2;
T.R1 = 1; T.R2 = 10; T.R3 = 5;
T.S1 = 0; T.S2 = 0; T.S3 = 0;
T.Qf_scale = 1;
T.V1 = 100; T.V2 = 8e4; T.V3 = 1;
T.tau_gamma = 0;
T.Wdg = [0 0 0];
T.W1 = 5e4; T.W2 = 5e4; T.W3 = 5e6; T.W4 = 5e6;

% ZENG stability indicator and its longitudinal speed governor.
T.ZengLong_on = 0;
T.Zg_rfloor = 0.05;     % rad/s
T.Zg_Vmin = 30/3.6;    % m/s
T.Zg_tau = 2.5;        % s, speed-error to deceleration
T.Zg_tpid = 0.3;       % s, CarSim speed-setpoint filter
T.Zg_cmax = 3;
T.Zg_ayfull = 0;       % 0: paper's steady-state ay ~= Vx*r
T.Zg_kovs = 1.3;
T.Zg_Npv = 12;         % preview nodes (capped by Np)
T.Zg_apv = 0;
T.Zg_dVdb = 0.3;       % m/s
end
