function T = Atuning_pmpc()
%ATUNING_PMPC  Editable tuning defaults for priority PMPC.
T.controller = 'PMPC';
T.Ts = 0.1;
T.Np = 10;
T.Nc = 5;

T.Q1 = 0; T.Q2 = 0; T.Q3 = 1e2;
T.Q4 = 0; T.Q5 = 2e4; T.Q6 = 2e2;
T.R1 = 1; T.R2 = 10; T.R3 = 5;
T.S1 = 0; T.S2 = 0; T.S3 = 0;
T.Qf_scale = 1;
T.V1 = 100; T.V2 = 8e4; T.V3 = 1;
T.tau_gamma = 0;
T.Wdg = [0 0 0];
T.W1 = 5e4; T.W2 = 5e4; T.W3 = 5e6; T.W4 = 5e6;

% Priority rule: L1 road/rollover, L2 stability, L3 tracking/actuators.
T.prio_vartheta = 2;
T.prio_wmin = 1;
T.prio_wmax = [1e10; 1e8];

% PMPC-only preview speed coordination; 0 disables, 2 is curvature-only.
T.LongCoordMode = 1;
T.Long_mu_reserve = 0.85;
T.Long_brake_reserve = 0.70;
T.Long_a_max = 3.0;             % m/s^2
T.Long_preview_nodes = 80;      % fixed code-generation array bound
T.Long_min_sustain_m = 30;      % m
T.Long_delay = 0.15;            % s
T.Long_tau = 0.35;              % s
T.Long_F_slew = 2e5;            % N/s
T.Long_VdownRate = 4.0;         % m/s^2
T.Long_VupRate = 1.5;           % m/s^2
T.Long_margin_trigger = 0.20;   % m
T.Long_margin_recover = 0.35;   % m
T.Long_trigger_band = 0.10;     % m/s
T.Long_release_band = 0.40;     % m/s
end
