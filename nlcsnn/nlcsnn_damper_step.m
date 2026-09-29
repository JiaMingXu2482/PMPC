function [F, h_new] = nlcsnn_damper_step(net, x, v, a, curr, dcurr, temp, dt, h)
%NLCSNN_DAMPER_STEP  One macro time step of the NLCSNN CDC damper model.
%   %#codegen compatible (no file I/O, no persistent state).
%
%   [F, h_new] = nlcsnn_damper_step(net, x, v, a, curr, dcurr, temp, dt, h)
%
%   Inputs (physical units, scalars):
%     net   struct from nlcsnn_damper_init (weights + normalization)
%     x     damper displacement            [mm]
%     v     damper velocity                [mm/s]
%     a     damper acceleration            [mm/s^2]
%     curr  damper current command         [A]
%     dcurr current derivative             [A/s]
%     temp  damper temperature             [degC]
%     dt    macro step length              [s]  (e.g. 0.01 for 10 ms)
%     h     hidden state, 8x1 (zeros at start)
%
%   Outputs:
%     F     damper force                   [N]
%     h_new updated hidden state, 8x1
%
%   Inside, the macro step is substepped with RK4 at 1 ms (the training
%   step): n_sub = round(dt/0.001), inputs held constant (zero-order hold).
%
%   Simulink wiring (one corner): MATLAB Function block calling this,
%   with a Unit Delay (8-dim, initial 0) feeding h back to h_new.
%   Use one block instance per corner so each keeps its own state.
%
%   Inference order per time step (matches training/validation rollout):
%     y = predict_force(u, h)   then   h = rk4_step(u, h)
%   i.e. the reported force uses the state at the START of the step.

%#codegen

n = net.norm;

% ---- normalize physical inputs -> u (6x1) ----
u = [(x - n.x_ref) / n.x_scale; ...
      v / n.v_scale; ...
      a / n.a_scale; ...
      curr / n.i_scale; ...
      dcurr / n.di_scale; ...
      (temp - n.t_ref) / n.t_scale];

% ---- inference robustness: clamp normalized inputs ----
cl = n.input_clamp;
u = min(max(u, -cl), cl);

% ---- force from state at start of step ----
F = nlcsnn_predict_force(net, x, v, a, curr, temp, h);

% ---- RK4 substeps at 1 ms ----
n_sub = max(1, round(dt / 0.001));
dts = dt / n_sub;
for k = 1:n_sub
    k1 = state_derivative(net, u, h);
    k2 = state_derivative(net, u, h + 0.5 * dts * k1);
    k3 = state_derivative(net, u, h + 0.5 * dts * k2);
    k4 = state_derivative(net, u, h + dts * k3);
    h = h + (dts / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

h_new = h;
end


function nfl = build_nfl(u, h)
% 63-dim physics NFL (physics_nfl=true variant), column vector.
x  = u(1); v = u(2); a = u(3);
ci = u(4);   % normalized current (di/dt not used in this NFL variant)

v_abs = abs(v);
v_pos = max(v, 0);
v_neg = min(v, 0);
sgn_v = tanh(50 * v);

laminar     = [v; v_pos; v_neg];
turbulent   = [v_abs*v; v_pos*v; v_neg*v_abs];
current_mod = [v*ci; v*ci*ci; v_abs*v*ci; v_abs*v*ci*ci];
inertia     = [a; a*v];
gas         = [x; x*x; x*x*x];
friction    = [sgn_v; ci*sgn_v];
valve       = [tanh(5*v_pos); tanh(20*v_pos); ...
               tanh(5*v_neg); tanh(20*v_neg); ...
               ci*tanh(10*v_pos); ci*tanh(10*v_neg)];
state       = [h; h*v; h*v_abs; h*ci; h*a];

nfl = [laminar; turbulent; current_mod; inertia; gas; friction; valve; state];
end


function g = gelu(z)
% Exact GELU (matches PyTorch nn.GELU default): 0.5*z*(1+erf(z/sqrt(2)))
g = 0.5 * z .* (1 + erf(z / sqrt(2)));
end


function dh = state_derivative(net, u, h)
nfl = build_nfl(u, h);
z = gelu(net.Wx1 * nfl + net.bx1);
z = gelu(net.Wx2 * z + net.bx2);
dh = net.Wx3 * z;   % no bias on last layer
end

