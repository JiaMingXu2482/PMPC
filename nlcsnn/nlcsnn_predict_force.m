function F = nlcsnn_predict_force(net, x, v, a, curr, temp, h)
%NLCSNN_PREDICT_FORCE Predict force without advancing the hidden state.
% Inputs use the NLCSNN convention: rebound velocity and tensile force are
% positive. Physical units are mm, mm/s, mm/s^2, A, degC, and N.

%#codegen

n = net.norm;
u = [(x - n.x_ref) / n.x_scale; ...
      v / n.v_scale; ...
      a / n.a_scale; ...
      curr / n.i_scale; ...
      0; ...
      (temp - n.t_ref) / n.t_scale];
cl = n.input_clamp;
u = min(max(u, -cl), cl);

nfl = build_nfl(u, h);
z = gelu(net.Wy1 * nfl + net.by1);
z = gelu(net.Wy2 * z + net.by2);
y = net.Wy3 * z + net.by3;
F = y * n.f_scale;
end

function nfl = build_nfl(u, h)
x = u(1);
v = u(2);
a = u(3);
ci = u(4);

v_abs = abs(v);
v_pos = max(v, 0);
v_neg = min(v, 0);
sgn_v = tanh(50 * v);

laminar = [v; v_pos; v_neg];
turbulent = [v_abs*v; v_pos*v; v_neg*v_abs];
current_mod = [v*ci; v*ci*ci; v_abs*v*ci; v_abs*v*ci*ci];
inertia = [a; a*v];
gas = [x; x*x; x*x*x];
friction = [sgn_v; ci*sgn_v];
valve = [tanh(5*v_pos); tanh(20*v_pos); ...
         tanh(5*v_neg); tanh(20*v_neg); ...
         ci*tanh(10*v_pos); ci*tanh(10*v_neg)];
state = [h; h*v; h*v_abs; h*ci; h*a];
nfl = [laminar; turbulent; current_mod; inertia; gas; friction; valve; state];
end

function g = gelu(z)
g = 0.5 * z .* (1 + erf(z / sqrt(2)));
end
