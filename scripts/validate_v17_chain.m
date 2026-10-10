function result = validate_v17_chain()
%VALIDATE_V17_CHAIN Check golden port and the deployed plant/inverse map.
root = startup_pmpc();
net = v17_damper_init(fullfile(root,'nlcsnn','v17_pmpc_package','v17_weights.mat'));
net.force_guard_enabled = true;
net.force_guard_c_min = 0.8;
net.force_guard_v_eps = 5;
net.force_guard_power_eps = 50;
cfg = struct('x_ref',net.norm.x_ref,'temp',42.5,'i_max',1.6, ...
    'dt_plant',0.001,'tau_accel',0.04);
h = zeros(217,4);
for t = 1:300
    for j = 1:4
        x = net.norm.x_ref+10*sin(2*pi*3*t/1000+j);
        v = 190*cos(2*pi*3*t/1000+j);
        a = -3600*sin(2*pi*3*t/1000+j);
        [~,h(:,j)] = nlcsnn_v17_step_flat(net,x,v,a,0.4*j,42.5,0.001,h(:,j));
    end
end
x = net.norm.x_ref + [4;-5;7;-8];
v = [150;-200;280;-110];
a = [1000;-1800;2200;-400];
ctx = func_NLCSNNContext(net,x,v,a,h,cfg);
assert(isequal(size(ctx.Fgrid),[9 4]));
targetI = [0.35;0.72;1.16;1.44];
Fdes = zeros(4,1);
for j = 1:4
    Fdes(j) = -nlcsnn_v17_force_at_current(net,x(j),v(j),a(j), ...
        targetI(j),cfg.temp,h(:,j));
end
[i_cmd,F_pred] = func_NLCSNNApply(net,ctx,Fdes,[0.4;0.7;1.1;1.4],cfg);
Ftrial = zeros(4,1);
for j = 1:4
    Ftrial(j) = -nlcsnn_v17_force_at_current(net,x(j),v(j),a(j), ...
        i_cmd(j),cfg.temp,h(:,j));
end
result = struct('force_error_N',abs(Ftrial-Fdes), ...
    'prediction_error_N',abs(Ftrial-F_pred), ...
    'current_A',i_cmd,'bounds_N',[ctx.F_lo ctx.F_hi]);
assert(all(isfinite(i_cmd)) && all(i_cmd>=0 & i_cmd<=1.6));
assert(max(result.prediction_error_N)<1e-7, ...
    'Controller predicted force differs from deployed forward map.');
assert(max(result.force_error_N)<10, ...
    'Deployed inverse failed to recover attainable test force.');
fprintf('v17 guarded inverse: max force error %.3f N, map mismatch %.3g N\n', ...
    max(result.force_error_N),max(result.prediction_error_N));
end
