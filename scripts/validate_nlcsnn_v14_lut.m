function metrics = validate_nlcsnn_v14_lut(lutFile, validationFile)
%VALIDATE_NLCSNN_V14_LUT Held-out inverse error against v14 force curves.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'nlcsnn'));
if nargin < 1 || isempty(lutFile)
    lutFile = fullfile(root,'nlcsnn','v14_actuator_lut.mat');
end
if nargin < 2 || isempty(validationFile)
    validationFile = fullfile(root,'nlcsnn','v14_lut_validation.mat');
end
L = load(lutFile,'lut');
V = load(validationFile);
n = numel(V.target_force_N);
error_N = zeros(n,1);
relative_error = zeros(n,1);
outside = false(n,1);
saturated = false(n,1);
unreachable = false(n,1);
dynamic_mismatch = zeros(n,1);
active = false(n,1);
axis = double(V.current_axis_A(:));
for k = 1:n
    x = repmat(V.x_mm(k),4,1);
    v = repmat(V.v_mmps(k),4,1);
    a = repmat(V.a_mmps2(k),4,1);
    requested = repmat(V.target_force_N(k),4,1);
    previous = repmat(V.actual_current_A(k),4,1);
    bounds = nlcsnn_v14_force_bounds_lut(L.lut,x,v,a);
    [i_cmd,~] = nlcsnn_v14_inverse_lut(L.lut,x,v,a,requested,previous);
    reference_curve = double(V.true_force_curve_N(k,:));
    actual = interp1(axis,reference_curve,double(i_cmd(1)),'linear');
    span = max(reference_curve)-min(reference_curve);
    error_N(k) = abs(actual-double(V.target_force_N(k)));
    relative_error(k) = error_N(k)/max(span,10);
    dynamic_mismatch(k) = abs(double(V.full_model_force_N(k))-double(V.target_force_N(k)));
    active(k) = span >= 100;
    outside(k) = bounds.out_of_domain(1);
    saturated(k) = i_cmd(1)<=single(0.001) || i_cmd(1)>=single(1.599);
    unreachable(k) = double(V.target_force_N(k)) < min(reference_curve) || ...
        double(V.target_force_N(k)) > max(reference_curve);
end
metrics = struct('count',n, 'heldout_file_count',double(V.heldout_file_count), ...
    'median_N',median(error_N), 'p95_N',pctl(error_N,0.95), ...
    'max_N',max(error_N), 'p95_authority_fraction',pctl(relative_error,0.95), ...
    'out_of_domain_fraction',mean(outside), ...
    'saturation_fraction',mean(saturated), ...
    'unreachable_fraction',mean(unreachable), ...
    'dynamic_mismatch_p95_N',pctl(dynamic_mismatch,0.95), ...
    'active_fraction',mean(active), ...
    'active_p95_N',pctl(error_N(active),0.95), ...
    'active_p95_authority_fraction',pctl(relative_error(active),0.95));
fprintf('v14 LUT held-out: %d points / %d files | median %.1f N | p95 %.1f N | max %.1f N\n', ...
    n,metrics.heldout_file_count,metrics.median_N,metrics.p95_N,metrics.max_N);
fprintf('p95/local authority %.3f | outside %.1f%% | saturation %.1f%% | target outside true curve %.1f%%\n', ...
    metrics.p95_authority_fraction,100*metrics.out_of_domain_fraction, ...
    100*metrics.saturation_fraction,100*metrics.unreachable_fraction);
fprintf('active span >=100 N: %.1f%%, p95 %.1f N / %.3f authority | full-history mismatch p95 %.1f N\n', ...
    100*metrics.active_fraction,metrics.active_p95_N, ...
    metrics.active_p95_authority_fraction,metrics.dynamic_mismatch_p95_N);
end

function value = pctl(x,p)
ordered = sort(x(:));
value = ordered(max(1,min(numel(ordered),ceil(p*numel(ordered)))));
end
