function tau_d = func_DamperDelayTau(v_d, abs_delta_i, sign_delta_i, tau_fixed)
%FUNC_DAMPERDELAYTAU  Aggregate damper delay for the 7-state predictor.
%   The current deployment uses a measured-literature fixed delay.  The
%   four corner inputs deliberately define the future LUT interface:
%   tau_j = LUT(v_d,j, abs(DeltaI_j), sign(DeltaI_j)).  When that LUT is
%   available, this function must return max(tau_j) because the predictor
%   has one conservative aggregate Md_a state.

if ~isvector(v_d) || numel(v_d) ~= 4 || ...
        ~isvector(abs_delta_i) || numel(abs_delta_i) ~= 4 || ...
        ~isvector(sign_delta_i) || numel(sign_delta_i) ~= 4 || ...
        any(~isfinite(v_d(:))) || any(~isfinite(abs_delta_i(:))) || ...
        any(~isfinite(sign_delta_i(:))) || any(abs_delta_i(:) < 0) || ...
        any(sign_delta_i(:) ~= -1 & sign_delta_i(:) ~= 0 & sign_delta_i(:) ~= 1)
    error('func_DamperDelayTau:InvalidInput', ...
        'v_d, abs_delta_i, and sign_delta_i must be finite 4-element corner vectors.');
end

if ~isscalar(tau_fixed) || ~isfinite(tau_fixed) || tau_fixed <= 0
    error('func_DamperDelayTau:InvalidDelay', ...
        'tau_fixed must be a finite positive scalar.');
end

tau_d = tau_fixed;
end
