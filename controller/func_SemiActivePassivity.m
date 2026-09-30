function Fsafe = func_SemiActivePassivity(Fraw, cmpRD, cfg)
%FUNC_SEMIACTIVEPASSIVITY Smoothly project a damper force into passivity.
%   Fsafe = func_SemiActivePassivity(Fraw, cmpRD, cfg) returns a force in
%   CarSim's compression-positive convention such that
%       Fsafe .* cmpRD >= cfg.c_min .* cmpRD.^2
%   at every corner. Thus the damper can only dissipate energy and never
%   becomes less damped than the physical 1.6 A soft state.
%
%   The projection is smooth at velocity reversals. cfg.c_min
%   [N/(mm/s)] is the fail-safe viscous floor. cfg.v_eps [mm/s]
%   regularizes the zero-speed division and cfg.power_eps [N*mm/s]
%   sets the narrow, continuous transition around the floor.

%#codegen

Fraw = reshape(Fraw, 4, 1);
cmpRD = reshape(cmpRD, 4, 1);
v_eps = max(cfg.v_eps, eps);
p_eps = max(cfg.power_eps, eps);
c_min = max(cfg.c_min, 0);

Fsafe = zeros(4, 1);
for j = 1:4
    Fj = Fraw(j);
    vj = cmpRD(j);
    if ~isfinite(Fj) || ~isfinite(vj)
        continue
    end

    % Project only the force above the physical soft-damper floor.  A
    % plain passivity projection would map an out-of-domain, active neural
    % prediction to zero force.  That leaves the wheel-hop mode effectively
    % undamped and creates the observed pulse train as velocity changes sign.
    Ffloor = c_min * vj;
    Pexcess = (Fj - Ffloor) * vj;
    Ppositive = 0.5 * (Pexcess + ...
        sqrt(Pexcess * Pexcess + p_eps * p_eps));
    Fsafe(j) = Ffloor + Ppositive * vj / (vj * vj + v_eps * v_eps);
end
end
