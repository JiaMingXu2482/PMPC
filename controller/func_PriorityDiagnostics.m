function diag = func_PriorityDiagnostics(cert) %#codegen
%FUNC_PRIORITYDIAGNOSTICS  Fixed 12-channel log of the online priority state.
%   [delta1 delta2 F w1 w2 certA certB s1 s2 gammaDB exitflag fallback]
%   cert is the fixed 36x1 pmpc_step state: [prior14; posterior11; slack8;
%   gammaDB; exitflag; fallback]. Keep the 54-channel CarSim sys unchanged.

diag = [cert(1:7); cert(26:27); cert(34:36)];
if numel(cert)==44
    % PMPC channels 13:20: on, valid1, valid2, margin1, margin2,
    % Lyapunov contraction, equilibrium residual, construction status.
    diag=[diag;cert(37:44)];
end
end
