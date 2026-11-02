function [sys, F_des, long_diag] = pmpc_external_variant_block(u, F_lo, F_hi, PMPC_P) %#codegen
%PMPC_EXTERNAL_VARIANT_BLOCK Shared force-bound interface for fixed variants.
if numel(u) ~= 54
    error('pmpc_external_variant_block:InvalidInputSize', ...
        'Expected 54 CarSim export channels.');
end
uc = u(:);
persistent St
if isempty(St)
    St = PMPC_P.S0;
end
[sys, St, F_des] = pmpc_step(uc, PMPC_P.Pm, St, ...
    zeros(217,4), reshape(F_lo,4,1), reshape(F_hi,4,1), zeros(4,1));
long_diag = zeros(20,1);
long_diag(1:12) = func_PriorityDiagnostics(St.cert(1:36));
end
