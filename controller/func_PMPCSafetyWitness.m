function D = func_PMPCSafetyWitness(P,A_cons,b_cons,x_opt,exitflag,dUc,gc,pc) %#codegen
%FUNC_PMPCSAFETYWITNESS No-slack margins of feasible QP sequences.
% A positive witness proves only that a checked sequence is safe in the
% current prediction model; a negative witness is not a feasibility proof.
% m_witness covers road and LTR; m_speed uses only controlled LTR rows.
D = struct('m_plan',NaN,'m_candidate',NaN,'m_witness',NaN, ...
    'm_speed',NaN, ...
    'plan_valid',false,'candidate_valid',false,'unknown',true);
n1 = P.Nu*P.Nc;
nIn = 2*P.Nu*P.Nc;
nSh = 4*P.Ncons_sh;
nFirst = 2*P.Ncons_r+4*P.Ncons_env;
first = nIn+nSh+1:nIn+nSh+nFirst;
% Only the controlled LTR rows set a longitudinal speed limit. Road rows
% remain in the QP, where steering and differential braking can correct
% tracking without spending four-wheel braking authority.
rollRows = nIn+nSh+(1:2*P.Ncons_r);
nx = n1+P.Ne+P.Nr;
if exitflag ~= 1 || numel(x_opt) ~= nx || numel(dUc) ~= n1 ...
        || size(A_cons,2) ~= nx || size(A_cons,1) < first(end) ...
        || numel(b_cons) < first(end) || numel(pc) < 14 ...
        || any(~isfinite(x_opt)) || any(~isfinite(dUc)) ...
        || any(~isfinite(A_cons(:))) || any(~isfinite(b_cons(:))) ...
        || ~isfinite(gc)
    return;
end

% The QP may buy safety through s1. Remove that credit in a copy only.
xp = x_opt;
xp(n1+1) = 0;
rPlan = A_cons*xp-b_cons;
mSpeedPlan = NaN;
mSpeedCandidate = NaN;
if max(A_cons(1:nIn,:)*x_opt-b_cons(1:nIn)) <= 1e-6
    D.m_plan = -max(rPlan(first));
    D.plan_valid = isfinite(D.m_plan);
    if D.plan_valid, mSpeedPlan = -max(rPlan(rollRows)); end
end

xc = zeros(nx,1);
xc(1:n1) = dUc;
xc(n1+P.Ne+1:end) = 1;
dbIndex = 2;
if isfield(P,'GammaDBIndex') && P.GammaDBIndex >= 1 ...
        && P.GammaDBIndex <= P.Nr
    dbIndex = round(P.GammaDBIndex);
end
xc(n1+P.Ne+dbIndex) = gc;
if isfinite(pc(14)) && pc(14) <= 1e-6 ...
        && max(A_cons(1:nIn,:)*xc-b_cons(1:nIn)) <= 1e-6
    D.m_candidate = -max(A_cons(first,:)*xc-b_cons(first));
    D.candidate_valid = isfinite(D.m_candidate) ...
        && isfinite(pc(1)) && abs(D.m_candidate-pc(1)) <= 1e-4;
    if D.candidate_valid
        mSpeedCandidate = -max(A_cons(rollRows,:)*xc-b_cons(rollRows));
    end
end

if D.plan_valid && D.candidate_valid
    D.m_witness = max(D.m_plan,D.m_candidate);
    D.m_speed = max(mSpeedPlan,mSpeedCandidate);
elseif D.plan_valid
    D.m_witness = D.m_plan;
    D.m_speed = mSpeedPlan;
elseif D.candidate_valid
    D.m_witness = D.m_candidate;
    D.m_speed = mSpeedCandidate;
end
D.unknown = ~(D.plan_valid || D.candidate_valid);
end
