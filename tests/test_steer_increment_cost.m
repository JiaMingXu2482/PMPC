function tests=test_steer_increment_cost
tests=functiontests(localfunctions);
end
function setupOnce(~)
startup_pmpc();
end
function testQuadraticMatchesDirectCost(t)
rng(12); z=randn(18,1); prev=.03; lim=.05; w=5000;
[H,f,c]=func_SteerIncrementCost(3,5,18,lim,prev,w);
d=diff([prev;z(1:3:15)]);
verifyEqual(t,.5*z'*H*z+f'*z+c,w*sum(d.^2)/lim^2,'RelTol',1e-12);
verifyGreaterThanOrEqual(t,min(eig(H)),-1e-6);
verifyEqual(t,H(2:3:end,:),zeros(6,18));
end
function testContinuingExecutedRateHasZeroCost(t)
[H,f,c]=func_SteerIncrementCost(3,5,18,.05,.02,5000);
z=zeros(18,1); z(1:3:15)=.02;
verifyEqual(t,.5*z'*H*z+f'*z+c,0,'AbsTol',1e-9);
end
function testDisabledHasNoEffect(t)
[H,f,c]=func_SteerIncrementCost(3,5,18,.05,.02,0);
verifyEqual(t,H,zeros(18)); verifyEqual(t,f,zeros(18,1)); verifyEqual(t,c,0);
end
