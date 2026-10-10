function tests = test_nlcsnn_v14_replacement
tests = functiontests(localfunctions);
end

function testProtectedForceAndInverseShareEnvelope(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'nlcsnn'),fullfile(root,'controller'));
net = nlcsnn_damper_init(fullfile(root,'nlcsnn','v14_weights.mat'));
net.force_guard_enabled=true;
net.force_guard_c_min=0.8;
net.force_guard_v_eps=5;
net.force_guard_power_eps=50;
cfg=struct('x_ref',281.0645,'temp',42.5,'i_max',1.6, ...
    'dt_plant',0.001,'tau_accel',0.04);
h=zeros(227,4,'single');
v=[0;10;-30;120];
x=cfg.x_ref*ones(4,1); a=zeros(4,1);
ctx=func_NLCSNNContext(net,x,v,a,h,cfg);
[I,Fpred]=func_NLCSNNApply(net,ctx,ctx.F_center,0.8*ones(4,1),cfg);
order=[1 3 2 4];
[F,~,~,~,~,~,~,~,Fraw]=func_NLCSNNPlant4(net, ...
    zeros(4,1),-v(order),I,h,a,v,1,cfg);
verifyEqual(testCase,F(order),Fpred,'AbsTol',1e-3);
verifyGreaterThanOrEqual(testCase,-Fpred.*v,0.8*v.^2-1e-6);
verifyEqual(testCase,F(1),0,'AbsTol',1e-12);
verifyGreaterThan(testCase,abs(Fraw(1)),1);
end

function testEquivalentForceHoldsPreviousCurrent(testCase)
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'nlcsnn'));
net=nlcsnn_damper_init(fullfile(root,'nlcsnn','v14_weights.mat'));
h=zeros(227,1,'single'); ip=.73;
fp=nlcsnn_v14_force_at_current(net,281.0645,0,0,ip,42.5,h);
grid=zeros(9,1);
for k=1:9
    grid(k)=nlcsnn_v14_force_at_current(net,281.0645,0,0,(k-1)*.2,42.5,h);
end
[I,F]=nlcsnn_v14_inverse(net,281.0645,0,0,fp+.1,42.5,h,1.6,ip,grid);
verifyEqual(testCase,I,ip);
verifyEqual(testCase,F,fp);
end

function testFullTwinInverseAtReachableMidpoint(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'nlcsnn'),fullfile(root,'controller'));
net = nlcsnn_damper_init(fullfile(root,'nlcsnn','v14_weights.mat'));
cfg = struct('x_ref',281.0645,'temp',42.5,'i_max',1.6);
h = zeros(double(net.state_length),4,'single');
x = [281;286;274;280]; v = [0;120;-180;55]; a = zeros(4,1);
for k=1:20
    for j=1:4
        [~,h(:,j)] = nlcsnn_damper_step(net,x(j),v(j),a(j), ...
            0.2+0.3*j,0,cfg.temp,0.001,h(:,j));
    end
end
before = h;
ctx = func_NLCSNNContext(net,x,v,a,h,cfg);
Fdes = ctx.F_center;
[I,Fpred] = func_NLCSNNApply(net,ctx,Fdes,0.8*ones(4,1),cfg);
verifyEqual(testCase,h,before);
verifySize(testCase,ctx.Fgrid,[9 4]);
verifyGreaterThanOrEqual(testCase,I,zeros(4,1));
verifyLessThanOrEqual(testCase,I,cfg.i_max*ones(4,1));
verifyLessThan(testCase,max(abs(Fpred-Fdes)),2);
end

function testFourCornerPlantHasV14StateAndRawForce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'nlcsnn'),fullfile(root,'controller'));
net = nlcsnn_damper_init(fullfile(root,'nlcsnn','v14_weights.mat'));
cfg = struct('x_ref',281.0645,'temp',42.5,'i_max',1.6, ...
    'dt_plant',0.001,'tau_accel',0.04);
h = zeros(227,4,'single');
[F,hnew,~,~,~,xp,vp,ap] = func_NLCSNNPlant4(net, ...
    zeros(4,1),zeros(4,1),0.5*ones(4,1),h, ...
    zeros(4,1),zeros(4,1),0,cfg);
verifySize(testCase,hnew,[227 4]);
verifyTrue(testCase,all(isfinite(F)));
verifyTrue(testCase,all(isfinite(hnew(:))));
reference = -nlcsnn_v14_force_at_current(net,xp(1),vp(1),ap(1), ...
    0.5,cfg.temp,h(:,1));
verifyEqual(testCase,F(1),reference,'AbsTol',1e-3);
end
