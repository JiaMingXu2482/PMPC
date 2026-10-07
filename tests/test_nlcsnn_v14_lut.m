function tests = test_nlcsnn_v14_lut
% An inverse table must be bounded, continuous and consistent with bounds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'nlcsnn'));
end

function testProducedTableHasValidAxesAndSource(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
path = fullfile(root, 'nlcsnn', 'v14_actuator_lut.mat');
verifyTrue(testCase, isfile(path));
if ~isfile(path), return; end
S = load(path);
lut = S.lut;
verifyTrue(testCase, all(diff(lut.x_axis_mm) > 0));
verifyTrue(testCase, all(diff(lut.v_axis_mmps) > 0));
verifyTrue(testCase, all(diff(lut.a_axis_mmps2) > 0));
verifyTrue(testCase, all(diff(lut.i_axis_A) > 0));
verifyEqual(testCase, char(lut.checkpoint_md5), ...
    '8a3bd0d508429ab2c00083321561561a');
verifyTrue(testCase, all(isfinite(lut.force_N(:))));
V = load(fullfile(root, 'nlcsnn', 'v14_lut_validation.mat'));
verifyTrue(testCase, isfield(V, 'full_model_force_N'));
end

function testSyntheticInverseAndReachableBounds(testCase)
verifyTrue(testCase, exist('nlcsnn_v14_inverse_lut', 'file') == 2);
if exist('nlcsnn_v14_inverse_lut', 'file') ~= 2, return; end
lut = synthetic_lut();
x = [0; 5; 10; 50]; v = zeros(4,1); a = zeros(4,1);
b = nlcsnn_v14_force_bounds_lut(lut,x,v,a);
verifyEqual(testCase, b.lower(:), single([0;5;10;10]), 'AbsTol', 1e-5);
verifyEqual(testCase, b.upper(:), single([160;165;170;170]), 'AbsTol', 1e-5);
verifyTrue(testCase, b.out_of_domain(4));
[i, f] = nlcsnn_v14_inverse_lut(lut,x,v,a, ...
    [80;85;1000;-1000], [0.5;0.5;0.5;0.5]);
verifyEqual(testCase, i(:), single([0.8;0.8;1.6;0]), 'AbsTol', 1e-5);
verifyEqual(testCase, f(:), single([80;85;170;10]), 'AbsTol', 1e-5);
verifyTrue(testCase, all(isfinite(i) & i>=0 & i<=single(1.6)));
end

function testLowAuthorityHoldAndCellContinuity(testCase)
verifyTrue(testCase, exist('nlcsnn_v14_inverse_lut', 'file') == 2);
if exist('nlcsnn_v14_inverse_lut', 'file') ~= 2, return; end
lut = synthetic_lut();
lut.force_N(:) = 42;
[i, f] = nlcsnn_v14_inverse_lut(lut,zeros(4,1),zeros(4,1), ...
    zeros(4,1),zeros(4,1),[0.3;1.2;1.8;-1]);
verifyEqual(testCase, i(:), single([0.3;1.2;1.6;0]), 'AbsTol', 1e-5);
verifyEqual(testCase, f(:), repmat(single(42),4,1), 'AbsTol', 1e-5);
lut = synthetic_lut();
b1 = nlcsnn_v14_force_bounds_lut(lut,[4.999;5.001;0;0],zeros(4,1),zeros(4,1));
verifyLessThan(testCase, abs(b1.upper(1)-b1.upper(2)), 0.01);
end

function testHeldOutValidationReportsPhysicalError(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'scripts'));
verifyTrue(testCase, exist('validate_nlcsnn_v14_lut','file') == 2);
if exist('validate_nlcsnn_v14_lut','file') ~= 2, return; end
m = validate_nlcsnn_v14_lut();
verifyGreaterThan(testCase, m.count, 0);
verifyTrue(testCase, all(isfinite([m.median_N,m.p95_N,m.max_N, ...
    m.p95_authority_fraction,m.out_of_domain_fraction,m.saturation_fraction])));
end

function lut = synthetic_lut()
lut.x_axis_mm = single([0;10]);
lut.v_axis_mmps = single([-1;1]);
lut.a_axis_mmps2 = single([-1;1]);
lut.i_axis_A = single([0;0.8;1.6]);
lut.force_N = zeros(2,2,2,3,'single');
for ix = 1:2
    for ii = 1:3
        lut.force_N(ix,:,:,ii) = lut.x_axis_mm(ix) + 100*lut.i_axis_A(ii);
    end
end
lut.authority_tol_N = single(1);
end
