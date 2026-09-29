function net = nlcsnn_damper_init(matFile)
%NLCSNN_DAMPER_INIT  Load trained NLCSNN damper weights into a struct.
%
%   net = nlcsnn_damper_init() loads 'nlcsnn_weights.mat' sitting next to
%   this file. Pass an explicit path to override:
%       net = nlcsnn_damper_init('C:\path\to\nlcsnn_weights.mat')
%
%   Call once (e.g. in the model InitFcn / pmpc_block.m), then pass `net`
%   as a parameter into nlcsnn_damper_step. Keeps file I/O out of the
%   per-step function so the step itself stays %#codegen compatible.
%
%   Model: legacy_cpu_repro checkpoint (180 epochs)
%     - 6-dim normalized input u = [x, v, a, i, di/dt, temp]
%     - 63-dim physics NFL, hidden 256 (GELU), 8-dim hidden state
%     - RK4 with dt = 1 ms; force output denormalized by f_scale

if nargin < 1 || isempty(matFile)
    thisDir = fileparts(mfilename('fullpath'));
    matFile = fullfile(thisDir, 'nlcsnn_weights.mat');
end

S = load(matFile);
net = struct( ...
    'Wx1', S.Wx1, 'bx1', S.bx1, ...
    'Wx2', S.Wx2, 'bx2', S.bx2, ...
    'Wx3', S.Wx3, ...
    'Wy1', S.Wy1, 'by1', S.by1, ...
    'Wy2', S.Wy2, 'by2', S.by2, ...
    'Wy3', S.Wy3, 'by3', S.by3, ...
    'norm', S.norm);
end
