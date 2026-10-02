function root = startup_pmpc()
%STARTUP_PMPC  Add the PMPC project folders to the MATLAB path.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'config'));
addpath(fullfile(root, 'controller'));
addpath(fullfile(root, 'nlcsnn'));
addpath(fullfile(root, 'data'));
if exist(fullfile(root, 'data', 'generated'), 'dir') == 7
    addpath(fullfile(root, 'data', 'generated'));
end
addpath(fullfile(root, 'scripts'));
addpath(fullfile(root, 'scripts', 'lib'));
end
