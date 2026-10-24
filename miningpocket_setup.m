function root = miningpocket_setup(includeRecorded)
% Add portable packages and unpack the fixed benchmark.
if nargin < 1, includeRecorded = false; end
root = fileparts(mfilename('fullpath'));
maxNumCompThreads(1); % Same single computational thread policy as the recorded experiments.
addpath(root, fullfile(root,'src'));
if ~isfile(fullfile(root,'benchmark','cases','B100.mat'))
    unzip(fullfile(root,'benchmark','cases.zip'),fullfile(root,'benchmark'));
end
if includeRecorded
    files = dir(fullfile(root,'results','recorded','*.zip'));
    for k = 1:numel(files)
        unzip(fullfile(files(k).folder,files(k).name),fullfile(root,'results','recorded'));
    end
end
out = fullfile(root,'results','reproduced');
if ~isfolder(out), mkdir(out); end
end
