% Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

function [configBlock, WBTConfig] = BlockInitialization(currentBlock, currentSystem)
%BLOCKINITIALIZATION Initialize blocks that read a Config block
%   This function is used to initialize the mask of block that need to
%   gather the WBToolbox configuration from a Config block.

try
    % Get the name of this block from the model's root
    %blockAbsoluteName = gcb;

    % Get the top level name
    %blockNameSplit = strsplit(blockAbsoluteName,'/');
    blockNameSplit = strsplit(currentBlock,'/');
    topLevel = blockNameSplit{1};

    % Get all the "Config" blocks from the top level of the system (name of block under mask matching 'ImConfig')
    blocks_system = find_system(topLevel,'LookUnderMasks','on','FollowLinks','on','Regexp','on','Name','ImConfig');

    % Get the name of the block's subsystem
    %[blockScopeName,~] = fileparts(blockAbsoluteName);
    %blockScopeName = gcs;
    blockScopeName = currentSystem;
catch
    error('[%s::Initialization] Failed to process model''s tree', char(currentBlock))
end

% Look a config block from the block's scope going up in the tree
analyzedScope = blockScopeName;
configBlock = char;
while ~isempty(analyzedScope)
    rule = strcat(strrep(analyzedScope,'/','\/'),'\/\w+\/ImConfig');
    idx = cellfun(@(x) ~isequal(regexp(x,rule),[]), blocks_system);
    if (sum(idx) > 1)
        error('[%s::Initialization] Found more than one configuration', char(currentBlock));
    elseif (sum(idx) == 1)
        configBlock = blocks_system{idx};
        configBlock = fileparts(configBlock);
        break;
    end
    analyzedScope = fileparts(analyzedScope);
end

if isempty(configBlock)
    error('[%s::Initialization] No configuration found', char(currentBlock));
end

configSource = get_param(configBlock,'ConfigSource');

if strcmp(configSource,'Mask')
    % Read the mask
    WBTConfig = WBToolbox.MaskToConfiguration(configBlock);
elseif strcmp(configSource,'Workspace')
    % Get the object name and populate the mask
    configObjectName = get_param(configBlock,'ConfigObject');
    WBToolbox.ConfigurationToMask(configBlock, configObjectName);
    % Read the mask
    WBTConfig = WBToolbox.MaskToConfiguration(configBlock);
else
    error('[%s::Initialization] Read ConfigSource from Config block not recognized', char(currentBlock));
end

end
