function [ WBTConfig ] = Mask2WBToolboxConfig(configBlock)
%MASK2WBTOOLBOXCONFIG Summary of this function goes here
%   Detailed explanation goes here

defaultString = '''NonValid''';

if (...
    strcmp(char(get_param(configBlock,'RobotName')),defaultString) && ...
    strcmp(char(get_param(configBlock,'UrdfFile')),defaultString) && ...
    strcmp(char(get_param(configBlock,'LocalName')),defaultString) && ...
    strcmp(char(get_param(configBlock,'GravityVector')),defaultString) && ...
    strcmp(char(get_param(configBlock,'ControlledJoints')),defaultString) && ...
    strcmp(char(get_param(configBlock,'ControlBoardsNames')),defaultString) ...
    )
    error('[Mask2WBToolboxConfig] The provided config is NonValid.');
end

WBTConfig = WBToolbox.WBToolboxConfig;

WBTConfig.RobotName = stripApices(char(get_param(configBlock,'RobotName')));
WBTConfig.UrdfFile  = stripApices(char(get_param(configBlock,'UrdfFile')));
WBTConfig.LocalName = stripApices(char(get_param(configBlock,'LocalName')));

ControlledJointsChar = stripApices(char(get_param(configBlock,'ControlledJoints')));
WBTConfig.ControlledJoints = evalin('base',ControlledJointsChar);

ControlBoardsNamesChar = stripApices(char(get_param(configBlock,'ControlBoardsNames')));
WBTConfig.ControlBoardsNames = evalin('base',ControlBoardsNamesChar);

GravityVectorChar = stripApices(char(get_param(configBlock,'GravityVector')));
WBTConfig.GravityVector = evalin('base',GravityVectorChar);

end

function str = stripApices(str)
    if (strcmp(str(1),'''') && strcmp(str(end),''''))
        str = str(2:end-1);
    end
end
