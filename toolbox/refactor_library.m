% Script for changing the S-Function name of all blocks incuded in a
% library.
%
% This might be useful for handling migration before major releases.
%
% Before proceeding, remember to define the new S-Function name in the
% toolbox using:
%
% #define S_FUNCTION_NAME newSFunctionName


clear

oldSFunctionName = 'WBToolbox';
newSFunctionName = 'WBToolbox2';

libraryPath = '/home/dferigo/git/WB-Toolbox/toolbox/';
libraryName = 'WBToolboxLibrary_repository';

refactoredLibraryName = 'WBToolbox2Library_repository';

%%

fprintf('=> Loading the %s/%s library\n', libraryPath, libraryName);

% Add the path where the library can be found
initialPath = pwd;
cd(libraryPath);
addpath(libraryPath);

% Load the library
load_system(libraryName);

% Get all the blocks
blocks = find_system('WBToolboxLibrary_repository','LookUnderMasks','on','FollowLinks','on');
fprintf('=> The library contains %d blocks\n', length(blocks));

% Unlock the library
set_param(libraryName,'Lock','off');

for i = 1:length(blocks)
    try
        oldFcnNameBlock = get_param(blocks{i}, 'FunctionName');
    catch
        continue;
    end

    fprintf('--> Processing %s block\n', blocks{i});
    if strcmp(oldFcnNameBlock, oldSFunctionName)
        set_param(blocks{i}, 'FunctionName', newSFunctionName);
    end
end

% Allow the library to appear in the Library Browser
fprintf('=> Enable the support of Simulink Library Browser\n');
set_param(libraryName,'EnableLBRepository','on');

% Lock the library
set_param(libraryName,'Lock','on');

% Save the updated library
save_system(libraryName, refactoredLibraryName);
close_system(libraryName, 0);

% Export the updated library
% fprintf('=> Exporting the library to the selected Simulink versions\n');
% load_system(refactoredLibraryName);
% save_system(refactoredLibraryName, 'WBToolbox2', 'ExportToVersion', 'R2012A_MDL');
% save_system(refactoredLibraryName, 'WBToolbox2', 'ExportToVersion', 'R2014B_SLX');
% close_system(refactoredLibraryName, 0);
fprintf('=> For exporting the library to the selected Simulink versions use:\n');
fprintf('   $ make export_libraries\n');

fprintf('=> Success\n');

cd(initialPath);
