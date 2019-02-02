% Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

fprintf('\nWhole Body toolbox exporting library to multiple versions\n');

if (verLessThan('matlab', '8.4'))
    error('This script should be launched with a MATLAB version >= than 2014b');
    quit;
end

addpath(genpath('library'));
libraryName = 'WBToolboxLibrary_repository';

try
  % Load the library
  open_system(libraryName,'loadonly');
  fprintf('\nLibrary loaded\n');

  % Enable library capabilities
  if (strcmp(get_param(libraryName, 'EnableLBRepository'), 'off'))
    set_param(libraryName, 'Lock', 'off');
    set_param(libraryName, 'EnableLBRepository', 'on');
    set_param(libraryName, 'Lock', 'on');
  end

  % Export the library. It must be in slx otherwise it will not show up in
  % the Simulink Library browser.
  fprintf('\nExporting for 2014b\n');
  save_system(libraryName, 'WBToolboxLibrary', 'ExportToVersion', 'R2014B_SLX');
  movefile('WBToolboxLibrary.slx', 'library/exported/WBToolboxLibrary.slx');

  % Unload the library
  close_system(libraryName);
catch
  error('Failed to export WBToolbox library')
end

fprintf('\nDone\n');
exit(0)
