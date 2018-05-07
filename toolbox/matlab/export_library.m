% Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

addpath(genpath('../../images'));

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

  % Export the library
  fprintf('\nExporting for 2014b\n');
  save_system(libraryName, 'WBToolboxLibraryTmp', 'ExportToVersion', 'R2014B_SLX');
  movefile('WBToolboxLibraryTmp.slx', 'library/exported/WBToolboxLibrary.slx');
  % TODO: Check if mdl support is still required
  % fprintf('\nExporting for 2012a\n');
  % save_system(libraryName, 'WBToolboxLibrary', 'ExportToVersion', 'R2012A_MDL');
  % movefile('WBToolboxLibrary.mdl', 'library/exported/WBToolboxLibrary.mdl');

  % Unload the library
  close_system(libraryName);
catch
  error('Failed to export WBToolbox library')
end

fprintf('\nDone\n');
exit(0)
