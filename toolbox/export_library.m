addpath(genpath('../images'));

fprintf('\nWhole Body toolbox exporting library to multiple versions\n');

if (verLessThan('matlab', '8.4'))
    error('This script should be launched with a MATLAB version >= than 2014b');
    quit;
end

libraryName = 'WBToolbox2Library_repository';

try
  open_system(libraryName,'loadonly');
  fprintf('\nLibrary loaded\n');
  if (strcmp(get_param(libraryName, 'EnableLBRepository'), 'off'))
    set_param(libraryName, 'Lock', 'off');
    set_param(libraryName, 'EnableLBRepository', 'on');
    set_param(libraryName, 'Lock', 'on');
  end;
  fprintf('\nExporting for 2014b\n');
  % This does not completely work: images are not saved.
  % Instad if saved form the GUI it works..
  save_system(libraryName, 'WBToolbox2Library', 'ExportToVersion', 'R2014B_SLX');
  fprintf('\nExporting for 2012a\n');
  save_system(libraryName, 'WBToolbox2Library', 'ExportToVersion', 'R2012A_MDL');
  close_system(libraryName);
catch ex;
end

fprintf('\nDone\n');
exit(0)
