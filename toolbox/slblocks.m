function blkStruct = slblocks

%  Copyright (C) 2013 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
%  Author: Jorhabib Eljaik Gomez
%  email: jorhabib.eljaik@iit.it
%  The development of this software was supported by the FP7 EU project
%  CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
%  http://www.codyco.eu
%  Permission is granted to copy, distribute, and/or modify this program
%  under the terms of the GNU General Public License, version 2 or any
%  later version published by the Free Software Foundation.
%  
%  This program is distributed in the hope that it will be useful, but
%  WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details
%  Copyright (C) 2013 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
%  Author: Jorhabib Eljaik Gomez
%  email: jorhabib.eljaik@iit.it
%  
%  The development of this software was supported by the FP7 EU project
%  CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
%  http://www.codyco.eu
%  
%  Permission is granted to copy, distribute, and/or modify this program
%  under the terms of the GNU General Public License, version 2 or any
%  later version published by the Free Software Foundation.
%  This program is distributed in the hope that it will be useful, but
%  WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details

Browser.Library   = 'WBToolbox2Library';     %Name of the .mdl file
Browser.Name      = 'Whole Body Toolbox 2.0 (Legacy)';
Browser.IsFlat    =  0;

if (~verLessThan('matlab', '8.4'))  % R2014b
  % Add repository information if not yet done
  % ???: is this part really needed?
  try
    load_system(Browser.Library);
    if (strcmp(get_param(Browser.Library, 'EnableLBRepository'), 'off'))
      set_param(Browser.Library, 'Lock', 'off');
      set_param(Browser.Library, 'EnableLBRepository', 'on');
      set_param(Browser.Library, 'Lock', 'on');
      save_system(Browser.Library);
    end;
    close_system(Browser.Library);
  catch ex;
  end
end;


blkStruct.Browser =  Browser;
