# Notes on issues
When filing new issues please use the `create_issue.sh` script (https://github.com/robotology-playground/WBI-Toolbox/blob/master/create_issue.sh). The script will open a new page in your preferred browser with some important information for us to debug the problem.
To make this script executable, move to `$CODYCO_SUPERBUILD_ROOT/main/WBIToolbox/` and type `chmod +x create_issue.sh`.

# Notes on the Toolbox library
In order to properly modify the Toolbox remember the procedure is as follows:
- Point to `$CODYCO_SUPERBUILD_ROOT/main/WBIToolbox/libraries.
- Open from Matlab the file `WBCLibrary.mdl/slx`.
- Unlock the library by doing `Diagram > Unlock Library`.
- If you want to create a new block, the best way is to copy and paste an existing one in the desired sublibrary.
- We now need to break the link  between this copied block and its parent by doing: `Diagram > Disable Link` followed by `Diagram > Break Link`. At this point the block is unlicked from its parent and all the modifications for the new block can be done.
- Save your modifications.
- Export the library to `Matlab R2012a` with the `mdl` extension and save the `slx` version for `Matlab R2014a+`.
- Commit your changes with a meaningful commit message.

# When creating new controllers or Simulink models
As of February 2015, all new controllers and models must be committed in the new repository:
https://github.com/robotology-playground/WBI-Toolbox-controllers


