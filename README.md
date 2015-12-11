![](http://drive.google.com/uc?export=view&id=0B6zDGh11iY6oc0gtM0lMdDNweWM)
Whole Body Toolbox - A Simulink Toolbox for Whole Body Control
-------------------------------------------------------------
### This toolbox is intended as a replacement (a 2.0 version) of the  [WBI-Toolbox](https://github.com/robotology-playground/WBI-Toolbox).

### Main Goal
> The library should allow non-programming experts or those researchers just getting acquainted with Whole Body Control to more easily deploy controllers either on simulation or a real YARP-based robotic platform, as well as to analyze their performance and take advantage of the innumerable MATLAB and Simulink toolboxes. We like to call it "rapid controller prototyping" after which a proper YARP module should be made for hard real time performance and final deployment.

The following video shows CoDyCo's latest results on iCub in which the top level controller has been implemented with the [WBI-Toolbox](https://github.com/robotology-playground/WBI-Toolbox) running at a 10ms rate!

<p align="center">
<a href="https://www.youtube.com/watch?v=VrPBSSQEr3A
" target="_blank"><img src="http://img.youtube.com/vi/VrPBSSQEr3A/0.jpg" 
alt="iCub balancing on one foot via external force control and interacting with humans" width="480" height="360" border="10" /></a>
</p>


- [Installation](https://github.com/robotology/WB-Toolbox#Installation)
- [Troubleshooting](https://github.com/robotology/WB-Toolbox#Troubleshooting)
- [Using the Toolbox](https://github.com/robotology/WB-Toolbox#Using the Toolbox)
- [Citing this work](https://github.com/robotology/WB-Toolbox#citing-this-work)

### Installation

#### Requirements
* Matlab V. 7.1+ and Simulink (Tested with Matlab R2015b, R2014a/b, R2013a/b, R2012a/b)
* Simulink Toolboxes: Simulink Coder.
* YARP (https://github.com/robotology/yarp) **-IMPORTANT-** Please compile as shared library. Currently a default yarp configuration option.
* (Optional. Needed for some blocks) iCub (https://github.com/robotology/icub-main)
* yarpWholeBodyInterface (https://github.com/robotology/yarp-wholebodyinterface)

**Note:** You can install the dependencies either manually or by using the [codyco-superbuild](https://github.com/robotology/codyco-superbuild).

###### Optional
* Gazebo Simulator (http://gazebosim.org/)
* gazebo_yarp_plugins (https://github.com/robotology/gazebo_yarp_plugins).

**Operating Systems supported: OS X, Linux, Windows.**

**Note: The following instructions are for *NIX systems, but it works similarly on the other operating systems.**

#### Compiling the C++ Code (Mex File)

**Prerequisite: Check the Matlab configuration.** Before going ahead with the compilation of the library, make sure that you have MATLAB and Simulink properly installed and running. Then, check that the MEX compiler for MATLAB is setup and working. For this you can try compiling some of the MATLAB C code examples as described in [http://www.mathworks.com/help/matlab/ref/mex.html#btz1tb5-12]. **If you installed Matlab in a location different from the default one, please set an environmental variable called either `MATLABDIR` or `MATLAB_DIR` with the root of your Matlab installation**, e.g. add a line to your `~/.bashrc` such as: `export MATLAB_DIR=/usr/local/bin/matlab`


If you used the [codyco-superbuild](https://github.com/robotology/codyco-superbuild) you can skip this step and go directly to the Matlab configuration step.
Otherwise follow the following instructions.

- Clone this repository: `git clone https://github.com/robotology/WB-Toolbox.git`
- Create the build directory: `mkdir -p WB-Toolbox/build`
- Move inside the build directory: `cd WB-Toolbox/build`
- Configure the project: `cmake ..`

**Note** We suggest to install the project instead of using it from the build directory. You should thus change the default installation directory by configuring the `CMAKE_INSTALL_PREFIX` variable. You can do it before running the first `cmake` command by calling `cmake .. -DCMAKE_INSTALL_PREFIX=/your/new/path`, or by configuring the project with `ccmake .`

- Compile the project: `make`
- *[Optional/Suggested]* Install the project: `make install`


#### Matlab configuration
In the following section we assume the `install` folder is either the one you specified in the previous step by using `CMAKE_INSTALL_PREFIX` or if you used the `codyco-superbuild`, is `/path/to/superbuild/build_folder/install`.

In order to use the WB-Toolbox in Matlab you have to add the `mex` and `share/WB-Toolbox` to the Matlab path.

```bash
    addpath(['your_install_folder'  /mex])
    addpath(genpath(['your_install_folder'  /share/WB-Toolbox]))
```

You can also use (only once after the installation) the script `startup_wbitoolbox.m` that you can find in the `share/WB-Toolbox` subdirectory of the install folder to properly configure Matlab.
**Note** This script configures Matlab to alwasy add the WB-Toolbox to the path. This assumes Matlab is always launched from the `userpath` folder. 

- **Robots' configuration files** Each robot that can be used through the Toolbox has its own configuration file. WB-Toolbox uses the Yarp [`ResourceFinder`](http://www.yarp.it/yarp_resource_finder_tutorials.html). You should thus follow the related instruction to properly configure your installation (e.g. set the `YARP_DATA_DIRS` variable)

### Troubleshooting
- **Problems finding libraries and libstdc++.** In case Matlab has trouble finding a specific library, a workaround is to launch it preloading the variable `LD_PRELOAD` (or `DYLD_INSERT_LIBRARIES` on Mac OS X) with the full path of the missing library. On Linux you might also have trouble with libstdc++.so since Matlab comes with its own. To use your system's libstdc++ you would need to launch Matlab something like (replace with your system's libstdc++ library):

`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.19   matlab`

You could additionally create an alias to launch Matlab this way:

`alias matlab_codyco="cd ~/Documents/MATLAB && LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.19 matlab"`

- In case you have compiled YARP in a directory different from the system default one and you are not using RPATH, you need to tell to MATLAB the location in which to find the shared libraries for YARP. If you launch MATLAB from command line, this task is already done for you by `bash` (if you edited `.bashrc`). If you launch MATLAB from the UI (e.g. on OS X by double clicking the application icon) you need to further add the variables in `${MATLAB_ROOT}/bin/.matlab7rc.sh` by first doing
```bash
    chmod +w .matlab7rc.sh
```
Then looking for the variable `LDPATH_SUFFIX` and assign to every instance the contents of your `DYLD_LIBRARY_PATH`. Finally do:
```bash
    chmod -w .matlab7rc.sh
```

The error message you get in this case might look something like:
```bash
Library not loaded: libyarpwholeBodyinterface.0.0.1.dylib
Referenced from:
${CODYCO_SUPERBUILD_DIR}/install/mex/robotState.mexmaci64
```

### Using the Toolbox
The Toolbox assumes the following information / variables to be defined:
##### Environment variables
If you launch Matlab from command line, it inherits the configuration from the `.bashrc` or `.bash_profile` file. If you launch Matlab directly from the GUI you should define this variables with the Matlab function `setenv`.

- `YARP_ROBOT_NAME`
- `YARP_DATA_DIRS`

##### Matlab variables

- `WBT_robotName`: if not defined the robot name defined in the `yarpWholeBodyInterface.ini` file is taken.
- `WBT_modelName`: if not defined by default is `WBT_simulink`.
- `WBT_wbiFilename`: the name of the Yarp WholeBodyInterface file, by default `yarpWholeBodyInterface.ini`.
- `WBT_wbiList`: the name of the robot list to be used in the Toolbox. By default `ROBOT_TORQUE_CONTROL_JOINTS`.

**Creating a model**
Before using or creating a new model keep in mind that WB-Toolbox is discrete in principle and your simulation should be discrete as well. By going to `Simulation > Configuration Parameters > Solver` you should change the solver options to `Fixed Step` and use a `discrete (no continuous states)` solver.

To start dragging and dropping blocks from the Toolbox open Simulink and search for `Whole Body Toolbox` in the libraries tree.

### Citing this work
Eljaik J., del Prete, A., Traversaro, S., Randazzo, M., Nori, F.,: Whole Body Interface Toolbox (WBI-T):
A Simulink Wrapper for Robot Whole Body Control. In: ICRA, Workshop on MATLAB/Simulink for Robotics, Education and Research. IEEE (2014). [Slides: http://goo.gl/2NnSrA]

#### Tested OS
OS X, Linux, Windows
