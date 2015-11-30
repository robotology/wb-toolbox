![](http://drive.google.com/uc?export=view&id=0B6zDGh11iY6oc0gtM0lMdDNweWM)
Whole Body Toolbox - A Simulink Toolbox for Whole Body Control
-------------------------------------------------------------

**Note** this is intended as a replacement for [WBI-Toolbox](https://github.com/robotology-playground/WBI-Toolbox)

This document contains instructions on how to install and use this toolbox, *tips and tricks* to do so and a walkthrough to get you started using it. Simulink blocks consist of S-functions (http://goo.gl/1GuHVd) which allow C/C++ user specific code compiled as Matlab Executable (MEX) files, thus extending the capabilities of the Simulink environment. In other words, MEX files have been created linking YARP, iCub, **iDynTree** (a more efficient and generic YARP-based robot dynamics library than its predecessor iDyn - http://goo.gl/BnGzKr) and CoDyCo, wrapping the **Whole Body Interface** described in http://goo.gl/dBWO3k. The following video shows CoDyCo's latest results on iCub in which the top level controller has been implemented with the WBI-Toolbox running at a 10ms rate!

- [Main Goal](https://github.com/robotology-playground/WBI-Toolbox#main-goal)
- [Requirements](https://github.com/robotology-playground/WBI-Toolbox#requirements)
- [Compilation](https://github.com/robotology-playground/WBI-Toolbox#compiling-the-toolbox-mex-files)
- [Installation](https://github.com/robotology-playground/WBI-Toolbox#installing-the-wbi-toolbox)
- [Working from the build tree](https://github.com/robotology-playground/WBI-Toolbox#working-from-the-build-tree)
- [Notes on Configuration Files](https://github.com/robotology-playground/WBI-Toolbox#notes-on-configuration-files)
- [Using the Toolbox](https://github.com/robotology-playground/WBI-Toolbox#using-the-toolbox)
- [iCub joints ordering](https://github.com/robotology-playground/WBI-Toolbox#details-on-icub-joints-ordering-in-wbi-toolbox)
- [Existing models](https://github.com/robotology-playground/WBI-Toolbox#existing-models)
- [Citing this work](https://github.com/robotology-playground/WBI-Toolbox/blob/master/README.md#citing-this-work)

<p align="center">
<a href="https://www.youtube.com/watch?v=VrPBSSQEr3A
" target="_blank"><img src="http://img.youtube.com/vi/VrPBSSQEr3A/0.jpg" 
alt="iCub balancing on one foot via external force control and interacting with humans" width="480" height="360" border="10" /></a>
</p>



#### Main Goal
> The library should allow non-programming experts or those researchers just getting acquainted with Whole Body Control to more easily deploy controllers either on simulation or a real YARP-based robotic platform, as well as to analyze their performance and take advantage of the innumerable MATLAB and Simulink toolboxes. We like to call it "rapid controller prototyping" after which a proper YARP module should be made for hard real time performance and final deployment.


#### Requirements
* Matlab V. 7.1+ and Simulink (Tested with Matlab R2014a/b, R2013a/b, R2012a/b)
* Simulink Toolboxes: Simulink Coder.
* YARP (https://github.com/robotology/yarp) **-IMPORTANT-** Please compile as shared library. Currently a default yarp configuration option.
* CoDyCo (https://github.com/robotology-playground/codyco-superbuild)
* iCub (https://github.com/robotology/icub-main)

###### Optional
* Gazebo Simulator (http://gazebosim.org/)
* gazebo_yarp_plugins (https://github.com/robotology/gazebo_yarp_plugins).


**Operating Systems supported: Linux, MAC OS X, Windows.**

**Note: The following instructions are for Linux distributions, but it works similarly on the other operating systems.**

#### Compiling the Toolbox MEX File
The WB-Toolbox can be compiled through the CoDyCo project (https://github.com/robotology-playground/codyco-superbuild). In the following steps assume that `$CODYCO_SUPERBUILD_DIR` points to the `/build` directory of your CoDyCo installation and `$CODYCO_SUPERBUILD_ROOT` to the corresponding root directory. In case you are using the simulator, make sure that the iCub models are being loaded and the `gazebo_yarp_plugins` properly working. This is easy to verify as you need only to launch a `yarpserver` followed by Gazebo and load the desired model, be it iCub (fixed) or iCub. If the robot does not fall under the effect of gravity, it means the plugins are working and you can go ahead with the installation of the Toolbox.

- **Check the Matlab configuration.** Before going ahead with the compilation of the library, make sure that you have MATLAB and Simulink properly installed and running. Then, check that the MEX compiler for MATLAB is setup and working. For this you can try compiling some of the MATLAB C code examples as described in [http://www.mathworks.com/help/matlab/ref/mex.html#btz1tb5-12]. **If you installed Matlab in a location different from the default one, please set an environmental variable called either `MATLABDIR` or `MATLAB_DIR` with the root of your Matlab installation**, e.g. add a line to your `~/.bashrc` such as: `export MATLAB_DIR=/usr/local/bin/matlab`

- **Compiling the WBI Toolbox.** To compile the WBI Toolbox via `codyco-superbuild`, you first need to configure the latter with CMake. A few flags need to be taken into account in order to do this. In particular if you want to use the Gazebo simulator please do:

```bash
   cd $CODYCO_SUPERBUILD_DIR
   cmake ../ -DCODYCO_USES_MATLAB=YES -DCODYCO_USES_WBI_TOOLBOX:BOOL=YES -DCODYCO_USES_URDFDOM:BOOL=YES
```
Then as usual type `c` to configure until no stars (*) show up and `g` to generate. Finally, to compile type `make`.
After this step all the subprojects will be installed inside the `$CODYCO_SUPERBUILD_DIR/install` directory. In order to use use it you will have to adjust some environment variables in your `~/.bashrc`

```bash
export PATH=$PATH:${CODYCO_SUPERBUILD_DIR}/install/bin/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${CODYCO_SUPERBUILD_DIR}/install/lib/
```

**Note: For more information on how to compile or update `codyco-superbuild` go to http://goo.gl/aU6EjH**

#### Installing the WBI-Toolbox
- **Installation.** There are a number of ways to install the Toolbox. They all consist in ensuring that the MEX files you just compiled are found in MATLAB's path, along with the Toolbox itself and its icons. We try to make your life easier and prepared an installation script that can be found under the name `startup_wbitoolbox.m` in `${CODYCO_SUPERBUILD_DIR}/install/share/WBI-Toolbox` which automatically takes into account where you installed the WBIToolbox as specified by the variable `CMAKE_INSTALL_PREFIX`. You can see the default value of this variable by going to `${CODYCO_SUPERBUILD_DIR}/main/WBIToolbox` and typing `ccmake ./` to see the CMake default options for the Toolbox. In this way, after compilation, running `startup_wbitoolbox.m` should automatically add the desired directories to MATLAB's path. By default it updates MATLAB's path by creating a `pathdef.m` file in your user startup folder e.g. `~/Documents/MATLAB` by default. To avoid problems when launching MATLAB from terminal you should **make sure you are in your startup directory**. In case the script fails for permission reasons, it will provide you with further instructions on how to manually solve the problem.

If for some reason the installation fails or you want to do this manually, the directories you need to add to the path are `${CODYCO_SUPERBUILD_DIR}/install/mex` (assuming the default CMake installation directory) and the one for the Toolbox itself, i.e. `${CODYCO_SUPERBUILD_DIR}/install/share/WBI-Toolbox` by doing

```bash
    addpath([getenv(CODYCO_SUPERBUILD_DIR)  /install/mex])
    addpath([getenv(CODYCO_SUPERBUILD_DIR)  /install/share/WBI-Toolbox])
    addpath([getenv(CODYCO_SUPERBUILD_DIR)  /install/share/WBI-Toolbox/images])
```
You can also create a .m file with these two lines and launch MATLAB from terminal as:
```bash
    matlab -r yourStartupFile
```

#### Working from the build tree
If you prefer to work from the build tree (for example you are not using the `codyco-superbuild` and you are not installing the library) you cannot use the provided script as it assumes the install directories.
Instead you have to manually add the following directories to your MATLAB's path
```bash
    addpath([getenv(WBI_TOOLBOX_BUILD_DIR)    /path/to/builded/libraries])
    addpath([getenv(WBI_TOOLBOX_SOURCES_DIR)  /libraries])
    addpath([getenv(WBI_TOOLBOX_SOURCES_DIR)  /libraries/images])
```

- **Robots' configuration files** Each robot that can be used through the Toolbox has its own configuration file. In order for WBI-Toolbox to find them your `YARP_DATA_DIRS` environmental variable **should include your CoDyCo `/share` directory** where CoDyCo contexts can be found. If you locally installed CoDyCo, it should be enough to append the following location:
`$CODYCO_SUPERBUILD_DIR/install/share/codyco` to `YARP_DATA_DIRS` in your bashrc as:

```bash
   export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${CODYCO_SUPERBUILD_DIR}/install/share/codyco
```
- **Problems finding libraries and libstdc++.** In case Matlab has trouble finding a specific library, a workaround is to launch it preloading the variable `LD_PRELOAD` (or `DYLD_INSERT_LIBRARIES` on Mac OS X) with the location of the missing library. On Linux you might also have trouble with libstdc++.so since Matlab comes with its own. To use your system's libstdc++ you would need to launch Matlab something like (replace with your system's libstdc++ library):

`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.19   matlab`

You could additionally create an alias to launch Matlab this way:

`alias matlab_codyco="cd ~/Documents/MATLAB && LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.19 matlab"`

- In case you have compiled YARP in a directory different from the system default one, you need to tell to MATLAB the location in which to find the shared libraries for YARP. If you launch MATLAB from command line, this task is already done for you by `bash` (if you edited `.bashrc`). If you launch MATLAB from the UI (e.g. on OS X by double clicking the application icon) you need to further add the variables in `${MATLAB_ROOT}/bin/.matlab7rc.sh` by first doing
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

#### Notes on configuration files
If you wish to change any of the default values of the toolbox's configuration file you should do it in `${CODYCO_SUPERBUILD_DIR}/install/share/codyco/contexts/wholeBodyInterfaceToolbox` (assuming you left the default installation directory of the WBI Toolbox, otherwise look for the corresponding `contexts` directory). Remember that these configuration files will be overwritten everytime you install the WBIToolbox.

In v0.2 the **source default** configuration file can be found in: https://github.com/robotology-playground/WBI-Toolbox/blob/master/libraries/wbInterface/conf/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini. In the previous version (v0.1) these options were mixed in the robot-specific configuration files installed with `yarpWholeBodyInterface`. Since they are unrelated, we decided to create an independent configuration file. This file is then installed as:

`$CODYCO_SUPERBUILD_DIR/install/share/codyco/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini`

which by default contains the following pairs:

```bash 
robot          icubGazeboSim
localName      simulink
worldRefFrame  l_sole
robot_fixed    true
wbi_id_list    ROBOT_TORQUE_CONTROL_JOINTS
wbi_config_file yarpWholeBodyInterface.ini
```
Where

`robot` indicates whether you are using the real platform (`icub`) or the simulator (`icubGazeboSim`)
`localName` corresponds to the name prefix of the ports that will be opened by the interface.
`worldRefFrame` Indicates which reference frame in the robot structure is used as the world reference frame for all the computations.
`robot_fixed` Tells the interface whether the robot is 'impaled' to a fixed virtual base attached to the origin of the `root` of the robot. (more info: http://eris.liralab.it/wiki/ICubForwardKinematics)
`wbi_id_list` List of parts to use for the robot defined through the environmental variable `YARP_ROBOT_NAME` (e.g. `iCubGenova01`) as defined in: https://github.com/robotology-playground/yarp-wholebodyinterface/tree/master/app/robots.
`wbi_config_file` Name of the configuration file of the `yarpWholeBodyInterface` used by the Toolbox.

**You should change the default options in the installed file** and not the source one.


#### Using the Toolbox
**What robot are you using?**
In the newest version (v0.2) of the toolbox you first need to define the environmental variable `YARP_ROBOT_NAME` in `~/.bashrc` with the actual name of your robot, e.g. `export YARP_ROBOT_NAME="icubGazeboSim"`. If you were to use the real robot, say `iCubGenova03`, then you assign this name to `YARP_ROBOT_NAME`. **Rationale:** Since now `yarpWholeBodyInterface` uses the `ResourceFinder` to find configuration files in your system, one of the data directories where it will search is `robots/$YARP_ROBOT_NAME`. Therefore, after you *install* `yarpWholeBodyInterface`, the default configuration files in it will be copied to `CODYCO_SUPERBUILD_DIR/install/share/codyco/robots` where you will find all the available robots. :warning: **Please note that if you compile `yarpWholeBodyInterface` (by say, doing `make` in `$CODYCO_SUPERBUILD_DIR/libraries/yarpWholeBodyInterface`) with the flag `CODYCO_INSTALL_ALL_ROBOTS=OFF` and no environmental variable `YARP_ROBOT_NAME` defined, you won't have installed any of these configuration files.** 
For more info: http://eris.liralab.it/yarpdoc/resource_finder_spec.html. This allows the WBI-Toolbox to be used not only with `iCubGenova0X`. It has been tested so far with `iCubHeidelberg01` and `COMAN`.

**yarpserver and Gazebo**
First launch `yarpserver` and open `gazebo` with the `iCub` model or any of its variants.

**Creating a model**
Before using or creating a new model keep in mind that WBI-Toolbox is discrete in principle and your simulation should be discrete as well. By going to `Simulation > Configuration Parameters > Solver` you should change the solver options to `Fixed Step` and use a `discrete (no continuous states)` solver.

To start dragging and dropping blocks from the Toolbox open Simulink and search for `Whole Body Interface Toolbox` in the libraries tree.

**Right before running your model**
All blocks **need** three basic parameters in order to start. These are: `robotName`, `localName` and `Ts`. These variables can be set in your Matlab command window or in your personal configuration script.
- `robotName` Corresponds to the real name of the robot, e.g. `iCubGenova01`, `iCubHeidelberg01`, `COMAN` or `icubGazeboSim`. 
- `localName` prefix of the ports that will be opened by the interface.
- `Ts` Thread rate.
- `ROBOT_DOF` (temporary) Number of DOF that should coincide with the number of parts of the list name specified in `wholeBodyInterfaceToolbox.ini` (25 in the case of `icub`/`icubGazeboSim`, while for `iCubHeidelberg01` it would be 15).

In particular, the values for `robotName` and `localName` overwrite those in `wholeBodyInterfaceToolbox.ini`.

**IMPORTANT** For your simulations you might want to drop into your simulink model the `ySynchronizer` block which will synchronize the simulation time with the yarp time, given a specified period `Ts`. For this to work you need to first define the environmental variable `$YARP_CLOCK=/clock`, launch `yarpserver` and finally gazebo as `gazebo -s libgazebo_yarp_clock.so` (remember to update your `gazebo-yarp-plugins` repository with the latest version). The rest works as usual. 

#### Existing models
**All Simulink models and controllers have been moved to https://github.com/robotology-playground/WBI-Toolbox-controllers and removed from WBI-Toolbox**. In order to get them you need to update codyco-superbuild to the most recent version, configure it with CMake activating the flag `CODYCO_USES_WBI_TOOLBOX_CONTROLLERS` (`OFF` by default) and build the whole thing as usual. After doing so, a new directory will show up in `$CODYCO_SUPERBUILD_ROOT/main` called `WBIToolboxControllers` with all the models previously found in the WBI-Toolbox. To use them, just move to this particular directory from within Matlab.

#### Details on iCub joints ordering in WBI Toolbox  
If you want more information about the serialization of the iCub joints used in the WBI-Toolbox, check [this wiki page](https://github.com/robotology-playground/WBI-Toolbox/wiki/Details-on-iCub-joints-ordering-in-WBI-Toolbox).

#### Citing this work
Eljaik J., del Prete, A., Traversaro, S., Randazzo, M., Nori, F.,: Whole Body Interface Toolbox (WBI-T):
A Simulink Wrapper for Robot Whole Body Control. In: ICRA, Workshop on MATLAB/Simulink for Robotics, Education and Research. IEEE (2014). [Slides: http://goo.gl/2NnSrA]

#### Tested OS
Linux, MAC OS X, Windows
