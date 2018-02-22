# Install

!!! info "Disclaimer"

    `WB-Toolbox` has been widely tested on `Ubuntu 16:04` and Matlab `R2017b`. If you face any issue either with your OS or Matlab version, please submit an [Issue](https://github.com/robotology/WB-Toolbox/issues).

## Requirements

- Matlab 7.1+ and Simulink: tested with Matlab `R2017b`, `R2016b`
- [`YARP`](https://github.com/robotology/yarp) compiled as shared library (default behavior)
- [`iDynTree`](https://github.com/robotology/idyntree)
- Supported Operating Systems: Linux, macOS,  Windows

## Optional requirements

- [`iCub`](https://github.com/robotology/icub-main) (needed for some blocks)
- [Gazebo Simulator](http://gazebosim.org/)
- [`gazebo_yarp_plugins`](https://github.com/robotology/gazebo_yarp_plugins).

## Installation

For a simplified installation procedure, jump to [Install using the `robotology-superbuild`](#install-using-the-robotology-superbuild).


!!! warning
    The following instructions are for Unix-like systems, but they work similarly on other operating systems.

### Dependencies

Install the required and the optional dependencies by following their installation instructions. These instructions need that `YARP` and `iDynTree` packages (and optionally `iCub`) can be found by `CMake` using `find_package`.

### Setup Matlab

Make sure that you have MATLAB and Simulink are properly installed and running.

`CMake` needs to find the Matlab installation folder in order to link the sources against its libraries. Make sure that `CMake` is able to [find your Matlab installation](https://cmake.org/cmake/help/v3.3/module/FindMatlab.html), and set the `Matlab_ROOT_DIR` environment variable if needed.

After this, check that the MEX compiler for MATLAB is properly configured and working. You can try compiling some of the MATLAB C code examples as described in the [mex official documentation](https://www.mathworks.com/help/matlab/ref/mex.html).

### Download, build and install

If all the dependencies are met, proceed with the following instructions:

```sh
git clone https://github.com/robotology/wb-toolbox.git
mkdir -p wb-toolbox/build && cd wb-toolbox/build
cmake .. -DCMAKE_INSTALL_PREFIX=<install-prefix>
cmake --build . --config Release
cmake --build . --config Release --target install
cmake --build . --config Release --target install
```

!!! note
    From refer to your install directory with the variable `<install-prefix>`. Every time you see this variable, you should substitute the absolute install path.

## Configuration

### Matlab

In order to use the `WB-Toolbox` in Matlab you have to add some folders to the Matlab path.

If you usually execute Matlab from the command line, exporting the following environment variable should be enough:

```bash
export MATLABPATH=<install-prefix>/mex:<install-prefix>/share/WB-Toolbox:<install-prefix>/share/WB-Toolbox/images
```

If, instead, you use the desktop launcher, a non-persistent Matlab configuration is the following:

```matlab
addpath(['<install-prefix>' /mex])
addpath(genpath(['<install-prefix>' /share/WB-Toolbox]))
```

We also provide for the latter scenario a persistent configuration of `WB-Toolbox`. After the installation, only once after the installation, run the `startup_wbitoolbox.m` script that you can find in the `<install-prefix>/share/WB-Toolbox` directory. This usage assumes that Matlab is always launched from the [`userpath` folder](https://it.mathworks.com/help/matlab/matlab_env/assign-userpath-as-the-startup-folder-on-unix-or-macintosh.html).

### Environment

Each robot that can be used through `WB-Toolbox` has its own configuration files. `WB-Toolbox` uses the `YARP`'s [`ResourceFinder`](http://www.yarp.it/yarp_resource_finder_tutorials.html) for finding files in the file system. You should thus follow the related instructions to properly configure your installation (e.g. setting the `YARP_DATA_DIRS` and `YARP_ROBOT_NAME` variables).

## Install using the `robotology-superbuild`

The @robotology/robotology-superbuild provides an easy way for users to setup an environment by downloading, compiling, installing all the projects together.

Follow the [superbuild installation instructions](https://github.com/robotology/robotology-superbuild/#installation) and enable the `ROBOTOLOGY_ENABLE_DYNAMICS` profile.

The configuration should be straightforward. Setup your `#!sh $HOME/.bashrc` file sourcing the `setup.sh` script as described in [Configure your environment](https://github.com/robotology/robotology-superbuild/#configure-your-environment).
