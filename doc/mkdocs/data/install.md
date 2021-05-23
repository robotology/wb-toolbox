# Install

!!! info "Disclaimer"
    `WBT` has been widely tested on `Ubuntu 16:04` and `Ubuntu 18.04` with Matlab `R2017b`. If you face any issue either with your OS or Matlab version, please submit an [Issue](https://github.com/robotology/wb-toolbox/issues).

## Requirements

- [`blockfactory`](https://github.com/robotology/blockfactory)
- [`YARP`](https://github.com/robotology/yarp) compiled as shared library (default behavior)
- [`iDynTree`](https://github.com/robotology/idyntree)
- [`YCM`](https://github.com/robotology/ycm)
- [`Eigen3`](http://eigen.tuxfamily.org)
- Supported Operating Systems: Linux, macOS, Windows

### Development requirements

## Optional requirements

- [`iCub`](https://github.com/robotology/icub-main)
- [Gazebo Simulator](http://gazebosim.org/)
- [`gazebo_yarp_plugins`](https://github.com/robotology/gazebo_yarp_plugins)
- [`qpOASES`](https://github.com/coin-or/qpOASES)
- [`matio`](https://github.com/tbeu/matio)
- [`osqp-eigen`](https://github.com/robotology/osqp-eigen)

## Installation

For a simplified installation procedure, jump to [Install using the `robotology-superbuild`](#install-using-the-robotology-superbuild).

### Dependencies

Install the required and the optional dependencies by following their installation instructions. These instructions need that `blockfactory`, `YARP`, `iDynTree`, `YCM` and `Eigen3` packages can be found by `CMake` using `find_package`.

!!! warning
    If an optional dependency is not found, the classes depending on it are not compiled. However, in the Simulink Library the blocks do not disappear. They will just not work, raising an appropriate error.

### Setup Matlab

Matlab and Simulink are not required to build the `WBT` plugin. However, through [`blockfactory`](https://github.com/robotology/blockfactory) the plugin library shipped in this repository can be loaded inside a Simulink model. `WBT` provides a Simulink Library that exposes all the C++ blocks of the plugin to Simulink, wrapping them in user-friendly masks.

!!! info
    Despite the blocks can be used directly from C++, this usage is not very user friendly. Simulink and alternative visual tools are very convenient solutions to connect blocks together, but nothing prevents using the provided blocks exploiting the `blockfactory` interfaces. From now on, we assume that you want to use Simulink since it is the most common use case.

#### For developers

Developers of `WBT` must have Simulink installed in order to operate on the provided Simulink Library.

Make sure that `CMake` is able to [find your Matlab installation](https://cmake.org/cmake/help/v3.3/module/FindMatlab.html), or manually set the `Matlab_ROOT_DIR` environment variable if needed.

### Download, build and install

If all the dependencies are met, proceed with the following instructions:

!!! example "Commands"

    Substitute to `<install-prefix>` the absolute path where you want to install the project.

    ````tab="GNU / Linux and macOS"
    git clone https://github.com/robotology/wb-toolbox.git
    mkdir -p wb-toolbox/build && cd wb-toolbox/build
    cmake .. -DCMAKE_INSTALL_PREFIX=<install-prefix>
    cmake --build .
    cmake --build . --target install
    ````

    ````tab="Windows"
    git clone https://github.com/robotology/wb-toolbox.git
    mkdir -p wb-toolbox/build && cd wb-toolbox/build
    cmake .. -DCMAKE_INSTALL_PREFIX=<install-prefix>
    cmake --build . --config Release
    cmake --build . --config Release --target install
    ````

## Configuration

### Plugin

In order to use Whole-Body Toolbox, the dynamic loader of the operating system should be able to find the plugin library. Add the folder `<install-prefix>/lib/blockfactory` to the following environment variable depending on the OS:

| GNU / Linux       | macOS               | Windows |
| ----------------- | ------------------- | ------- |
| `LD_LIBRARY_PATH` | `DYLD_LIBRARY_PATH` | `Path`  |


### Matlab

In order to use the `WBT` in Matlab you have to add some folders to the Matlab path.

If you usually launch Matlab from the command line, exporting the following environment variable should be enough:

```bash
export MATLABPATH=<install-prefix>/mex:<install-prefix>/share/WBToolbox:<install-prefix>/share/WBToolbox/images
```

If, instead, you use the desktop launcher, a non-persistent Matlab configuration is the following:

```matlab
addpath(['<install-prefix>' /mex])
addpath(genpath(['<install-prefix>' /share/WBToolbox]))
```

We also provide for the latter scenario a persistent configuration of `WBT`. After the installation, run once the `startup_wbitoolbox.m` script that you can find in the `<install-prefix>/share/WBToolbox` directory. It will place a file `pathdef.m` in your `userpath` that loads the right variables to Matlab's environment. Note that this usage assumes that Matlab is always launched from the [`userpath` folder](https://it.mathworks.com/help/matlab/matlab_env/assign-userpath-as-the-startup-folder-on-unix-or-macintosh.html).

### Environment

Each robot that can be used with `WBT` has its own configuration files. `WBT` uses the `YARP`'s [`ResourceFinder`](http://www.yarp.it/yarp_resource_finder_tutorials.html) for finding files in the file system. You should thus follow the related instructions to properly configure your installation (e.g. setting the `YARP_DATA_DIRS` and `YARP_ROBOT_NAME` variables).

## Install using the `robotology-superbuild`

The @robotology/robotology-superbuild provides an easy way for users to setup an environment by downloading, compiling, installing all the projects together.

Follow the [superbuild installation instructions](https://github.com/robotology/robotology-superbuild/#installation) and enable the `ROBOTOLOGY_ENABLE_DYNAMICS` profile. If `WBT` is not downloaded and built, check that `ROBOTOLOGY_USES_MATLAB` is `ON` and `ROBOTOLOGY_NOT_USE_SIMULINK` is `OFF`.

The configuration of the environment should be straightforward following the [Configure your environment](https://github.com/robotology/robotology-superbuild/#configure-your-environment) and [Matlab](https://github.com/robotology/robotology-superbuild#matlab) sections.
