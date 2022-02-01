# Changelog
All notable changes to this project will be documented in this file.

The format of this document is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Added
- New `RelativeJacobian` block that implements `KinDynComputations::getRelativeJacobian(.)` of `iDynTree` (https://github.com/robotology/wb-toolbox/issues/225).

## [5.4.1] - 2021-05-28

### Changed
- The `WBToolboxLibrary.slx` file is now encoded in R2016b slx format, so at least MATLAB R2016b is required to use it.

### Fixed 
- Fixed bug that caused Simulink models that used OSQP block to hang indefinitely in "Initializing" or "Compiling" phase (https://github.com/robotology/wb-toolbox/pull/220).
- Disable verbose output option in OSQP block (https://github.com/robotology/wb-toolbox/pull/220).
- Fixed problem introduced in 5.4.0 that prevented the `WholeBodyToolbox` library to appear correctly in the Simulink library browser on some MATLAB versions and on some Operating Systems (spotted on MATLAB R2020b with Ubuntu 20.04) (https://github.com/robotology/wb-toolbox/issues/219, https://github.com/robotology/wb-toolbox/pull/220).

## [5.4.0] - 2021-05-24

### Added
- Add new OSQP block (https://github.com/robotology/wb-toolbox/pull/209).

### Fixed 
- Re-implement the Holder block to make it compatible with code generation and 'treat as atomic unit' (https://github.com/robotology/wb-toolbox/pull/211).

## [5.3] - 2019-02-02

### Changed
- Remove deprecated and not used iDynTree headers (https://github.com/robotology/wb-toolbox/pull/191) 
- Created CMM Simulink Block (https://github.com/robotology/wb-toolbox/pull/194)
- Update citations (https://github.com/robotology/wb-toolbox/pull/195)
- Migrate from Travis CI to Github workflows (https://github.com/robotology/wb-toolbox/pull/196)

## [5.2] - 2020-06-16

### Changed
- Improved compilation time of WBToolbox blocks by optimizing Blockinitialization() (https://github.com/robotology/wb-toolbox/issues/190)

## [5.1] - 2020-04-05

### Fixed
- Fixed calls to `yarp::os::Network` (https://github.com/robotology/wb-toolbox/issues/171)

### Removed
- Remove xenial ci (https://github.com/robotology/wb-toolbox/pull/185)
- Remove deprecated YCM calls (https://github.com/robotology/wb-toolbox/pull/184)

## [5] - 2019-02-02

The `v5` release of Whole-Body Toolbox brings a big change in the architecture: the core of the toolbox became a standalone repository hosted at [robotology/blockfactory](https://github.com/robotology/blockfactory). This repository will only provide a BlockFactory plugin.

More in detail, starting with this release `WBT` will contain only the following components:

- `WBToolboxBase` provides the base classes that simplify the interfacing with iDynTree and YARP libraries.
- `WBToolbox` is the real BlockFactory plugin, and it contains all the implementations of the `blockfactory::core::Block` interface.

To migrate a Simulink model from WB-Toolbox v4 to v5, check the [upgrade guide](https://github.com/robotology/wb-toolbox/issues/167).

### Changed

- The Core, Mex and Coder components have been removed
- New dependency: [robotology/blockfactory](https://github.com/robotology/blockfactory)
- If Matlab is found, the Simulink Library that wraps the block classes is installed

## [4.1] - 2019-02-02

### Fixed
- Fixed failing behavior of qpOASES block (https://github.com/robotology/wb-toolbox/issues/158).

## [4] - 2018-08-21

This is the first release for which a changelog was introduced.

### Important Changes

- Yarp >= `3.0.0` is now a required dependency.
- The toolbox style is now defined by a [`.clang-format`](https://github.com/robotology/wb-toolbox/.clang-format) file.
  PRs must be compliant to this style in order to be merged.
- The `master` branch is protected. Only repo admins can directly push to `master` (even if this is discouraged).
- CMake 3.5 is now required.
- The toolbox is now split in three components: `WBToolboxLibrary`, `WBToolboxMex`, `WBToolboxCoder`.
- Doxygen documentation greatly improved.
- The _Yarp - DoFs Converter_ block has been removed in favour of custom user logic.

### Added

- First version that support Simulink Coder for generating C++ code from a model.
- Most of the classes now use the pimpl idiom.
- Headers have been cleaned with IWYU.
- New _SetMotorParameters_ block. It substitutes the _SetLowLevelPID_ allowing to set also motor constant and back emf constant.
- New _RelativeTransformation_ block for computing generic homogeneous transform between two frames of the model.
- New _GetMotorMeasurements_ block for retrieving position, velocity, acceleration, PWM, and current from the motors.
- New _SetMotorReferences_ block for actuating current and PWM references.
- New _QP_ block implemented with the new `QpOases` class.

#### `WBToolboxLibrary`

- `BlockInformation` interface has been polished. Now it uses enums and const arguments.
- `BlockInformation` interface has new methods for handling parameters.
- `BlockInformation` does not contain anymore any reference to external dependencies (e.g. Yarp and iDynTree).
- `BlockInformation` returns const input signals.
- Implemented new logic for parsing parameters. Added `ConvertStdVector` helper and `Parameter` `Parameters` classes.
- `Log` switched to a `stringstream` implementation, and the verbosity automatically changes between `Debug` and `Release`.
- All the templates have been moved in the `cpp` file, and the allowed instantiations are now explicitly specialized or declared in the header.
- Improved the logic to compute the number of block's parameters.
- Changed many enums names and their scope for improving readability. Switched to `enum class`.
- Improved checks on Signal validity before their usage in all blocks.
- The singleton has been renamed to `WholeBodySingleton`.
- Deprecated the retain / release logic for the `RemoteControlBoardRemapper`. It was substituted by a new logic that exploits smart pointers.
- Removed `ModelPartitioner` class and all utility methods used by it.
- The _SetReferences_ block in Velocity control mode now supports setting the reference acceleration used by the internal trajectory generator.

#### `WBToolboxMex`

- The S-Function was renames from `toolbox.cpp` to `WBToolbox.cpp`.
- Now also the `BlockInformation` object is stored in the PWork instead of being created every time.
- Checks on pointers gathered from the PWork have been improved.
- Constraints on blocks sizes have been released. Now if a Block does not set input / output port sizes, they are inherited from the signal propagation of the entire model.

#### `WBToolboxCoder`

- First version of `CoderBlockInformation` class.
- First version of `WBToolbox.tlc` file for inlining the S-Function.
- First version of the C++ wrapper for executing the class autogenerated by Simulink Coder.

### Fixed

#### `WBToolboxLibrary`

- Fixed the `Position` control mode of the _SetReferences_ block. It now streams new references only when they change.
- Aligned the unit of measurement of the `Reference Speed` used in the `Position` control mode of the _SetReferences_ block.
- Restore the default `Reference Speed` when _SetReferences_ terminates.
- Fixed bug which allowed modifying Signal's buffer content from an const object.

#### `WBToolboxMex`

#### `WBToolboxCoder`

### Changed

#### CMake

- Matlab is not anymore a required dependency. If Matlab is not found, the S-Function, `SimulinkBlockInformation`, and `MxAnyType` are not compiled. However, the `WBToolboxLibrary` and the `WBToolboxCoder` targets build succesfully.

#### Other

- Added Travis CI tests for Linux and macOS on gcc and clang. They now build matlab-independent targets.
- Added Appveyor tests for Visual studio 2015 and 2017 for matlab-independent targets.
- `MxAnyType` is now a shared library.
- Added the `clock_rpc.thrift` file. `ClockServer` sources can now be easily generated again during project configuration by enabling `ALLOW_IDL_GENERATION`.
- All files now include the license header.


### [3] - 2018-02-22

See https://github.com/robotology/community/discussions/279 .
