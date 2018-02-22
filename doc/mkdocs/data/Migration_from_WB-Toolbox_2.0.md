# From WB-Toolbox 2.0 to WB-Toolbox 3.*

Most of the major changes delivered with the `3.0` version of the `WB-Toolbox` don't affect directly the end-user. Under the hood the toolbox had an important polishing, and the small manual intervention required by this new release match the new features which have been developed.

You can read [Release Notes](https://github.com/robotology/WB-Toolbox/issues/65) for a detailed overview. Below are described only the steps required to port Simulink models to this new release.

## New toolbox configuration

The `WB-Toolbox 2.0` was based on top of [`yarpWholeBodyInterface`](https://github.com/robotology/yarp-wholebodyinterface), which configuration was stored in a `yarpWholeBodyInterface.ini` file. This file was retrieved by `ResourceFinder` and its information was then loaded into the toolbox.

### Store the configuration in the Simulink model

`WB-Toolbox 3.0` deprecated the support of `yarpWholeBodyInterface`, and for reducing the complexity and sparsity of the information storage it allows configuring a Simulink model from the model itself.

The new _Configuration_ block allows inserting information such as **Robot Name**, **URDF Name**, **Controlled Joints**, ... directly from the block's mask.

### Load the configuration from the Workspace

Sometimes it might be useful loading the model's configuration directly from the Workspace. For this purpose, a new `WBToolbox.WBToolboxConfig` class has been developed. The _Configuration_ block needs to know only the name of the variable which refers to the object. Its data is then read before the simulation runs.

This snippet of code shows an example of how to initialize a configuration object:

```matlab
# Initialize a config object
WBTConfigRobot = WBToolbox.WBToolboxConfig;

# Insert robot data
WBTConfigRobot.RobotName = 'icubSim';
WBTConfigRobot.UrdfFile = 'model.urdf';
WBTConfigRobot.ControlledJoints = {...
    'torso_pitch','torso_roll','torso_yaw',...
    'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow',...
    'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow'};
WBTConfigRobot.ControlBoardsNames = {'torso','left_arm','right_arm'};
WBTConfigRobot.LocalName = 'WBT';
```

To check if the data has been read correctly, it is displayed as read-only in the block's mask.

Furthermore, a good sign for a valid configuration is the `WBTConfigRobot.ValidConfiguration` property.

### Multi-robot support

The scope of the introduction of the _Configuration_ block goes beyond the need of a simpler toolbox configuration. One of the biggest limitation of the `2.0` version is the support of [controlling only one robot per model](https://github.com/robotology/WB-Toolbox/issues/37).

`WB-Toolbox 3.0` is now capable of reading / sending data from / to multiple robots. Multiple _Configuration_ blocks can be present in the same model attaining to the following rules:

* In the same hierarchical level of a Simulink model, only one _Configuration_
 block should be present. In other words, you should never see in the display more than one _Configuration_ block.
 * _Configuration_ blocks put deeper in the hierarchy (e.g. in a Subsystem) override the previous ones.

 There are a few pitfalls which are worth to be highlighted:

 * It is legit having two Subsystems with different _Configuration_ blocks which point to the same robot. They can have for instance a different joint list and use different control boards. Although, despite reading information never creates problems, sending data to the robot in such scenario can be disastrous. In fact, consider the case these two subsystems share one link, and configure it in two different control modes (e.g. Position and Torque). Sending references to this link causes unpredictable effects.
 * In line of theory it would be possible to have two subsystems in which the first one refers to a Gazebo model and the second one to a real robot. However, this case causes unpredictable behaviour for what concerns the synchronization. In fact, two different blocks for such aim are present in the toolbox: _Simulator Synchronizer_ and _Real Time Syncronizer_. They should be always used exclusively.

## Other manual edits

* All the _Get Estimate_ blocks need to be replaced by the new _Get Measurement_ block.
* All the hardcoded digital filters (e.g. for the joints velocities) have been removed. A new `Discrete Filter` block has been developed, and it should be manually added if the read raw signal (e.g. from the _Get Measurement_ block) requires filtering.
* The `C++` class used by the _DoFs Converter_ changed. All the blocks in the `YARP To WBI` configuration need to be connected again.
* The gravity vector is stored is the `WBToolboxConfig` class. If an alternative value is needed, set it globally directly in the configuration object or scope the block which needs it in a Subsystem with its own _Configuration_ block.
* In order to set the low level PIDs, loading in the Workspace a `WBToolbox.WBTPIDConfig` object should be configured as follows:

```matlab
# Initialize an empty object
pids = WBToolbox.WBTPIDConfig;

# Insert data
pids.addPID(WBToolbox.PID('l_elbow', WBToolbox.PID(1, 1, 0)));
pids.addPID(WBToolbox.PID('l_wrist_pitch', WBToolbox.PID(1.5, 0, 0.1)));
pids.addPID(WBToolbox.PID('r_shoulder_pitch', WBToolbox.PID(0.2, 0, 0)));
pids.addPID(WBToolbox.PID('torso_roll', WBToolbox.PID(0.1, 0.1, 0)));
```

If some of the controlled joints are not specified, the PIDs are kept in their default values.


## Deprecations

* _Inverse Kinematics_ and _Remote Inverse Kinematics_ have been temporary deprecated. They will see a major release in the coming months. If you need them please do not upgrade to the `3.0` version.
* _Set Low Level PID_ block lost the capability of switching between multiple configurations. Since they were stored in an external file, this change is aligned to the simplification process chosen for for the configuration.
