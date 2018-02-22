# From WBI-Toolbox to WB-Toolbox 2.*

Given a simulink model with some WBI-Toolbox blocks inside, the general procedure is to **substitute each block with the corresponding one from WB-Toolbox 2.0**. However, there are some things the user should take care while doing this operation. This guide points out the main differences between the two toolboxes. For more information about the WBI-Toolbox, please have a look at the [WBI-Toolbox README](https://github.com/robotology-playground/WBI-Toolbox/blob/master/README.md).

This guide follows the WBI and WB Toolbox blocks partitioning in Simulink library. It is divided in the following sections:

- [Required variables](#required-variables)
- [Utilities](#utilities)
- [wholeBodyActuators](#wholebodyactuators)
- [wholeBodyModel](#wholebodymodel)
- [wholeBodyStates](#wholebodystates)

### Required variables
As explained in the [WB-Toolbox README](https://github.com/robotology/WB-Toolbox#Using the Toolbox), first of all the user should define the following variables:
- WBT_modelName
- WBT_wbiList
- WBT_wbiFilename
- WBT_robotName

They have already meaningful default values. Nevertheless you should take a look at at least the following two variables:
- **WBT_modelname = 'matlabTorqueBalancing'** or be careful that the default name does not conflicts with any other modules or YARP ports
- **WBT_wbiList   = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP'** if you simulate a 23-DoFs iCub robot

#### Floating base position estimate
**In the WB-Toolbox the world-to-base homogeneous transformation matrix is not calculated inside each block, but it must be provided from the Simulink model as a block input.**

The world-to-base homogeneous transformation matrix is a 4x4 matrix that maps position and orientation of a rigid body from an initial frame of reference to another.

For back-compatibility, the transformation happending under the hood in the WBI-Toolbox can be obtained using forward kinematics blocks as in the following example:

![](https://cloud.githubusercontent.com/assets/12396934/12293044/3337da3c-b9f1-11e5-959f-b40418b5469d.png)

where forward kinematics is used to compute the transformation matrices from world to the left foot and from world to the root link, while the desired transformation matrix is obtained as a matrices product, using the homogeneous transformation matrix properties.

### Utilities
In this section the user should note that:
- the `Minimum Jerk Trajectory Generator` block has now only the reference trajectory as input. The initial value is automatically taken at startup.
- `Yarp Read` and some other blocks require now the **WBT_modelName** instead of the **localName** variable.

### wholeBodyActuators
Instead of having different blocks for each kind of control mode, only one block is now present. The user can choose the control mode by double-clicking on it and selecting one of the possible modes (position, position direct, velocity and torques)

### wholeBodyModel
It is divided into three subsections. The `Joint Limits` block is now moved into **wholeBodyStates** section.

#### Dynamics
- the `dJdq` blocks have been moved into **jacobians** subsection;
- for mass matrix, generalized bias forces and centroidal momentum computation is now required to calculate explicitly the world-to-base homogeneous transformation matrix and the base velocity. Furthermore, the base frame pose and velocity and the joint configuration are now separate inputs.

#### Jacobians
There is now only one generic block for jacobians and one for `dJdq` calculation. The link with respect to which the Jacobian is computed is determined by its frame name as specified in the **URDF model**. As for the dynamics, the base pose and velocity and the joint position and velocity are required as input.

#### Kinematics
As for the section **Jacobians**, there is now only one generic block for forward kinematics computation. World-to-base homogeneous transformation matrix and joint position are the required input.

### wholeBodyStates
As in the previous section, one generic block is used, from which the user can estimate joint position, velocity and acceleration and joint torques. The `Joint Limits` block is moved into this section.
