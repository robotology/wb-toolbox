## Migration from WB**I**-Toolbox To WB-Toolbox
Given a simulink model with some WBI-Toolbox blocks inside, the general procedure is to **substitute each block with the corresponding one from WB-Toolbox**. However, there are some things the user should take care while doing this operation. This guide points out the main differences between the two toolbox. For more info about the WBI-Toolbox, please have a look at the [WBI-Toolbox README](https://github.com/robotology-playground/WBI-Toolbox/blob/master/README.md).

This guide follows the WBI and WB Toolbox blocks partition in Simulink library. It is divided in the following sections:
- [Required variables](#required-variables);
- [Utilities](#utilities);
- [wholeBodyActuators](#wholebodyactuators);
- [wholeBodyModel](#wholebodymodel);
- [wholeBodyStates](#wholebodystates);

### Required variables
As explained in the [WB-Toolbox README](https://github.com/robotology/WB-Toolbox#Using the Toolbox), first of all the user should define the following variables:
- WBT_modelName
- WBT_wbiList
- WBT_wbiFilename
- WBT_robotName

In particular, for iCub torque balancing simulator with 23 DOF, define:
- **WBT_modelname = 'matlabTorqueBalancing'**
- **WBT_wbiList   = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP'**

#### Floating base position estimate
**In the WB-Toolbox the world-to-base homogeneous transformation matrix is not calculated inside each block, but it must be provided from the Simulink model as a block input.**

The world-to-base homogeneous transformation matrix is a 4x4 matrix that maps position and orientation of a rigid body from an initial frame of reference to another.

It can be obtained using forward kinematics block as in the following example: 

![](https://cloud.githubusercontent.com/assets/12396934/12293044/3337da3c-b9f1-11e5-959f-b40418b5469d.png)

Where forward kinematics is used to calculate the transformation matrices from world to the left foot and from world to the root link, while the desired transformation matrix is obtained as a matrices product, using the homogeneous transformation matrix properties.

### Utilities
In this section the user should note that:
- the `Minimum Jerk Trajectory Generator` block has now only the reference trajectory as input.
- `Yarp Read` and some other blocks require now the **WBT_modelName** instead of the **localName** variable.

### wholeBodyActuators
Instead of different blocks for each output, only one generic block is used. The user can choose its output by double-clicking on it and selecting joint position, velocity or joint torques.

### wholeBodyModel
It is divided into three subsections. The `Joint Limits` block is now moved into **wholeBodyStates** section.

#### dynamics
- the `dJdq` blocks have been moved into **jacobians** subsection;
- for mass matrix, generalized bias forces and centroidal momentum computation is now required to calculate explicitly the world-to-base homogeneous transformation matrix and the base velocity. Furthermore, the base frame pose and velocity and the joint configuration are now separate inputs. 

#### jacobians
There is now only one generic block for jacobians and one for `dJdq` calculation. The link with respect to which the jacobian is calculated is indicated by a frame name, specified in the **URDF model**. As for the dynamics, the base pose and velocity and the joint position and velocity are required as input.

#### kinematics
As for the section **jacobians**, there is now only one generic block for forward kinematics computation. World-to-base homogeneous transformation matrix and joint position are the required input.

### wholeBodyStates
As in the previous section, one generic block is used, from which the user can estimate joint position, velocity and acceleration and joint torques. The `Joint Limits` block is moved into this section.
