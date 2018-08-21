

# Whole-Body Toolbox
###A Simulink Toolbox for Whole-Body Control

#VSLIDE

### Goals of this Toolbox

- Whole-Body Toolbox (WB-Toolbox) is a Simulink Toolbox to develop whole-body controllers
- Allow rapid prototyping by leveraging the power of Matlab/Simulink:
    - Matlab syntax (easy for math!) and Visual development (by Simulink)
    - You can use also other toolboxes in the same Simulink model
- Toolbox wrapping the C++ library <span style="font-size: 90%;"> [yarpWholeBodyInterface](https://github.com/robotology/yarp-wholebodyinterface)</span>

#VSLIDE

### Toolbox running the iCub


![Video](https://www.youtube.com/embed/UXU3KSa201o)


#HSLIDE

### Content of the Toolbox

- The toolbox is composed of four main parts
    - Model (kinematics and dynamics)
    - State (obtain information on the robot)
    - Actuators (send references to robot)
    - Utilities

#VSLIDE

### Model

- Kinematics
     - forward, inverse kinematics
     - Jacobians
- Dynamics
     - Mass Matrix, Bias forces

#VSLIDE

### State

Obtain information on robot (e.g. configuration and velocity)

![State](.presentations/WBToolbox2/images/state.png)

##### Note: to run you need a robot to be present (simulator or real)

#VSLIDE

### Actuators

Send references to the robot (e.g. position, torques)

![Actuators](.presentations/WBToolbox2/images/actuators.png)

##### Note: to run you need a robot to be present (simulator or real)

#VSLIDE

### Utilities

- Read and write to YARP ports
- Synchronise to simulator or wall clock 
- ...

![Utilities](.presentations/WBToolbox2/images/utilities.png)


#HSLIDE

### Configuration of the Toolbox

- The toolbox needs some variables to be defined.
- Most important: `YARP_ROBOT_NAME` and `YARP_DATA_DIRS` environment variables need to be set
- All other variables are defaulted based on the `YARP_ROBOT_NAME` variable
- `YARP_DATA_DIRS` specifies where resources should be looked for in the file system

#VSLIDE

### Configuration of the Toolbox

- `WBT_modelName`: name of the simulink model. Used as prefix for opened YARP ports
- `WBT_robotName`: name of the robot YARP ports
- `WBT_wbiFilename`: name of the `WholeBodyInterface` configuration file. Found used `yarp::os::ResourceFinder`
- `WBT_wbiList`: name of the `WholeBodyInterface` joint serialization list


#HSLIDE

### Associated Publication

- If you find this toolbox useful for your research or work, please cite us with the following citation

```TeX
@INPROCEEDINGS{RomanoWBI17, 
author={F. Romano and S. Traversaro and D. Pucci and A. Del Prete and J. Eljaik and F. Nori}, 
booktitle={2017 IEEE 1st International Conference on Robotic Computing}, 
title={A Whole-Body Software Abstraction layer for Control Design of free-floating Mechanical Systems}, 
year={2017}, 
}
```
