# Whole-Body Toolbox
## A Simulink Toolbox for Whole-Body Control

[![Build Status (master)](https://img.shields.io/travis/robotology/wb-toolbox/master.svg?logo=travis&label=master)](https://travis-ci.org/robotology/wb-toolbox)
[![Build Status (devel)](https://img.shields.io/travis/robotology/wb-toolbox/devel.svg?logo=travis&label=devel)](https://travis-ci.org/robotology/wb-toolbox)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/1c726331d58b4a1ebfba1c25d15f00ad)](https://www.codacy.com/app/diegoferigo/wb-toolbox?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=robotology/wb-toolbox&amp;utm_campaign=Badge_Grade)

[![License](https://img.shields.io/badge/license-LGPL-19c2d8.svg)](https://github.com/robotology/wb-toolbox/blob/master/LICENSE.LGPL2)
[![GitHub release](https://img.shields.io/github/release/robotology/wb-toolbox.svg)](https://github.com/robotology/wb-toolbox/releases)
[![GitHub issues](https://img.shields.io/github/issues-raw/robotology/wb-toolbox.svg)](https://github.com/robotology/wb-toolbox/issues)
<a href="https://zenhub.com"><img src="https://img.shields.io/badge/Shipping_faster_with-ZenHub-blue.svg?colorB=435198"></a>

!!! quote ""
​    This toolbox allows non-programming experts and researchers approaching _Whole-Body Control_ to more easily develop controllers on either simulated or real `YARP`-based robotic platforms.<br>
​    Develop to deployment time is minimized by exploiting the rich variety of Simulink's toolboxes and its capabilities on rapid prototyping and visual debugging.

`WBT` is based on the dataflow framework [`blockfactory`](https://github.com/robotology/blockfactory). It mainly wraps functionalities of the [`YARP`](https://github.com/robotology/yarp) middleware and the [`iDynTree`](https://github.com/robotology/idyntree) rigid-body dynamics library, providing an interface compatible with dataflow programming. The `WBT` library can be embedded in any C++ framework and run from all the engines supported by [`blockfactory`](https://github.com/robotology/blockfactory), we mainly use and support Simulink by providing a Simulink Library.

## Who uses `WBT`?

`WBT` is used extensively in the controllers stored in @robotology/whole-body-controllers.

This video shows the latest results on the iCub robot achieved in the EU project CoDyCo in which a top level controller implemented with the @robotology/wb-toolbox achieves a running `100 Hz` rate.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/VrPBSSQEr3A/0.jpg)](https://youtu.be/UXU3KSa201o "iCub balancing on one foot via external force control and interacting with humans")


## Citing this work

!!! quote "Romano F., Traversaro S., Pucci D., Nori F."

    **A Whole-Body Software Abstraction layer for Control Design of free-floating Mechanical Systems**

    _Journal of Software Engineering for Robotics, 2017_

??? quote "Bibtex citation"

    ```
    @ARTICLE{RomanoWBI17Journal,
    author={F. Romano and S. Traversaro and D. Pucci and F. Nori},
    journal={Journal of Software Engineering for Robotics},
    title={A Whole-Body Software Abstraction layer for Control Design of free-floating Mechanical Systems},
    year={2017},
    }
    ```
