# Whole-Body Toolbox

[![Build Status (master)](https://img.shields.io/travis/robotology/wb-toolbox/master.svg?logo=travis&label=master)](https://travis-ci.org/robotology/wb-toolbox)
[![Build Status (devel)](https://img.shields.io/travis/robotology/wb-toolbox/devel.svg?logo=travis&label=devel)](https://travis-ci.org/robotology/wb-toolbox)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/1c726331d58b4a1ebfba1c25d15f00ad)](https://www.codacy.com/app/diegoferigo/wb-toolbox?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=robotology/wb-toolbox&amp;utm_campaign=Badge_Grade)

[![License](https://img.shields.io/badge/license-LGPL-19c2d8.svg)](https://github.com/robotology/wb-toolbox/blob/master/LICENSE.LGPL2)
[![GitHub release](https://img.shields.io/github/release/robotology/wb-toolbox.svg)](https://github.com/robotology/wb-toolbox/releases)
[![GitHub issues](https://img.shields.io/github/issues-raw/robotology/wb-toolbox.svg)](https://github.com/robotology/wb-toolbox/issues)
<a href="https://zenhub.com"><img src="https://img.shields.io/badge/Shipping_faster_with-ZenHub-blue.svg?colorB=435198"></a>
[![GitPitch](https://gitpitch.com/assets/badge.svg)](https://gitpitch.com/robotology/wb-toolbox/master?p=.presentations/WBToolbox2)

### A Simulink Toolbox for Whole-Body Control

This toolbox allows non-programming experts and researchers approaching _Whole-Body Control_ to more easily develop controllers on either simulated or real `YARP`-based robotic platforms.<br>
Develop to deployment time is minimized by exploiting the rich variety of Simulink's toolboxes and its capabilities on rapid prototyping and visual debugging.

`WBT` is based on the dataflow framework [`blockfactory`](https://github.com/robotology/blockfactory). Visit the [Whole-Body Toolbox Website](https://robotology.github.io/wb-toolbox/) for more information.

## Who uses `WBT`

`WBT` is used extensively in the controllers stored in [robotology/whole-body-controllers](https://github.com/robotology/whole-body-controllers).

This video shows the latest results on the iCub robot achieved in the EU project CoDyCo in which a top level controller implemented with the `WBT` achieves a running `100 Hz` rate.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/VrPBSSQEr3A/0.jpg)](https://youtu.be/UXU3KSa201o "iCub balancing on one foot via external force control and interacting with humans")

### Citing this work

Please cite the following publications if you are using Whole-Body Toolbox for your own research and/or robot controllers:

```
@article{FerigoControllers2020,
	title = {A generic synchronous dataflow architecture to rapidly prototype and deploy robot controllers},
	author = {Ferigo, Diego and Traversaro, Silvio and Romano, Francesco and Pucci, Daniele},
	url = {https://journals.sagepub.com/doi/10.1177/1729881420921625},
	doi = {10.1177/1729881420921625},
	journal = {International Journal of Advanced Robotic Systems},
	year = {2020},
}
```

```
@article{RomanoWBI17Journal,
  title={A Whole-Body Software Abstraction layer for Control Design of free-floating Mechanical Systems},
  author={F. Romano and S. Traversaro and D. Pucci and F. Nori},
  journal={Journal of Software Engineering for Robotics},
  year={2017},
}
```

### Acknowledgments

The development of Whole-Body Toolbox is supported by:

- FP7 EU projects CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics)
- H2020 EU projects AnDy (No. 731540 H2020-ICT-2016-1)
