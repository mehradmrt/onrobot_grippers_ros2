# onrobot

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS drivers for OnRobot Grippers.
This repository is an extension of [Osaka University Harada Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot) and [BrettRD/onrobot_grippers](https://github.com/BrettRD/onrobot_grippers)

## Features

- ROS2 (Foxy & Humble)
- Controller for OnRobot RG2 / RG6 via Modbus/TCP

## Dependency

- pymodbus==2.5.3  

## Installation

```
cd ros2_ws/src
git clone https://github.com/mehradmrt/onrobot_grippers_ros2
cd ../
colcon build
```

## Usage

1. Connect the cable between Compute Box and Tool Changer
2. Connect an ethernet cable between Compute Box and your computer
3. Execute programs (Please refer to [onrobot/Tutorials](http://wiki.ros.org/onrobot/Tutorials))
4. Adjust the ip address parameter based on your computer box ip address. Currently is set to 192.168.0.15 @ 502

### RG2 / RG6

#### Send motion commands
##### Interactive mode
```
ros2 launch onrobot2_rg_control bringup.launch.py
ros2 run onrobot2_rg_control OnRobotRGSimpleController.py
```

##### ROS service call
```

```

##### Display models on Rviz
```
ros2 launch onrobot_rg_description rg2.launch.py
```
##  Original author

[Takuya Kiyokawa](https://github.com/takuya-ki)


##  Other contributors

[Roberto Mendivil C](https://github.com/Robertomendivil97)
[BrettRD/onrobot_grippers](https://github.com/BrettRD/onrobot_grippers)

##  Current contributor

[Mehrad Mortazavi](https://github.com/mehradmrt/onrobot_grippers_ros2)  

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
