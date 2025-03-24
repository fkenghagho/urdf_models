# Tracebot Gripper Description

This repository contains the mesh and urdf files of the Tracebot gripper which can be visualised within Rviz.
![Tracebot screenshot](.res/tracebot_gripper.png)

## Table of Contents

- [Tracebot Description](#tracebot-gripper-description)
  - [Table of Contents](#table-of-contents)
  - [Usage](#usage)
  - [Setup](#setup)
  - [Acknowledgements](#acknowledgements)



## Usage

This repository offers visualisation of the gripper model within rviz using the `display_tracebot_gripper_rviz.launch` file, inside `tracebot_gripper_description`.

The rviz simulation give a basic visualisation of :
- the TraceBot gripper, 
- the cable winder that is intended to be mounted on the UR10e 5th axis (or wrist_2  link),
- the driver box that is intended to be mounted on the 3rd axis of the UR10e (or forearm link).

Firsly ensuring that the workspace is sourced:

```bash
source ~/path/to/tracebot_ws/devel/setup.bash
```

- The rviz visualistation can be run with :

  ```bash
  roslaunch tracebot_gripper_description display_tracebot_gripper_rviz.launch
  ```

## Setup

These packages are known to support ROS Melodic and Noetic, although other distros may be supported as well.

The packages can be built/installed following standard ROS-based procedures.
The examples listed below use [catkin_tools](https://catkin-tools.readthedocs.io) as build tool, although `catkin_make` and `catkin_make_isolated` _should_ be also supported:

- Create a workspace:

  ```bash
  mkdir -p ~/path/to/tracebot_ws/src
  cd ~/path/to/tracebot_ws
  catkin config --extend /opt/ros/"$ROS_DISTRO"
  ```

- Pull the packages into the repository:

  ```bash
  cd ~/path/to/tracebot_ws/src
  git clone https://gitlab.com/tracebot/cea/tracebot_gripper_description.git
  ```

- Then finally build everything:

  ```bash
  cd ~/path/to/tracebot_ws
  catkin build
  ```

## Acknowledgements

![TraceBot logo](https://www.tracebot.eu/files/layout/img/TraceBOT%20Logo%202021%20RGB-01.png)

Supported by TraceBot : [Visit project website][tracebot_website]

This project receives funding from the European Union‘s H2020-EU.2.1.1. INDUSTRIAL LEADERSHIP programme (grant agreement No 101017089).

![eu logo](https://www.tracebot.eu/files/Bilder/Logo_Footer/Logo_EU_Horizon2020.png)

The opinions and arguments expressed reflect only the author‘s view and
reflect in no way the European Commission‘s opinions.
The European Commission is not responsible for any use that may be made
of the information it contains.

[tracebot_website]: https://www.tracebot.eu "Go to website"
