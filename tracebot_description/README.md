# Tracebot Description

This repository contains the stl and urdf files of the robot and enviroment which can be visualised within Rviz.
![Tracebot screenshot](.res/tracebot_rviz_v5.png)

## Table of Contents

- [Tracebot Description](#tracebot-description)
  - [Table of Contents](#table-of-contents)
  - [Model Parameters](#model-parameters)
  - [Usage](#usage)
  - [Setup](#setup)
  - [Object models convention](#object-models-convention)
  - [Camera-robot calibration tool](#camera-robot-calibration-tool)
    - [Basic usage](#basic-usage)
  - [Acknowledgements](#acknowledgements)

## Model Parameters

The URDF model contained in `tracebot_description` is parametrized using xacro.
There are various parameters that can be modified by creating a new setup inside the [`config`](./config) directory.
Currently two setups exist, reproducing the physical setup at [Astech](./config/astech) (default) and [Tecnalia](./config/tecnalia).

The table below lists the available parameters and their meaning.

| Parameter name           | Description                                                                                                      |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------- |
|`robot_name`              | Set the prefix name of the robot, 'tracebot' by default.                                                         |
|`camera`-`tilt`           | The forward tilt position of the camera. (Radians from the horizontal)                                           |
|`camera`-`z`              | Offset in Z direction of the camera with respect to the top of the robot base. (Between 0m and 0.574m)           |
|`left_arm_mount`          | Translation and Rotation of the tracebot_left_arm_mount frame wrt tracebot_arm_mount_base frame for left robot   |
|`left_arm_ft_sensor_tx`   | Translation and Rotation of the F/T sensor wrt to robot flange for left robot                                    |
|`left_arm_tcp`            | Translation and Rotation of the TCP wrt to robot flange for left robot                                           |
|`right_arm_mount`         | Translation and Rotation of the tracebot_right_arm_mount frame wrt tracebot_arm_mount_base frame for right robot |
|`right_arm_ft_sensor_tx`  | Translation and Rotation of the F/T sensor wrt to robot flange for right robot                                   |
|`right_arm_tcp`           | Translation and Rotation of the TCP wrt to robot flange for right robot                                          |
|`parameters`-`left_arm`   | Parameter files for the left robot                                                                               |
|`parameters`-`right_arm`  | Parameter files for the right robot                                                                              |
|`left_arm_enabled`        | Select to load left arm                                                                                          |
|`right_arm_enabled`       | Select to load right arm                                                                                         |
|`robotiq_grippers_enabled`| Set to `false` to suppress the gripper addition to the robot flanges                                             |
|`mixed_grippers`          | Set to `true` to mount the TraceBot gripper on the right arm and the RobotiQ gripper on the left arm             |
|`load_scene`              | Select to load environment meshes like table and pump                                                            |
|`show_base_model`         | Set to false to hide the TraceBot Model (replaces with cuboid)                                                   |
|`pump`-`enabled`          | Set to false to hide the Pump model                                                                              |
|`pump`-`show_pump_holder` | Set to false to hide the bottle holder from the Pump model                                                       |
|`pump`-`x`/`y`            | Translation of the Pump wrt the centre of the table                                                              |

All parameters use SI units.

## Usage

This repository offers visualisation of the mockup model within rviz using the `view_tracebot_mockup.launch` file, inside `tracebot_mockup_description`.

The rviz simulation give a basic visualisation of the robot with any of the robot arms (the default being ur5e).
The launchfile exposes the parameters listed in [Model Parameters](#model-parameters) as arguments, providing reasonable defaults.

Firsly ensuring that the workspace is sourced:

```bash
source ~/path/to/tracebot_ws/devel/setup.bash
```

- The rviz visualistation can be run with default parameters:

  ```bash
  roslaunch tracebot_description launch_tracebot_rviz.launch
  ```

- Or using the simulation variable to launch a mockup:

  ```bash
  roslaunch tracebot_description launch_tracebot_rviz.launch simulation:=true"
  ```

- A custom config.yaml file can be used by setting the `robot_configuration` launch parameter

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
  git clone https://gitlab.com/tracebot/tracebot_description.git
  ```

- Certain dependencies are not released as binary packages to either Melodic or Noetic, pull those into the workspace as well:

  ```bash
  cd ~/path/to/tracebot_ws/src
  vcs import < tracebot_description/upstream.repos # Install python(3)-vcstool if not available
  ```

- Install the rest of dependencies from binary repositories

  ```bash
  cd ~/path/to/tracebot_ws
  rosdep install -iy --from-paths src --rosdistro "$ROS_DISTRO"
  ```

- Then finally build everything:

  ```bash
  cd ~/path/to/tracebot_ws
  catkin build
  ```

## Object models convention

The project consortium agreed upon using [Bremen object modelling rules](https://ai.uni-bremen.de/wiki/3dmodeling/items).
You can find a copy of these conventions also in the [wiki of this project](https://gitlab.com/tracebot/tracebot_description/-/wikis/home)

## Camera-robot calibration tool

Relative poses between robots and camera has to be somehow measured or estimated. It is assumed that the `realsense` camera will be used for that purpose.
Once these transformations are acquired, they have to be introduced within the robot description.

A tool `camera_robot_calibration` is available to compute transformations and manually introduce them within the `urdf` as parameters.

The structure of URDF has been kept as it was before, but some additional parameters have been added to introduce the transformation from `tracebot_arm_mount_base` to  `tracebot_right_arm_mount` or `tracebot_left_arm_mount` depending the robot.

Calculations correspond to this:

```yaml
robot_base_link_T_cam = cam_T_arm_mount_base * arm_mount_base_T_arm_mount * arm_mount_T_robot_base_link
arm_mount_base_T_arm_mount = inv(cam_T_arm_mount_base) * inv(robot_base_link_T_cam) * inv(arm_mount_T_robot_base_link)
```

### Basic usage

Configure your `robot_to_camera` transformation in [configuration file](config/camera_robot_calibration.yaml)

`robot_description` has to be loaded previously.

Launch the tool:

```bash
roslaunch tracebot_description camera_robot_calibration.launch
```

This program will show the result of calculation. Copy the values and fill the `model parameters`:

- `left_arm_mount_tx` Translation in x axis of tracebot_left_arm_mount frame wrt tracebot_arm_mount_base frame for left robot
- `left_arm_mount_ty` Translation in y axis of tracebot_left_arm_mount frame wrt tracebot_arm_mount_base frame for left robot
- `left_arm_mount_tz` Translation in z axis of tracebot_left_arm_mount frame wrt tracebot_arm_mount_base frame for left robot
- `left_arm_mount_rr` Rotation in roll axis of tracebot_left_arm_mount frame wrt tracebot_arm_mount_base frame for left robot
- `left_arm_mount_rp` Rotation in pitch axis of tracebot_left_arm_mount frame wrt tracebot_arm_mount_base frame for left robot
- `left_arm_mount_ry` Rotation in yaw axis of tracebot_left_arm_mount frame wrt tracebot_arm_mount_base frame for left robot

**_NOTE:_**  Once these parameters are obtained from camera_robot_calibration tool, and set within the robot description, it is mandatory to **NOT** modify `left_arm_base_rot` and `right_arm_base_rot`.

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
