cmake_minimum_required(VERSION 3.0.2)
project(tracebot_description)

find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY config launch meshes urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)

catkin_install_python(PROGRAMS scripts/camera_robot_calibration DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})