# MBZIRC 2019

## Workspace

Here are some common workspace commands for updating and running the simulation and system.

### Update Dependencies

    wstool update -t src

### Build

    catkin build

### Run Simulation

    roslaunch mbzirc_gazebo challenge1_simulator.launch

### Run System

#### Simulation

    roslaunch mbzirc_gazebo challenge1_system.launch

#### Hardware

    roslaunch mbzirc_system bogey0.launch

## Installation Instructions

### Configure Environment

Add this to your .bashrc file. **Note CATKIN_WS should be set to where you have this Catkin workspace.**

    export CATKIN_WS=$HOME/catkin_ws
    export SITL_GAZEBO_PATH=$CATKIN_WS/src/sitl_gazebo

    source /opt/ros/lunar/setup.bash
    source /usr/share/gazebo/setup.sh
    source $CATKIN_WS/devel/setup.bash

    if [[ $GAZEBO_RESOURCE_PATH != *"${SITL_GAZEBO_PATH}"* ]]
    then
      export GAZEBO_RESOURCE_PATH=$SITL_GAZEBO_PATH:$GAZEBO_RESOURCE_PATH
    fi
    if [[ $GAZEBO_PLUGIN_PATH != *"${SITL_GAZEBO_PATH}/build"* ]]
    then
      export GAZEBO_PLUGIN_PATH=$SITL_GAZEBO_PATH/build:$GAZEBO_PLUGIN_PATH
    fi
    if [[ $GAZEBO_MODEL_PATH != *"${SITL_GAZEBO_PATH}/models"* ]]
    then
      export GAZEBO_MODEL_PATH=$SITL_GAZEBO_PATH/models:$GAZEBO_MODEL_PATH
    fi

### System Dependencies

    sudo apt install ros-lunar-mavros ros-lunar-mavros-extras ros-lunar-mavros-msgs
    sudo apt install protobuf-compiler python-pip python-numpy python-toml python-jinja2
    sudo geographiclib-get-geoids all
    sudo apt-get install python-catkin-tools
## Control Reference

### Takeoff

    rosservice call /bogey0/offboard_control/takeoff "{}"

### Land

    rosservice call /bogey0/offboard_control/land "{}"

### Waypoint

    rosservice call /bogey0/offboard_control/waypoint "{ position: { x: 0.0, y: 0.0, z: 10.0 }, yaw: 0.0 }"

### Velocity

    rosservice call /bogey0/offboard_control/velocity "{ linear: { x: 0.0, y: 0.0, z: 0.0 }, yaw: 0.0 }"

### State

    rostopic echo /bogey0/offboard_control/state

## Simulation Notes

### Generate Bogey Models

You can change the number of Bogeys that get generated in the script.

    $CATKIN_WS/src/mbzirc_gazebo/scripts/generate_bogey_models.sh

You may need to set the permissions for this script:

    chmod +x $CATKIN_WS/src/mbzirc_gazebo/scripts/generate_bogey_models.sh
