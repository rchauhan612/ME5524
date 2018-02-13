# MBZIRC 2019

## Update Dependencies

   wstool update -t src

## Build Workspace

    catkin build

## Run Simulation

    roslaunch mbzirc_gazebo simulator.launch

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

### Dependencies

    sudo apt install protobuf-compiler python-pip python-numpy python-toml python-jinja2
    sudo geographiclib-get-geoids all
