#!/bin/bash

# CONDA_DIR="/home/$USER/anaconda3"
# CONDA_BIN_DIR="$CONDA_DIR/bin"
# CONDA_SETUP_FILE="$CONDA_DIR/bin/conda"
# CONDA_PROFILE_FILE="$CONDA_DIR/etc/profile.d/conda.sh"

# SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# # >>> conda initialize >>>
# # !! Contents within this block are managed by 'conda init' !!
# __conda_setup="$($CONDA_SETUP_FILE 'shell.bash' 'hook' 2> /dev/null)"
# if [ $? -eq 0 ]; then
#     eval "$__conda_setup"
# else
#     if [ -f $CONDA_PROFILE_FILE ]; then
#         . $CONDA_PROFILE_FILE
#     else
#         export PATH="$CONDA_BIN_DIR:$PATH"
#     fi
# fi
# unset __conda_setup
# # <<< conda initialize <<<

# conda activate habitat
# cd $SCRIPT_DIR/src/segmentation_proc/scripts
# python3 ./habitat_online_360_v0.2.1.py &
# sleep 2
# cd ../../..
# source ./devel/setup.bash
# roslaunch vehicle_simulator system_matterport.launch

ENVIRONMENT_DIR = "/Titan/dataset/data_benchmark_visual_nav/matterport/17DRP5sb8fy"

# habitat
conda activate habitat
cd $SCRIPT_DIR/src/segmentation_proc
python3 segmentation_proc/scripts/habitat_online_v0.2.1.py \
    --scene "$ENVIRONMENT_DIR"/navigation_environment/segmentations/matterport.glb

# environment
sleep 2
cd ../../..
source ./devel/setup.bash
roslaunch matterport/system_17DRP5sb8fy.launch useLocalPlanner:=false

# iPlanner
sleep 2
roslaunch iplanner.launch rgb_topic:=/habitat_camera/color/image depth_topic:=/habitat_camera/depth/image use_rviz:=true rviz_name:=habitat.rviz

# waypoint example
sleep 2
roslaunch waypoint_example.launch environment_directory:="$ENVIRONMENT_DIR"

