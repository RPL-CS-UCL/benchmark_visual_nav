### Usage
1. Run matterport 17DRP5sb8fy
```
roslaunch environment/launch/system_17DRP5sb8fy.launch useLocalPlanner:=true use_rviz:=true
python cmu_autonomous_exploration_development/src/segmentation_proc/scripts/habitat_online_v0.2.1.py
```

### Run iPlanner
```
roslaunch benchmark_visual_nav iplanner.launch config:=anymal use_rviz:=true rviz_name:=anymal
```


### Issues

1. Ignore the issue in ROS: ```Spawn service failed. Exiting.```