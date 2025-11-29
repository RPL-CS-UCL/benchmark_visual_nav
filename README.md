### Environment Setup

#### 1. CMU Environment

- Official site: [cmu-exploration.com](https://www.cmu-exploration.com/)
- To download the simulation environments (~500MB), run:
  ```
  ./data/cmu_environment/download_environments.sh
  ```
- If the script fails, you can manually download the environments from [here](https://drive.google.com/file/d/1GMT8tptb3nAb87F8eFfmIgjma6Bu0reV/view?usp=sharing) and unzip them into `./data/cmu_environment/`.

#### 2. Matterport Environment

- Follow [these instructions](https://drive.google.com/file/d/1xV3L2xW4JtPMZpY8t43aqlXDhraZYLDi/view?usp=sharing) from the [official site](https://www.cmu-exploration.com/) to set up the Matterport environment.
- Place all downloaded files into the corresponding environment directory. For example, for the environment ID `17DRP5sb8fy`, use:
  ```
  data/matterport/17DRP5sb8fy/navigation_environment/
    meshes/
    preview/
    segmentations/
  ```

### Usage

#### 1. Launch Matterport 17DRP5sb8fy
```
roslaunch environment/launch/system_17DRP5sb8fy.launch useLocalPlanner:=true use_rviz:=true
python cmu_autonomous_exploration_development/src/segmentation_proc/scripts/habitat_online_v0.2.1.py
```

#### 2. Run iPlanner
```
roslaunch benchmark_visual_nav iplanner.launch config:=anymal use_rviz:=true rviz_name:=anymal
```

### Troubleshooting

- You may safely ignore the ROS warning: `Spawn service failed. Exiting.`