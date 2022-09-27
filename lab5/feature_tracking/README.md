# Installation

1. Here are some extra dependencies that you might need for this code to work:

```
sudo apt-get update
sudo apt-get install libgoogle-glog-dev python-catkin-tools autoconf python-wstool libgtk2.0-dev pkg-config
```
2. Let us use catkin build to build the code:
```
cd {VNAV_HOME}/vnav_ws/
catkin clean -y
catkin build lab_5
```

> **Note** now you can build the workspace from anywhere inside the workspace, no need to go to the src folder. Please post on Piazza if you run into any build issues.

Remember to source your workspace:
```
source {VNAV_HOME}/vnav_ws/devel/setup.bash
```

# Usage

1. To just compare feature trackers using a pair of images, run:
```
roslaunch lab_5 two_frames_tracking.launch
```
> **Note** Not so fast, first you have to implement the functions inside the source code, follow the handout at this point.

2. To run the feature tracking on a video sequence, we need to play the dataset rosbag (please see handout for details and link).
For this we provide another launch file.
```
roslaunch lab_5 video_tracking.launch path_to_dataset:=/.../your/path/here/file.bag
```


