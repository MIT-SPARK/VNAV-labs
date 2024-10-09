# Installation

1. Add new packages to your catkin workspace using wstool:
```
cd {VNAV_HOME}/vnav_ws/src/
cp -r {VNAV_HOME}/Labs/Lab_6 ./
wstool merge {VNAV_HOME}/vnav_ws/src/Lab_6/install/lab_6.rosinstall -y
wstool update -j8
```

> **Note** this might take 5min, opengv is large.

4. Build lab_6:
```
catkin build lab_6
```

Remember to source your workspace:
```
source {VNAV_HOME}/vnav_ws/devel/setup.bash
```

# Usage

1. To run the pose estimation on the given rosbag, we will use:
```
roslaunch lab_6 video_tracking.launch
```
> **Note** Not so fast, first you have to implement the functions inside the source code, follow the handout at this point.

<!-- # How to Use Lab 5 provided solution for Lab 6!
We have provided you with a solution example for lab_5.
It is stored in the Lab_5_solution.
To use this solution you will have to do the following:
```
cd {VNAV_HOME}/vnav_ws/src/Labs/Lab_5
touch CATKIN_IGNORE 
```

This command basically tells catkin build system to ignore Lab_5 project.
This same CATKIN_IGNORE is in the Lab_5_solution, so that you do not have conflicts with your current Lab_5.
Nevertheless, you should now remove it to use our solution instead!
```
cd {VNAV_HOME}/vnav_ws/src/Labs/Lab_5_solution
rm CATKIN_IGNORE
```
Now, make sure you clean lab_5 project in catkin, otherwise catkin gets confused by this new folder change.
```
catkin clean lab_5
catkin build lab_5
```

> Also note that, **if you do not use the provided solution for lab_5, you will need to modify track_features function of FeatureTracker 
> to return the keypoint correspondences**. -->
