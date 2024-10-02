# Usage

1. To just compare feature trackers using a pair of images, run:
```
ros2 launch lab_5 two_frames_tracking.launch.yaml
```
> **Note** Not so fast, first you have to implement the functions inside the source code, follow the handout at this point.

2. To run the feature tracking on a video sequence, we need to play the dataset rosbag.
For this we provide another launch file.
```
ros2 launch lab_5 video_tracking.launch.yaml
```
