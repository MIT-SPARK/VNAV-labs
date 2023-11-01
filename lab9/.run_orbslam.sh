#!/bin/bash
ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc ORB_SLAM3/Vocabulary/ORBvoc.txt ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml /datasets/MH_01_easy ORB_SLAM3/Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt
cp CameraTrajectory.txt /output/orbslam
