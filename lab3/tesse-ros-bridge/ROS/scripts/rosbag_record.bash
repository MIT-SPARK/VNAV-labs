#!/bin/bash

rosbag record \
  /clock \
  /tesse/left_cam/camera_info \
  /tesse/left_cam/rgb/image_raw \
  /tesse/left_cam/mono/image_raw \
  /tesse/right_cam/camera_info \
  /tesse/right_cam/rgb/image_raw \
  /tesse/right_cam/mono/image_raw \
  /tesse/depth_cam/camera_info \
  /tesse/depth_cam/mono/image_raw \
  /tesse/seg_cam/camera_info \
  /tesse/seg_cam/rgb/image_raw \
  /tesse/front_lidar/scan \
  /tesse/rear_lidar/scan \
  /tesse/imu/clean/imu \
  /tesse/imu/noisy/imu \
  /tesse/imu/noisy/biases/gyro \
  /tesse/imu/noisy/biases/accel \
  /tesse/odom \
  /tf \
  /tf_static # \
  # /tesse/odom_noisy \
  # /tesse/collision \
  # /tesse/drive \
  # /tesse/third_person/camera_info \
  # /tesse/third_person/rgb/image_raw \
