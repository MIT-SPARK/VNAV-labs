/*
 * @file pose_estimation.h
 * @brief Include file for pose_estimation.cpp
 * Estimates the pose from frame to frame.
 * (TODO) Feel free to use a header file to organize your solution.
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <memory>

#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>