/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 */

#pragma once

// As this is a full 3D problem, we will use Pose3 variables to represent the camera
// positions and Point3 variables (x, y, z) to represent the landmark coordinates.
// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
// We will also need a camera object to hold calibration information and perform projections.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

// We will also need a camera object to hold calibration information and perform projections.
#include <gtsam/geometry/SimpleCamera.h>

/* ************************************************************************* */
std::vector<gtsam::Point3> createPoints() {
  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(1.0,1.0,1.0));
  points.push_back(gtsam::Point3(-1.0,1.0,1.0));
  points.push_back(gtsam::Point3(-1.0,-1.0,1.0));
  points.push_back(gtsam::Point3(1.0,-1.0,1.0));
  points.push_back(gtsam::Point3(1.0,1.0,-1.0));
  points.push_back(gtsam::Point3(-1.0,1.0,-1.0));
  points.push_back(gtsam::Point3(-1.0,-1.0,-1.0));
  points.push_back(gtsam::Point3(1.0,-1.0,-1.0));

  return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses() {
  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;
  double radius = 3.0;
  int i = 0;
  double theta = 0.0;
  gtsam::Point3 up(0,0,1);
  gtsam::Point3 target(0,0,0);
  for(; i < 8; ++i, theta += 2*M_PI/8) {
    gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
    gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
    poses.push_back(camera.pose());
  }
  return poses;
}
/* ************************************************************************* */
