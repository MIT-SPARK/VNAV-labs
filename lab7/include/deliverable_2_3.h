#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class MoCapPosition3Factor : public NoiseModelFactor1<Pose3> {
 private:
  // Measurement information.
  Point3 m_;

 public:
  /**
   * Constructor
   * @param poseKey    The key of the pose.
   * @param m          Point3 measurement.
   * @param model      Noise model for Motion Capture sensor.
   */
  MoCapPosition3Factor(gtsam::Key poseKey, const gtsam::Point3& m,
                       const gtsam::SharedNoiseModel& model)
      : NoiseModelFactor1<Pose3>(model, poseKey), m_(m) {}

  // Error function.
  // @param p 3D pose.
  // @param H optional Jacobian matrix.
  gtsam::Vector evaluateError(
      const gtsam::Pose3& p,
      boost::optional<gtsam::Matrix&> H = boost::none) const {

    // 3a. Complete definition of factor
    // TODO: 
    // Return error vector and jacobian if requested (aka H !=
    // boost::none).
    //
    // Insert code below:

    // End 3a. 
  }

  ~MoCapPosition3Factor() {}
};

}  // namespace gtsam
