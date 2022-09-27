#pragma once


#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opengv/relative_pose/methods.hpp>


#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Eigen>

/*
 * Structure containing the camera parameters.
 */
struct CameraParams {
  cv::Mat K, D;
};

geometry_msgs::Pose cv2Pose(const cv::Mat& R, const cv::Mat& t) {
  geometry_msgs::Pose pose;

  if (!t.empty() && !R.empty()) {
    pose.position.x = t.at<double>(0,0);
    pose.position.y = t.at<double>(1,0);
    pose.position.z = t.at<double>(2,0);

    // Convert OpenCV to tf matrix
    tf::Matrix3x3 tf_rot(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));

    // convert rotation matrix to an orientation quaternion
    tf::Quaternion quat;
    tf_rot.getRotation(quat);

    // convert quaterion to geometry_msgs/Orientation
    tf::quaternionTFToMsg(quat, pose.orientation);
  }

  return pose;
}


// ----------------------------------------------------------------------------

/**
 * @brief      Extract pose of camera 1 w.r.t camera 2
 *
 * @param[in]  Es            Vector of essential matrix solutions from OpenGV
 * @param[in]  pts1          Feature correspondences from camera 1
 * @param[in]  pts2          Feature correspondences from camera 2
 * @param[in]  camera_params Camera parameters
 * @param      R             Best rotation matrix
 * @param      t             Best translation vector
 *
 * @return     Number of inliers achieved from best solution
 */
int extractPose(const opengv::essentials_t& Es,
                const std::vector<cv::Point2f>& pts1,
                const std::vector<cv::Point2f>& pts2,
                const CameraParams camera_params,
                cv::Mat& R, cv::Mat& t)
{
  // std::cout << std::endl << std::string(80,'-') << std::endl;

  // OpenCV findEssentialMat
  // cv::Mat E = cv::findEssentialMat(pts1, pts2, camera_params_.K);
  // int inliers = cv::recoverPose(E, pts1, pts2, camera_params_.K, R, t);
  // return inliers;
  // std::cout << "From OpenCV (" << inliers << " inliers):" << std::endl;
  // std::cout << E << std::endl;

  int max_inliers = 0;

  // std::cout << "From OpenGV (" << Es.size() << " solns): " << std::endl;
  for (int i=0; i<Es.size(); i++) {
    // for convenience
    auto E = Es[i];

    // convert eigen to cv mat
    cv::Mat Emat;
    cv::eigen2cv(E, Emat);

    cv::Mat tmpR, tmpT;
    int inliers = cv::recoverPose(Emat, pts1, pts2, camera_params.K, tmpR, tmpT);

    // std::cout << "Soln (" << i << "): " << inliers << " inliers:" << std::endl;
    // std::cout << "[" << E << "]" << std::endl;

    // choose the solution with the most inliers
    if (inliers > max_inliers) {
      max_inliers = inliers;
      cv::swap(R, tmpR);
      cv::swap(t, tmpT);
    }
  }

  return max_inliers;
}

// ----------------------------------------------------------------------------

geometry_msgs::Pose eigen2Pose(const opengv::transformation_t& T) {
  geometry_msgs::Pose pose;

  pose.position.x = T(0,3);
  pose.position.y = T(1,3);
  pose.position.z = T(2,3);

  tf::Matrix3x3 tf_rot;
  tf::matrixEigenToTF(T.block<3,3>(0,0), tf_rot);

  // convert rotation matrix to an orientation quaternion
  tf::Quaternion quat;
  tf_rot.getRotation(quat);

  // convert quaterion to geometry_msgs/Orientation
  tf::quaternionTFToMsg(quat, pose.orientation);

  return pose;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Create a color from red to green, based on blend
 *
 * @param[in]  blend  Blend parameter \in [0 1]. One is green.
 *
 * @return     cv::Scalar
 */
cv::Scalar blendedColor(double blend) {
  double r = 2*(1-blend);
  double g = 2*blend;

  return cv::Scalar(0, static_cast<int>(g*255), static_cast<int>(r*255));
}

// ----------------------------------------------------------------------------
