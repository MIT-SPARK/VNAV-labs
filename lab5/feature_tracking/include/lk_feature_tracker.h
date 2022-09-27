#pragma once

#include <vector>
#include "feature_tracker.h"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

class LKFeatureTracker {
 public:
  LKFeatureTracker();

  ~LKFeatureTracker();

  /** TODO: this function tracks features for a given frame
   * @param[in] the current image frame
   * Should update prev_frame_ and prev_corners_ as needed
   */
  void trackFeatures(const cv::Mat& frame);

  /** Compute Inlier Mask out of the given matched keypoints.
   @param[in] pts1 List of keypoints detected on the first image.
   @param[in] pts2 List of keypoints detected on the second image.
   @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
  */
  void inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                             const std::vector<cv::Point2f>& pts2,
                             std::vector<uchar>* inlier_mask) const;

 private:

  /** TODO Display image with tracked features from prev to curr on the image
   * corresponding to 'frame'
   * @param[in] frame The current image frame, to draw the feature track on
   * @param[in] prev The previous set of keypoints
   * @param[in] curr The set of keypoints for the current frame
   */
  void show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                 std::vector<cv::Point2f>& curr);

  static constexpr const char* window_name_ = "LK";
  cv::Mat prev_frame_;
  std::vector<cv::Point2f> prev_corners_;

  // Statistics
  float avg_num_keypoints_img1_ = 0;
  float avg_num_keypoints_img2_ = 0;
  float avg_num_matches_ = 0;
  float avg_num_inliers_ = 0;
  float avg_inlier_ratio_ = 0;
  unsigned int num_samples_ = 0;
};
