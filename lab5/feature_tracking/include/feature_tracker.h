#pragma once

#include "feature_tracker.h"
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>

class FeatureTracker {
public:
  FeatureTracker() = default;

  virtual ~FeatureTracker() = default;

  /** Main function (TODO)
   * This function receives a pair of images and tracks features from one image
   * to the other.
    @param[in] img_1, img_2 Images where to track features.
    @param[out] matched_kp_1_kp_2 pair of vectors of keypoints with the same size
    so that matched_kp_1_kp_2.first[i] matches with matched_kp_1_kp_2.second[i].
  */
  void trackFeatures(const cv::Mat& img_1, const cv::Mat& img_2,
                     std::pair<std::vector<cv::KeyPoint>,
                               std::vector<cv::KeyPoint>>* matched_kp_1_kp_2 = nullptr);

  void printStats() const;

protected:
  /**
    @param[in] img Image input where to detect keypoints.
    @param[out] keypoints List of keypoints detected on the given image.
  */
  virtual void detectKeypoints(const cv::Mat& img,
                               std::vector<cv::KeyPoint>* keypoints) const = 0;

  /**
    @param[in] img Image used to detect the keypoints.
    @param[in, out] keypoints List of keypoints detected on the image. Depending
    on the detector used some keypoints might be added or removed.
    @param[out] descriptors List of descriptors for the given keypoints.
  */
  virtual void describeKeypoints(const cv::Mat& img,
                                 std::vector<cv::KeyPoint>* keypoints,
                                 cv::Mat* descriptors) const = 0;

  /** This function matches descriptors.
      @param[in] descriptors_1 First list of descriptors.
      @param[in] descriptors_2 Second list of descriptors.
      @param[out] matches List of k best matches between descriptors.
      @param[out] good_matches List of descriptors classified as "good"
  */
  virtual void matchDescriptors(
      const cv::Mat& descriptors_1,
      const cv::Mat& descriptors_2,
      std::vector<std::vector<cv::DMatch>>* matches,
      std::vector<cv::DMatch>* good_matches) const = 0;

  /** Compute Inlier Mask out of the given matched keypoints.
   *  Both keypoints_1 and keypoints_2 input parameters must be ordered by match
   * i.e. keypoints_1[0] has been matched to keypoints_2[0].
   * Therefore, both keypoints vectors must have the same length.
    @param[in] keypoints_1 List of keypoints detected on the first image.
    @param[in] keypoints_2 List of keypoints detected on the second image.
    @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
  */
  void inlierMaskComputation(const std::vector<cv::KeyPoint>& keypoints_1,
                             const std::vector<cv::KeyPoint>& keypoints_2,
                             std::vector<uchar>* inlier_mask) const;

  void drawMatches(const cv::Mat& img_1, const cv::Mat& img_2,
                   const std::vector<cv::KeyPoint>& keypoints_1,
                   const std::vector<cv::KeyPoint>& keypoints_2,
                   const std::vector<std::vector<cv::DMatch>>& matches);

private:
  //Statistics
  float avg_num_keypoints_img1_ = 0;
  float avg_num_keypoints_img2_ = 0;
  float avg_num_matches_ = 0;
  float avg_num_good_matches_ = 0;
  float avg_num_inliers_ = 0;
  float avg_inlier_ratio_ = 0;
  unsigned int num_samples_ = 0;
};
