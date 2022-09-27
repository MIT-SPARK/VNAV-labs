#pragma once

#include "feature_tracker.h"
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>

class SiftFeatureTracker: public FeatureTracker {
 public:
  SiftFeatureTracker();

  ~SiftFeatureTracker() = default;

 protected:
  /** TODO: this function detects keypoints in an image.
      @param[in] img Image input where to detect keypoints.
      @param[out] keypoints List of keypoints detected on the given image.
  */
  virtual void detectKeypoints(const cv::Mat& img,
                               std::vector<cv::KeyPoint>* keypoints) const;

  /** TODO: this function describes keypoints in an image.
      @param[in] img Image used to detect the keypoints.
      @param[in, out] keypoints List of keypoints detected on the image. Depending
      on the detector used some keypoints might be added or removed.
      @param[out] descriptors List of descriptors for the given keypoints.
  */
  virtual void describeKeypoints(const cv::Mat& img,
                                 std::vector<cv::KeyPoint>* keypoints,
                                 cv::Mat* descriptors) const;

  /** TODO: this function matches descriptors.
      @param[in] descriptors_1 First list of descriptors.
      @param[in] descriptors_2 Second list of descriptors.
      @param[out] matches List of k best matches between descriptors.
      @param[out] good_matches List of descriptors classified as "good"
  */
  virtual void matchDescriptors(
                                const cv::Mat& descriptors_1,
                                const cv::Mat& descriptors_2,
                                std::vector<std::vector<cv::DMatch>>* matches,
                                std::vector<cv::DMatch>* good_matches) const;
 private:
  // Detector
  cv::Ptr<cv::xfeatures2d::SIFT> detector;
};




