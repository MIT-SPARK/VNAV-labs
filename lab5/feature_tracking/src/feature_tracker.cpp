#include "feature_tracker.h"

#include <vector>
#include <numeric>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2021  - Lab 5 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a generic feature tracker base class
//  which you will augment with several derived classes for SIFT, SURF, ORB, and
//  FAST+BRIEF feature tracking.
//
// NOTE: Deliverables for the TEAM portion of this assignment start at number 3
// and end at number 7. If you have completed the parts labeled Deliverable 3-7,
// you are done with the TEAM portion of the lab. Deliverables 1-2 are
// individual.

using namespace cv;
using namespace cv::xfeatures2d;

/** TODO This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] img_1, img_2 Images where to track features.
 @param[out] matched_kp_1_kp_2 pair of vectors of keypoints with the same size
 so that matched_kp_1_kp_2.first[i] matches with matched_kp_1_kp_2.second[i].
*/
void FeatureTracker::trackFeatures(const cv::Mat &img_1,
                                   const cv::Mat &img_2,
                                   std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> *matched_kp_1_kp_2) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 3 | Feature Descriptors (SIFT)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For your convenience,  
  // here are two vectors of keypoints representing keypoints for image 1 and 2.
  std::vector<KeyPoint> keypoints_1, keypoints_2; 
  //
  //
  // For this part, you will need to complete the following tasks.
  //
  // Note: the actual logic for computing features  will happen inside each of the 
  // derived class. Here we are just calling the derived classes functions 
  // (because they share the same interface), and operate on the results on a
  // higher level.
  //
  //   1. Detect the keypoints for each image independently. Implement and use
  //   the skeleton function 'detectKeypoints' in the derived class, (i.e. in
  //   SiftFeatureTracker)
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  //
  //   2. Display detected keypoints on both images, and save them for the
  //   deliverable. Make use of the OpenCV functions 'drawKeypoints', 'imwrite'. 
  //   Tip: use 'waitKey(0);' to avoid opencv closing the visualization window. 
  //   Mind that using waitKey(0) will block ROS from dying (bottom of this 
  //   file we offer an alternative).
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  //
  //   3. Compute descriptors by filling in the skeleton function
  //   `describeKeypoints` in the relevant derived classes (i.e.
  //   SiftFeatureTracker)
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  //
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 3
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4 | Descriptor-based Feature Matching
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  // Matches and good matches to be populated
  std::vector<std::vector<DMatch>> matches;
  std::vector<DMatch> good_matches;
  //
  // For this part, you will need to:
  //   1. Match descriptor vectors using the FLANN matcher. FLANN = Fast
  //   Approximate Nearest Neighbor Search Library. Use and fill in the skeleton
  //   function 'matchDescriptors' in the derived class (e.g.
  //   SiftFeatureTracker), and call them here.
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  //
  //   2. Plot the matches using the opencv function 'drawMatches'. Save the
  //   image for deliverable.
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  //
  //   3. Use the function 'inlierMaskComputation' to get the inliers amongst
  //   the matches you computed. (You are not supposed to modify this function,
  //   but you will learn what it is doing in the next lecture).
  //   Note: you may need to create additional variables based on match, 
  //   good_matches and keypoints_1/2 as inputs to the
  //   'inlierMaskComputation' function.
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  //
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 4
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 5 | Keypoint Matching Quality
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  unsigned int num_inliers = 0; // TODO compute number of inliers
  // 
  // For this part, you will need to:
  //   1. Draw the inlier (green) and outlier (red) matches in a similar way
  //   than in the handout. Make use of the function 'drawMatches'. Tip: you can
  //   first draw the outliers and then re-use the same image to draw the
  //   inliers. See the handout for more details
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  //
  //   2. Calculate the statistics to fill the table in the handout.
  //   Feel free to edit the code below to include more interesting stats.

  float const new_num_samples = static_cast<float>(num_samples_) + 1.0f;
  float const old_num_samples = static_cast<float>(num_samples_);
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * old_num_samples + static_cast<float>(keypoints_1.size())) / new_num_samples;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * old_num_samples + static_cast<float>(keypoints_2.size())) / new_num_samples;
  avg_num_matches_ = (avg_num_matches_ * old_num_samples + static_cast<float>(matches.size())) / new_num_samples;
  avg_num_good_matches_ = (avg_num_good_matches_ * old_num_samples + static_cast<float>(good_matches.size())) / new_num_samples;
  avg_num_inliers_ = (avg_num_inliers_ * old_num_samples + static_cast<float>(num_inliers)) / new_num_samples;
  avg_inlier_ratio_ =
      (avg_inlier_ratio_ * old_num_samples + (static_cast<float>(num_inliers) / static_cast<float>(good_matches.size()))) / new_num_samples;
  ++num_samples_;

  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 5
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/** Compute Inlier Mask out of the given matched keypoints.
   *  Both keypoints_1 and keypoints_2 input parameters must be ordered by match
   * i.e. keypoints_1[0] has been matched to keypoints_2[0].
   * Therefore, both keypoints vectors must have the same length.
    @param[in] keypoints_1 List of keypoints detected on the first image.
    @param[in] keypoints_2 List of keypoints detected on the second image.
    @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
  */
void FeatureTracker::inlierMaskComputation(const std::vector<KeyPoint> &keypoints_1,
                                           const std::vector<KeyPoint> &keypoints_2,
                                           std::vector<uchar> *inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);
  const size_t size = keypoints_1.size();
  CHECK_EQ(keypoints_2.size(), size) << "Size of keypoint vectors "
                                        "should be the same!";

  std::vector<Point2f> pts1(size);
  std::vector<Point2f> pts2(size);
  for (size_t i = 0; i < keypoints_1.size(); i++) {
    pts1[i] = keypoints_1[i].pt;
    pts2[i] = keypoints_2[i].pt;
  }

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC, max_dist_from_epi_line_in_px, confidence_prob, *inlier_mask);
  } catch (...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}

/** Example of function to draw matches. Feel free to re-use this example or
 *  create your own. You will need to modify it in order to plot the different
 *  figures. You can add more functions to this class if needed.
 */
void FeatureTracker::drawMatches(const cv::Mat &img_1,
                                 const cv::Mat &img_2,
                                 const std::vector<KeyPoint> &keypoints_1,
                                 const std::vector<KeyPoint> &keypoints_2,
                                 const std::vector<std::vector<DMatch>> &matches) {
  cv::namedWindow("tracked_features", cv::WINDOW_NORMAL);
  cv::Mat img_matches;
  cv::drawMatches(img_1,
                  keypoints_1,
                  img_2,
                  keypoints_2,
                  matches,
                  img_matches,
                  Scalar::all(-1),
                  Scalar::all(-1),
                  std::vector<std::vector<char>>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  // Show detected matches
  imshow("tracked_features", img_matches);

  // Wait indefinitely for key pressed, but allow ROS to also kill everything.
  // Otherwise ROS will not die unless we close the window.
  // Use this when wanting to visualize pairs of images, one at a time.
  while (ros::ok() && waitKey(10) == -1) {} // Instead of using waitKey(0);

  // Alternatively, just wait for some seconds. Use this when playing with video
  // sequences.
  // waitKey(10);
}

void FeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of good matches: " << avg_num_good_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
