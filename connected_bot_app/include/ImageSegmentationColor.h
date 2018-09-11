#ifndef IMAGESEGMENTATIONCOLOR_H_
#define IMAGESEGMENTATIONCOLOR_H_

#include <opencv2/opencv.hpp>

struct ColorSegmentation {
  int color_conversion;
  cv::Scalar thresholds_lower;
  cv::Scalar thresholds_upper;  
};

class ImageSegmentationColor {
 public:
  ImageSegmentationColor()
      : segmentations_(0, std::vector<ColorSegmentation>(0)) {
  };

  ImageSegmentationColor(std::vector<std::vector<ColorSegmentation>> cs) : segmentations_(cs) {

  }

  cv::Mat segment(const cv::Mat& img);

 private:
  std::vector<std::vector<ColorSegmentation>> segmentations_;
};

#endif