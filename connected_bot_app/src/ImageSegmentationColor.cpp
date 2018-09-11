#include "ImageSegmentationColor.h"

cv::Mat ImageSegmentationColor::segment(const cv::Mat& img) {
  cv::Mat img_seg(img.size(), CV_8U);
  cv::Mat img_partmask(img.size(), CV_8U);
  cv::Mat img_mask(img.size(), CV_8U);

  cv::Mat img_value;
  cv::extractChannel(img, img_value, 2);
  double min_v, max_v;
  cv::minMaxLoc(img_value, &min_v, &max_v);
  std::cout << "Max = " << max_v << std::endl;

  for (const auto& seg_or : segmentations_) {
    img_mask = cv::Scalar(1);
    for (const auto& seg_and : seg_or) {
      cv::Mat img_colorspace;
      cvtColor(img, img_colorspace, seg_and.color_conversion);
      std::cout << "Thresholds: " << seg_and.thresholds_lower << " "
                << seg_and.thresholds_upper << std::endl;
      cv::inRange(img, seg_and.thresholds_lower, seg_and.thresholds_upper,
                  img_partmask);
      img_mask &= img_partmask;
    }
    img_seg |= img_mask;
  }

  return img_seg;
}

#if 1
int main() {
  cv::Mat img = cv::imread(
      "/home/arthur/catkin_ws/src/connected_bot_app/data/images/1.jpg");

  //ColorSegmentation cc0{0, {150, 0, 0}, {0, 0, 0}};
  ColorSegmentation cc1{cv::COLOR_BGR2HSV, {0, 0, 70}, {255, 255, 255}};
  std::vector<std::vector<ColorSegmentation>> color_seg_vals{{cc1}};

  ImageSegmentationColor isc(color_seg_vals);

  cv::Mat mask = isc.segment(img);

  cv::Mat segImg;
  img.copyTo(segImg, mask);

  cv::imshow("robot_camera_viewer", segImg);
  cv::waitKey(0);
}
#endif