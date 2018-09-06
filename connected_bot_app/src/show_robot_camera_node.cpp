#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

constexpr size_t NUM_IR_SENSORS = 5;
std::vector<float> sensor_meas(NUM_IR_SENSORS);

void sensorCallback(const std_msgs::Float32MultiArray& msg) {
  for (auto i = 0u; i < NUM_IR_SENSORS; i++) {
    // std::cout << msg.data[i] << " ";
    constexpr float fac = 0.7;
    sensor_meas[i] = fac * sensor_meas[i] + (1. - fac) * msg.data[i];
  }
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
  cv::Mat img = cv::imdecode(cv::Mat(msg->data), 1);

  // visualize the ir distance measurements as circles in the image
  int ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[2] * sensor_meas[2])));
  circle(img, cv::Point(320, 20), ref_val,
          cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[1] * sensor_meas[1])));
  circle(img, cv::Point(20, 20), ref_val,
          cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[3] * sensor_meas[3])));
  circle(img, cv::Point(620, 20), ref_val,
          cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[0] * sensor_meas[0])));
  circle(img, cv::Point(20, 300), ref_val,
          cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[4] * sensor_meas[4])));
  circle(img, cv::Point(620, 300), ref_val,
          cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);

  

  cv::imshow("robot_camera_viewer", img);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_camera_viewer");
  ros::NodeHandle nh;

  cv::namedWindow("robot_camera_viewer");

  ros::Subscriber sub =
      nh.subscribe("/webcam/image_raw/compressed", 1, imageCallback);

  ros::Subscriber sub_ir = nh.subscribe("/sensor/ir", 1, sensorCallback);

  ros::spin();

  cv::destroyWindow("robot_camera_viewer");
}