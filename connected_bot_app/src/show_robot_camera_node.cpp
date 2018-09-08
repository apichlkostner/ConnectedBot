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

void processImage(cv::Mat& img) {
#if 0
  // Canny edge detection
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
  Canny(img_gray, img_gray, 100, 200, 3);
  cv::Mat img_edges;
  cv::cvtColor(img_gray, img_edges, cv::COLOR_GRAY2BGR);

  // merge edges and image
  addWeighted(img, 0.2, img_edges, 0.8, 0.0, img);
#endif
  // img = img * 5;

  // visualize the ir distance measurements as circles in the image
  int ref_val =
      std::max(1, (int)(80. * 40. / (sensor_meas[2] * sensor_meas[2])));
  circle(img, cv::Point(320, 20), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[1] * sensor_meas[1])));
  circle(img, cv::Point(20, 20), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[3] * sensor_meas[3])));
  circle(img, cv::Point(620, 20), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[0] * sensor_meas[0])));
  circle(img, cv::Point(20, 300), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas[4] * sensor_meas[4])));
  circle(img, cv::Point(620, 300), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
  cv::Mat img = cv::imdecode(cv::Mat(msg->data), 1);

  processImage(img);

  cv::imshow("robot_camera_viewer", img);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_camera_viewer");
  ros::NodeHandle nh;

  // default camera topic
  std::string image_topic;
  ros::param::param<std::string>("~image_topic", image_topic,
                                 "/raspi_camera/image_raw/compressed");

  ROS_INFO_STREAM("Camera topic: " + image_topic);

  cv::namedWindow("robot_camera_viewer");

  // subscriber for camera and IR sensors
  ros::Subscriber sub = nh.subscribe(image_topic, 1, imageCallback);
  ros::Subscriber sub_ir = nh.subscribe("/sensor/ir", 1, sensorCallback);

  ros::spin();

  cv::destroyWindow("robot_camera_viewer");
}