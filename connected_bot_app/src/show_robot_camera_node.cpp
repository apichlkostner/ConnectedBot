#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <connected_bot_app/photoAction.h>

#include "show_robot_camera_node.h"


void ShowRobotCamera::sensorCallback(const std_msgs::Float32MultiArray& msg) {
  for (auto i = 0u; i < NUM_IR_SENSORS; i++) {
    constexpr float fac = 0.7;
    sensor_meas_[i] = fac * sensor_meas_[i] + (1. - fac) * msg.data[i];
  }
}

void ShowRobotCamera::photoActionCb(
    const connected_bot_app::photoGoalConstPtr& goal) {
  ROS_INFO("Action: photoAction");
  
  result_.photo_done = 1;
  as_.setSucceeded(result_);  
  save_photo_ = true;

  // as_.publishFeedback(feedback_);
}

void ShowRobotCamera::processImage(cv::Mat& img) {
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
      std::max(1, (int)(80. * 40. / (sensor_meas_[2] * sensor_meas_[2])));
  circle(img, cv::Point(320, 20), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas_[1] * sensor_meas_[1])));
  circle(img, cv::Point(20, 20), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas_[3] * sensor_meas_[3])));
  circle(img, cv::Point(620, 20), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas_[0] * sensor_meas_[0])));
  circle(img, cv::Point(20, 300), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas_[4] * sensor_meas_[4])));
  circle(img, cv::Point(620, 300), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);
}

void ShowRobotCamera::imageCallback(
    const sensor_msgs::CompressedImageConstPtr& msg) {
  static int fn_nr = 1;
  cv::Mat img = cv::imdecode(cv::Mat(msg->data), 1);

  processImage(img);

  if (save_photo_) {
    ROS_INFO("Saved photo");
    std::string fn = std::to_string(fn_nr++) + ".jpg";
    cv::imwrite(
        "/home/arthur/catkin_ws/src/connected_bot_app/data/images/" + fn, img);
    save_photo_ = false;
  }

  cv::imshow("robot_camera_viewer", img);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_camera_viewer");

  // default camera topic
  std::string image_topic;
  ros::param::param<std::string>("~image_topic", image_topic,
                                 "/raspi_camera/image_raw/compressed");

  ROS_INFO_STREAM("Camera topic: " + image_topic);

  cv::namedWindow("robot_camera_viewer");

  ShowRobotCamera showRobotCamera("/make_photo", image_topic);

  ros::spin();

  cv::destroyWindow("robot_camera_viewer");
}