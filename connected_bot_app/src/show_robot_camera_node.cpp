#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <connected_bot_app/photoAction.h>

#include "show_robot_camera_node.h"

ShowRobotCamera::ShowRobotCamera()
    : as_(nh_, "/make_photo",
          boost::bind(&ShowRobotCamera::photoActionCb, this, _1), false),
      action_name_("/make_photo"),
      sensor_meas_(NUM_IR_SENSORS) {
  // parameter from calibration
  D_ = (cv::Mat_<double>(4, 1) << -0.03213332, 0.02991439, -0.07085706,
        0.04153998);
  K_ = (cv::Mat_<double>(3, 3) << 331.46297565, 0., 305.05513925, 0.,
        330.99605545, 225.88179462, 0., 0., 1.);

  identity_ = cv::Mat::eye(3, 3, CV_64F);

  // default camera topic
  std::string image_topic;
  ros::param::param<std::string>("~image_topic", image_topic,
                                 "/raspi_camera/image_raw/compressed");
  ROS_INFO_STREAM("Camera topic: " + image_topic);

  // subscribe topics
  sub_ = nh_.subscribe(image_topic, 1, &ShowRobotCamera::imageCallback, this);
  sub_ir_ =
      nh_.subscribe("/sensor/ir", 1, &ShowRobotCamera::sensorCallback, this);

  // start action server (save photo from camera to file system)
  as_.start();
}

void ShowRobotCamera::sensorCallback(const std_msgs::Float32MultiArray& msg) {
  // moving average over ir distance sensor measurements
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

void ShowRobotCamera::undistort(cv::Mat& img) {
  cv::Mat map1 = cv::Mat::zeros(3, 3, CV_32F);
  cv::Mat map2 = cv::Mat::zeros(4, 1, CV_32F);
  cv::Mat new_K;

  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      K_, D_, img.size(), identity_, new_K, 0.4);

  cv::fisheye::initUndistortRectifyMap(K_, D_, identity_, new_K, img.size(),
                                       CV_32F, map1, map2);
  cv::remap(img, img, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
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
  addWeighted(img, 0.4, img_edges, 0.6, 0.0, img);
#endif

  // image undistortion
  undistort(img);

  // visualize the ir distance measurements as circles in the image
  int ref_val =
      std::max(1, (int)(80. * 40. / (sensor_meas_[2] * sensor_meas_[2])));
  circle(img, cv::Point(320, 460), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas_[1] * sensor_meas_[1])));
  circle(img, cv::Point(20, 20), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas_[3] * sensor_meas_[3])));
  circle(img, cv::Point(620, 20), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas_[0] * sensor_meas_[0])));
  circle(img, cv::Point(20, 350), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);

  ref_val = std::max(1, (int)(80. * 40. / (sensor_meas_[4] * sensor_meas_[4])));
  circle(img, cv::Point(620, 350), ref_val, cv::Scalar(0, 0, 255), cv::FILLED,
         cv::LINE_8);
}

void ShowRobotCamera::imageCallback(
    const sensor_msgs::CompressedImageConstPtr& msg) {
  static int fn_nr = 1;

  // decode compressed image
  cv::Mat img = cv::imdecode(cv::Mat(msg->data), 1);

  // save photo can be triggered by action
  if (save_photo_) {
    ROS_INFO("Saved photo");
    std::string fn = std::to_string(fn_nr++) + ".jpg";
    cv::imwrite(
        "/home/arthur/catkin_ws/src/connected_bot_app/data/images/" + fn, img);
    save_photo_ = false;
  }

  // process the image
  processImage(img);

  // send to viewer thread
  emit newImageReady(QPixmap::fromImage(
      QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888)
          .rgbSwapped()));
}

void ShowRobotCamera::init(int argc, char** argv) {
  ros::init(argc, argv, "robot_camera_viewer");
}

void ShowRobotCamera::run() {
  ros::Rate loop_rate(60);

  while (ros::ok() && !stop_thread_) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void ShowRobotCamera::quit() { stop_thread_ = true; }

std::string ShowRobotCamera::image_topic_name() {
  return sub_.getTopic();
}