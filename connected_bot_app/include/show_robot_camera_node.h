#ifndef SHOW_ROBOT_CAMERA_NODE_H
#define SHOW_ROBOT_CAMERA_NODE_H

#include <QThread>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <QPixmap>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <connected_bot_app/photoAction.h>

class ShowRobotCamera : public QThread {
  Q_OBJECT
 public:
  ShowRobotCamera(std::string name, std::string image_topic);

  void run() override;
  static void init(int argc, char** argv);

 signals:
  void newImageReady(QPixmap pixmap);

 private:
  void sensorCallback(const std_msgs::Float32MultiArray& msg);
  void processImage(cv::Mat& img);
  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
  void photoActionCb(const connected_bot_app::photoGoalConstPtr& goal);

  void undistort(cv::Mat& img);

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<connected_bot_app::photoAction> as_;
  std::string action_name_;
  connected_bot_app::photoFeedback feedback_;
  connected_bot_app::photoResult result_;

  static constexpr size_t NUM_IR_SENSORS = 5;
  std::vector<float> sensor_meas_;

  ros::Subscriber sub_;
  ros::Subscriber sub_ir_;

  bool save_photo_ = false;

  cv::Mat identity_;
  cv::Mat K_;
  cv::Mat D_;
};

#endif