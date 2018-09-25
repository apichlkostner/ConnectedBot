#ifndef SHOW_ROBOT_CAMERA_NODE_H
#define SHOW_ROBOT_CAMERA_NODE_H

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <QPixmap>
#include <QThread>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <connected_bot_app/photoAction.h>

/*!
  Receives camera images and ir distance sensor values from ros nodes.
  Processes the image and publishes the resultc++ code documentation as QT
  signal

  Since this thread will only have a ROS event loop and no Qt event loop it can
  only emit signals and can't provides slots. So implementation a subclass
  of QThread is fine and makes the implementation simpler.
*/
class ShowRobotCamera : public QThread {
  Q_OBJECT
 public:
  //! constructor
  ShowRobotCamera();

  //! thread main loop
  void run() override;

  //! stops the ros event queue
  void quit();

  //! returns the current name of the image topic
  std::string image_topic_name();

  //! ros node initialization
  static void init(int argc, char** argv);

  // signals are thread safe but not slots
 signals:
  void newImageReady(const QPixmap& pixmap);  //! new camera image available

 private:
  //! callback for sensor messages
  void sensorCallback(const std_msgs::Float32MultiArray& msg);

  //! callback for image messages
  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  //! callback for action - save camera image to files system
  void photoActionCb(const connected_bot_app::photoGoalConstPtr& goal);

  //! do all needed image processing
  void processImage(cv::Mat& img);

  //! undistortion of the fish erye camera image
  void undistort(cv::Mat& img);

  ros::NodeHandle nh_;  //! node handle

  //! action server for saving frames to file system
  actionlib::SimpleActionServer<connected_bot_app::photoAction> as_;
  std::string action_name_;
  connected_bot_app::photoFeedback feedback_;
  connected_bot_app::photoResult result_;

  static constexpr size_t NUM_IR_SENSORS = 5;  //! number of ir distance sensors
  std::vector<float> sensor_meas_;  //! measurements from ir distance sensors

  ros::Subscriber sub_;     //! subscriber for camera images
  ros::Subscriber sub_ir_;  //! subscriber for ir distance sensor measurements

  bool save_photo_ = false;  //! set from action server to save the next frame
                             //! to the file system

  cv::Mat identity_;  //! identity matrix is often used for camera undistortion
  cv::Mat K_;         //! matrix K for fish eye camera image undistortion
  cv::Mat D_;         //! matrix D for fish eye camera image undistortion

  bool stop_thread_ = false; //! stop thread, set true by method stop()
};

#endif