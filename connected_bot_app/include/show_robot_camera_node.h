#ifndef SHOW_ROBOT_CAMERA_NODE_H
#define SHOW_ROBOT_CAMERA_NODE_H

class ShowRobotCamera {
 public:
  ShowRobotCamera(std::string name, std::string image_topic)
      : as_(nh_, name, boost::bind(&ShowRobotCamera::photoActionCb, this, _1),
            false),
        action_name_(name),
        sensor_meas_(NUM_IR_SENSORS) {
    sub_ = nh_.subscribe(image_topic, 1, &ShowRobotCamera::imageCallback, this);
    sub_ir_ = nh_.subscribe("/sensor/ir", 1, &ShowRobotCamera::sensorCallback, this);
    as_.start();
  }

 private:
  void sensorCallback(const std_msgs::Float32MultiArray& msg);
  void processImage(cv::Mat& img);
  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
  void photoActionCb(const connected_bot_app::photoGoalConstPtr& goal);


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
};

#endif