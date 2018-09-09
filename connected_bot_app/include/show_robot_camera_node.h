#ifndef SHOW_ROBOT_CAMERA_NODE_H
#define SHOW_ROBOT_CAMERA_NODE_H

class ShowRobotCamera {
 public:
  ShowRobotCamera(std::string name, std::string image_topic)
      : as_(nh_, name, boost::bind(&ShowRobotCamera::photoActionCb, this, _1),
            false),
        action_name_(name),
        sensor_meas_(NUM_IR_SENSORS) {
    D_ = (cv::Mat_<double>(4, 1) << -0.03213332, 0.02991439, -0.07085706,
          0.04153998);
    K_ = (cv::Mat_<double>(3, 3) << 331.46297565, 0., 305.05513925, 0.,
          330.99605545, 225.88179462, 0., 0., 1.);
    //std::cout << "D = " << D_ << "\nK = " << K_ << std::endl;
    identity_ = cv::Mat::eye(3, 3, CV_64F);
    sub_ = nh_.subscribe(image_topic, 1, &ShowRobotCamera::imageCallback, this);
    sub_ir_ =
        nh_.subscribe("/sensor/ir", 1, &ShowRobotCamera::sensorCallback, this);
    as_.start();
  }

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