#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include "DistanceSensor.h"
#include "RedBot.h"

using namespace ConnectedBotFirmware;

constexpr int numDistanceSensors = 5;

ros::NodeHandle nh;
sensor_msgs::Range range_msg;

ros::Publisher pub_ranges[numDistanceSensors] = {
    ros::Publisher("/range/ir_range0", &range_msg),
    ros::Publisher("/range/ir_range1", &range_msg),
    ros::Publisher("/range/ir_range2", &range_msg),
    ros::Publisher("/range/ir_range3", &range_msg),
    ros::Publisher("/range/ir_range4", &range_msg)};

std_msgs::Int16 wheel_msg;
ros::Publisher pub_wheel[2] = {
    ros::Publisher("/wheel/left", &wheel_msg),
    ros::Publisher("/wheel/right", &wheel_msg),
};

unsigned long range_timer;

constexpr char frameid[5][17] = {"/range/ir_range0", "/range/ir_range1",
                                 "/range/ir_range2", "/range/ir_range3",
                                 "/range/ir_range4"};

DistanceSensor d_sensors[numDistanceSensors] = {
    DistanceSensor(A3), DistanceSensor(A1), DistanceSensor(A0),
    DistanceSensor(A6), DistanceSensor(A7)};

RedBotEncoder encoder = RedBotEncoder(A2, 10);

void setup() {
  nh.initNode();

  nh.advertise(pub_wheel[0]);
  nh.advertise(pub_wheel[1]);

  for (size_t i = 0; i < numDistanceSensors; i++) {
    nh.advertise(pub_ranges[i]);
  }
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id = frameid[0];
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.1;
  range_msg.max_range = 0.8;
}

void loop() {
  if ((millis() - range_timer) > 50) {
    range_timer = millis();
    for (size_t i = 0; i < numDistanceSensors; i++) {
      float dist = d_sensors[i].getDistance();
      range_msg.range = max(min(80., dist), 10.);
      range_msg.header.stamp = nh.now();
      pub_ranges[i].publish(&range_msg);
    }

    int lCount = encoder.getTicks(LEFT);  // read the left motor encoder
    wheel_msg.data = lCount;
    pub_wheel[0].publish(&wheel_msg);
    int rCount = encoder.getTicks(RIGHT);  // read the right motor encoder
    wheel_msg.data = rCount;
    pub_wheel[1].publish(&wheel_msg);
  }
  nh.spinOnce();
}
