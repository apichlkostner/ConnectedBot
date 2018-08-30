#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "DistanceSensor.h"

using namespace ConnectedBotFirmware;

constexpr int numDistanceSensors = 5;

ros::NodeHandle nh;
sensor_msgs::Range range_msgs[numDistanceSensors];

ros::Publisher pub_ranges[numDistanceSensors] = {
    ros::Publisher("/range/ir_range0", &range_msgs[0]),
    ros::Publisher("/range/ir_range1", &range_msgs[1]),
    ros::Publisher("/range/ir_range2", &range_msgs[2]),
    ros::Publisher("/range/ir_range3", &range_msgs[3]),
    ros::Publisher("/range/ir_range4", &range_msgs[4])};

unsigned long range_timer;

constexpr char frameid[5][21] = {"/firmware/ir_ranger0", "/firmware/ir_ranger1",
                                 "/firmware/ir_ranger2", "/firmware/ir_ranger3",
                                 "/firmware/ir_ranger4"};

DistanceSensor d_sensors[numDistanceSensors] = {
    DistanceSensor(A3), DistanceSensor(A1), DistanceSensor(A0),
    DistanceSensor(A6), DistanceSensor(A7)};

void setup() {
  nh.initNode();
  
  for (size_t i = 0; i < numDistanceSensors; i++) {
    nh.advertise(pub_ranges[i]);
    range_msgs[i].radiation_type = sensor_msgs::Range::INFRARED;
    range_msgs[i].header.frame_id = frameid[i];
    range_msgs[i].field_of_view = 0.01;
    range_msgs[i].min_range = 0.1;
    range_msgs[i].max_range = 0.8;
  }
}

void loop() {
  if ((millis() - range_timer) > 50) {
    range_timer = millis();
    for (size_t i = 0; i < numDistanceSensors; i++) {
      float dist = d_sensors[i].getDistance();
      range_msgs[i].range = max(min(80., dist), 10.);
      range_msgs[i].header.stamp = nh.now();
      pub_ranges[i].publish(&range_msgs[i]);
    }    
  }
  nh.spinOnce();
}
