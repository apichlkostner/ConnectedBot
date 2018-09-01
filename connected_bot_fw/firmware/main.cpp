#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32.h>
#include "DistanceSensor.h"
#include "Motor.h"
#include "RedBot.h"

using namespace ConnectedBotFirmware;

constexpr int numDistanceSensors = 5;

unsigned long range_timer;

DistanceSensor d_sensors[numDistanceSensors] = {
    DistanceSensor(A3), DistanceSensor(A1), DistanceSensor(A0),
    DistanceSensor(A6), DistanceSensor(A7)};

RedBotMotors motor;

ros::NodeHandle nh;
std_msgs::Float32MultiArray range_msg;
ros::Publisher pub_ranges("/sensor/ir", &range_msg);

std_msgs::Int32MultiArray wheel_msg;
ros::Publisher pub_wheel("/wheel/count", &wheel_msg);

void speed_cb(const std_msgs::UInt32& msg) {
  uint32_t setv = msg.data;

  // first two bytes: left
  int speed_forward_left = setv & 0xFF;
  int speed_backward_left = (setv >> 8) & 0xFF;
  if (speed_forward_left > 0)
    motor.leftDrive(speed_forward_left);
  else
    motor.leftDrive(-speed_backward_left);

  // last two bytes: right
  int speed_forward_right = (setv >> 16) & 0xFF;
  int speed_backward_right = (setv >> 24) & 0xFF;
  if (speed_forward_right > 0)
    motor.rightDrive(speed_forward_right);
  else
    motor.rightDrive(-speed_backward_right);
}

ros::Subscriber<std_msgs::UInt32> sub("/motor/speed", &speed_cb);

RedBotEncoder encoder = RedBotEncoder(A2, 10);

float farray[numDistanceSensors];
long warray[2];

void setup() {
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(pub_wheel);
  nh.advertise(pub_ranges);

  range_msg.data_length = numDistanceSensors;
  range_msg.data = farray;

  wheel_msg.data_length = 2;
  wheel_msg.data = warray;
}

void loop() {
  if ((millis() - range_timer) > 200) {
    range_timer = millis();
    for (int i = 0; i < numDistanceSensors; i++) {
      float dist = d_sensors[i].getDistance();
      range_msg.data[i] = max(min(80., dist), 10.);
    }
    pub_ranges.publish(&range_msg);

    int lCount = encoder.getTicks(LEFT);  // read the left motor encoder
    wheel_msg.data[0] = lCount;
    int rCount = encoder.getTicks(RIGHT);  // read the right motor encoder
    wheel_msg.data[1] = rCount;
    pub_wheel.publish(&wheel_msg);
  }
  nh.spinOnce();
}
