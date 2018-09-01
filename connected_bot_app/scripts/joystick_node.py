#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy

global val_forward
global val_sideward

val_forward = 0
val_sideward = 0

def callback_receive_joy(msg):
    global val_forward, val_sideward
    val_sideward = msg.axes[0]
    val_forward  = msg.axes[1]
    

if __name__ == '__main__':
    global forward, sideward

    rospy.init_node('connected_bot_joystick')

    pub = rospy.Publisher('/motor/speed', UInt32, queue_size=10)
    sub = rospy.Subscriber('/joy', Joy, callback_receive_joy)

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = UInt32()
        if abs(val_forward) < 0.05:
            msg.data = 0
        elif val_forward > 0:            
            fw = min(val_forward * 255., 255.)
            ratio = (val_sideward + 1.) / 2.
            fwr = int(ratio * fw)
            fwl = int((1. - ratio) * fw)
            
            msg.data = fwl  | fwr << 16
        else:
            fw = min(-val_forward * 255., 255.)
            ratio = (val_sideward + 1.) / 2.
            fwl = int(ratio * fw)
            fwr = int((1. - ratio) * fw)
            msg.data = fwl << 8 | fwr << 24

        pub.publish(msg)
        rate.sleep()
    