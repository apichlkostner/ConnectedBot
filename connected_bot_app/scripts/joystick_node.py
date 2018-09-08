#!/usr/bin/env python
import rospy
import actionlib
import connected_bot_app.msg
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy

 

global val_forward
global val_sideward
global make_photo

val_forward = 0
val_sideward = 0
make_photo = 0

def callback_receive_joy(msg):
    global val_forward, val_sideward, make_photo
    val_sideward = msg.axes[0]
    val_forward  = msg.axes[1]
    make_photo = msg.buttons[0]
    

if __name__ == '__main__':
    global forward, sideward, make_photo

    rospy.init_node('connected_bot_joystick')

    pub = rospy.Publisher('/motor/speed', UInt32, queue_size=10)
    sub = rospy.Subscriber('/joy', Joy, callback_receive_joy)

    action_client = actionlib.SimpleActionClient('/make_photo', connected_bot_app.msg.photoAction)

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

        if make_photo != 0:
            goal = connected_bot_app.msg.photoGoal(make_photo=1)
            action_client.send_goal(goal)
            action_client.wait_for_result()
            make_photo = 0

        rate.sleep()
    