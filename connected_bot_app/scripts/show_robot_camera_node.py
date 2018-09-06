#!/usr/bin/env python
import glob
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from CameraCalibration import CalibrateCamera
import numpy as np

class image_subscriber:
    def __init__(self, calCam=None):
        self.calCam = calCam
        self.image_sub = rospy.Subscriber(
            "/webcam/image_raw/compressed", CompressedImage, self.callback)
        self.img = None

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self.calCam is not None:
            self.img = self.calCam.undistort(img)

    def show_image(self):
        if self.img is not None:
            cv2.imshow("Connected_Bot_Camera", self.img)
            cv2.waitKey(100)


def main(args):
    #camera calibration
    calCam = CalibrateCamera.load()

    # if no precalculated data is available start calculation
    if calCam == None:
        images = glob.glob('data/camera_cal/*.jpg')
        calCam = CalibrateCamera()
        calCam.findCorners(images, (9, 6))
        calCam.calibrateCamera()
        calCam.write()

    # create subscriber class
    ims = image_subscriber(calCam)

    rospy.init_node('show_robot_camera')

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ims.show_image()
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
