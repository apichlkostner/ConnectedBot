#!/usr/bin/env python
import numpy as np
import cv2
import glob
import pickle
import os.path
import logging
import matplotlib.pyplot as plt


class CalibrateCamera():

    def __init__(self):
        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d points in real world space
        self.imgpoints = []  # 2d points in image plane.
        self.img_size = (0, 0)
        self.corner_size = (0, 0)
        self.mtx = None
        self.dist = None
        self.rvecs = None
        self.tvecs = None
        self.FISHEYE = True

    def findCorners(self, images, corner_size):
        self.corner_size = corner_size
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros(
            (1, self.corner_size[0] * self.corner_size[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:self.corner_size[0],
                               0:self.corner_size[1]].T.reshape(-1, 2)

        subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

        # Step through the list and search for chessboard corners
        for idx, fname in enumerate(images):
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            self.img_size = (img.shape[1], img.shape[0])

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(
                gray, self.corner_size, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

            # If found, add object points, image points
            if ret:
                print("Found corners in " + fname)
                self.objpoints.append(objp)
                cv2.cornerSubPix(gray, corners, (3, 3),
                                 (-1, -1), subpix_criteria)
                self.imgpoints.append(corners)
                #debug_img = cv2.drawChessboardCorners(img, corner_size, corners, ret)
                #cv2.imwrite('debug_images/' + fname, debug_img)

    def calibrateCamera(self):
        # Do camera calibration given object points and image points
        if (self.FISHEYE):
            self.K = np.zeros((3, 3))
            self.D = np.zeros((4, 1))
            ret, self.K, self.D, self.rvecs, self.tvecs = cv2.fisheye.calibrate(
                self.objpoints, self.imgpoints, self.img_size, self.K, self.D)
        else:
            ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, self.img_size, None, None)

        return self.mtx, self.dist

    def undistort(self, img):
        h,  w = img.shape[:2]

        if self.FISHEYE:
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, np.eye(3), self.K, img.shape[:2], cv2.CV_16SC2)
            undist = cv2.remap(
                img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        else:
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.dist, (w, h), 1, (w, h))
            undist = cv2.undistort(
                img, self.mtx, self.dist, None, newcameramtx)

        return undist

    def write(self, filename='data/obj_save/CameraCalibration.p'):
        directory = os.path.dirname(filename)

        if not os.path.exists(directory):
            os.mkdir(directory)

        with open(filename, 'wb') as pf:
            pickle.dump(self, pf)

    @staticmethod
    def load(filename='data/obj_save/CameraCalibration.p'):
        loaded = None
        if os.path.isfile(filename):
            with open(filename, 'rb') as pf:
                logging.info('Loading CalibrateCamera from ' + filename)
                loaded = pickle.load(pf)

        return loaded


def main():
    logging.basicConfig(level=logging.DEBUG)

    os.chdir('/home/arthur/catkin_ws/src/connected_bot_app/scripts')

    # load precalulated calibration data if available
    #calCam = CalibrateCamera.load()
    calCam = None

    # if no precalculated data is available start calculation
    if calCam is None:
        images = glob.glob('data/camera_cal/calibrate*.jpg')

        calCam = CalibrateCamera()

        calCam.findCorners(images, (9, 6))

        calCam.calibrateCamera()

        calCam.write()

    # for debuggind and documentation
    debug_img = cv2.imread('data/camera_cal/2018-09-05-135232.jpg')
    debug_img_undist = calCam.undistort(debug_img)
    cv2.imwrite(
        'data/debug_images/2018-09-05-135232_undist.jpg', debug_img_undist)

    print(calCam.D)
    print(calCam.K)


if __name__ == "__main__":
    main()
