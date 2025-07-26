import numpy as np
import cv2 as cv
import glob
import os
import re


class HandEyeCalibration:
    def __init__(self, save_dir, img_dir, posefile, eye_to_hand, chessboardSize, frameSize, size_of_chessboard_squares_m):
        self.save_dir = save_dir
        # images
        self.img_dir = img_dir
        self.images = sorted(glob.glob(os.path.join(img_dir, '*.png')),
                             key=lambda x: int(re.search(r'\d+', os.path.basename(x)).group()))
        # chessboardSize and camera parameters
        self.chessboardSize = chessboardSize
        self.frameSize = frameSize
        self.size_of_chessboard_squares_m = size_of_chessboard_squares_m

        # intrinsic parameters and robot poses
        self.gripper2base = np.load(posefile)
        self.cameraMatrix = np.load(self.save_dir+"/camera_intrinsic/cameraMatrix.npy")
        self.dist = np.load(self.save_dir+"/camera_intrinsic/dist.npy")

        # eye to hand or eye in hand
        self.eye_to_hand = eye_to_hand

        # termination criteria
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # translation and rotation vectors
        # target2cam
        self.R_tar2cam = []
        self.t_tar2cam = []

        # gripper2base
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.gripper2base_matrix = []

    def get_target2cam(self):
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.chessboardSize[0] * self.chessboardSize[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:self.chessboardSize[0],0:self.chessboardSize[1]].T.reshape(-1,2)
        objp = objp * self.size_of_chessboard_squares_m

        for image in self.images:
            img = cv.imread(image)
            gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, self.chessboardSize, None)
            if ret == True:
                corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1), self.criteria)
                # Find the rotation and translation vectors. 
                # rvecs [rx, ry, rz] in rad, tvecs [x, y, z] in m(defined by size_of_chessboard_squares_m)
                ret, rvecs, tvecs = cv.solvePnP(objp, corners2, self.cameraMatrix, self.dist)
                # target2cam
                self.R_tar2cam.append(cv.Rodrigues(rvecs)[0])
                self.t_tar2cam.append(tvecs)
            else:
                print('no chessboard corners found:'+image)
                continue
        
    def get_gripper2base(self):
        # convert 6D pose to 4x4 matrix 
        # pose [x,y,z,rx,ry,rz] in m and rad
        for pose in self.gripper2base:
            #pose to homo
            R, _ = cv.Rodrigues(pose[3:])
            H_matrix = np.zeros((4, 4))
            H_matrix[:3, :3] = R
            H_matrix[:3, 3] = pose[:3]
            H_matrix[3, 3] = 1.0

            if self.eye_to_hand:
                # inverse to base2gripper https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
                H_matrix = np.linalg.inv(H_matrix)

            r = H_matrix[:3, :3]
            t = H_matrix[:3, 3]

            self.R_gripper2base.append(r)
            self.t_gripper2base.append(t)
            self.gripper2base_matrix.append(H_matrix)

    def handeye_calibrate(self):
        # return cam2base if eye to hand
        # return cam2gripper if eye in hand
        R, t = cv.calibrateHandEye(self.R_gripper2base, self.t_gripper2base, self.R_tar2cam, self.t_tar2cam, method=cv.CALIB_HAND_EYE_TSAI)
        extrinsic = np.hstack((R, t))
        extrinsic = np.vstack((extrinsic, [0, 0, 0, 1]))
        print('-------TSAI---------')
        print(extrinsic)

        R, t = cv.calibrateHandEye(self.R_gripper2base, self.t_gripper2base, self.R_tar2cam, self.t_tar2cam, method=cv.CALIB_HAND_EYE_PARK)
        extrinsic = np.hstack((R, t))
        extrinsic = np.vstack((extrinsic, [0, 0, 0, 1]))
        print('-------extrinsic PARK---------')
        print(extrinsic)

        R, t = cv.calibrateHandEye(self.R_gripper2base, self.t_gripper2base, self.R_tar2cam, self.t_tar2cam, method=cv.CALIB_HAND_EYE_HORAUD)
        extrinsic = np.hstack((R, t))
        extrinsic = np.vstack((extrinsic, [0, 0, 0, 1]))
        print('-------extrinsic HORAUD---------')
        print(extrinsic)

        R, t = cv.calibrateHandEye(self.R_gripper2base, self.t_gripper2base, self.R_tar2cam, self.t_tar2cam, method=cv.CALIB_HAND_EYE_ANDREFF)
        extrinsic = np.hstack((R, t))
        extrinsic = np.vstack((extrinsic, [0, 0, 0, 1]))
        print('-------extrinsic ANDREFF---------')
        print(extrinsic)

        R, t = cv.calibrateHandEye(self.R_gripper2base, self.t_gripper2base, self.R_tar2cam, self.t_tar2cam, method=cv.CALIB_HAND_EYE_DANIILIDIS)
        extrinsic = np.hstack((R, t))
        extrinsic = np.vstack((extrinsic, [0, 0, 0, 1]))
        print('-------extrinsic DANIILIDIS---------')
        print(extrinsic)
    
    def calibrate(self):
        self.get_target2cam()
        self.get_gripper2base()
        self.handeye_calibrate()

    def save(self, method=cv.CALIB_HAND_EYE_TSAI):
        R, t = cv.calibrateHandEye(self.R_gripper2base, self.t_gripper2base, self.R_tar2cam, self.t_tar2cam, method=method)
        extrinsic = np.hstack((R, t))
        extrinsic = np.vstack((extrinsic, [0, 0, 0, 1]))
        np.save(self.save_dir+"/extrinsic.npy", np.array(extrinsic))
        print('-------extrinsic saved---------')

