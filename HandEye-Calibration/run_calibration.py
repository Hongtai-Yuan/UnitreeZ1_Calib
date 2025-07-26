from Intrinsic_calibration import IntrinsicCalibration
from handeye_calibration import HandEyeCalibration
import cv2 as cv
import os


def main(chessboardSize, size_of_chessboard_squares_m, eye_to_hand,
         frameSize=(480, 640), save_dir="handeye_img_pose"):
    # —— 确保所有需要的子目录都已存在 —— #
    os.makedirs(save_dir, exist_ok=True)
    os.makedirs(os.path.join(save_dir, 'img'), exist_ok=True)
    os.makedirs(os.path.join(save_dir, 'camera_intrinsic'), exist_ok=True)
    os.makedirs(os.path.join(save_dir, 'handeye'), exist_ok=True)

    # calibrate camera
    cam_calib = IntrinsicCalibration(
        save_dir=save_dir,
        img_dir=os.path.join(save_dir, 'img'),
        chessboardSize=chessboardSize,
        frameSize=frameSize,
        size_of_chessboard_squares_m=size_of_chessboard_squares_m
    )
    cam_calib.calibrate()
    # cam_calib.draw_pose()

    # calibrate handeye
    handeye_calib = HandEyeCalibration(
        save_dir=save_dir,
        img_dir=os.path.join(save_dir, 'img'),
        posefile=os.path.join(save_dir, 'pose.npy'),
        eye_to_hand=eye_to_hand,
        chessboardSize=chessboardSize,
        frameSize=frameSize,
        size_of_chessboard_squares_m=size_of_chessboard_squares_m
    )
    handeye_calib.calibrate()
    # 将手眼标定结果保存到 save_dir/handeye 下
    handeye_calib.save(method=cv.CALIB_HAND_EYE_TSAI)


if __name__ == '__main__':
    # change the sizes according to your chess board
    chessboardSize = (7, 10)
    size_of_chessboard_squares_m = 0.024
    eye_to_hand = True

    main(chessboardSize, size_of_chessboard_squares_m, eye_to_hand)
