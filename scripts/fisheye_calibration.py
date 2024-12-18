import os
import glob
import argparse

import cv2
import numpy as np

def parse_tuple(arg_string):
    # Convert a comma-separated string to a tuple of integers
    return tuple(map(int, arg_string.split(',')))

parser = argparse.ArgumentParser(description="Get a file path from the user.")

# Add an argument for the file path
parser.add_argument('path', type=str, 
                    help="The path to the folder containing the fisheye images.", 
                    default='../resource/fisheye_images')
# Add an argument for the image format
parser.add_argument('format', type=str, 
                    help="The format of the fisheye images.", 
                    default='jpg')
# dimensions of the chessboard pattern
parser.add_argument('grid_size', type=parse_tuple,
                    help="The dimensions of the chessboard pattern.",
                    default=(9, 6))
# size of a square of the chessboard
parser.add_argument('square_size', type=float,
                    help="The size of a square on your chessboard.",
                    default=1.0)
parser.add_argument('resize_factor', type=float,
                    help="The resize factor for the images.",
                    default=1.0)
args = parser.parse_args()

assert os.path.exists(args.path), "The path does not exist."
assert args.format in ['jpg', 'png', 'jpeg', 'tiff', 'bmp'], "unsupported image format"
assert args.grid_size[0] > 0 and args.grid_size[1] > 0, "grid size must be positive"
assert args.square_size > 0, "square size must be positive"
assert args.resize_factor > 0, "resize factor must be positive"

fisheye_calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def calibrate_fisheye_images():
    grid_size = args.grid_size
    square_size = args.square_size

    # Prepare object points (3D points in real world space)
    objp = np.zeros((grid_size[0] * grid_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    # Load images
    image_file_names = glob.glob(args.path + '/*.' + args.format)
    for file_name in image_file_names:
        img = cv2.imread(file_name)
        img_resized = cv2.resize(img, (0, 0), fx=args.resize_factor, fy=args.resize_factor)
        gray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)

        # Improve corner detection with cornerSubPix
        ret, corners = cv2.findChessboardCorners(gray, grid_size, 
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            objpoints.append(objp)
            # Refine corner locations
            corners_ = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix_criteria)
            imgpoints.append(corners_)

            cv2.drawChessboardCorners(img_resized, grid_size, corners, ret)
            cv2.imshow('Corners', img_resized)
            cv2.waitKey(100)

    cv2.destroyAllWindows()
    
    # Calibrate fisheye camera
    N = len(objpoints)
    print(N, "images used for calibration")
    objpoints = np.expand_dims(np.asarray(objpoints), -2)
    
    K_fisheye = np.zeros((3, 3), dtype=np.float64)
    D_fisheye = np.zeros((4, 1), dtype=np.float64)
    rvecs_fisheye = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(N)]
    tvecs_fisheye = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(N)]

    ret, K_fisheye, D_fisheye, rvecs_fisheye, tvecs_fisheye = cv2.fisheye.calibrate(
        objpoints, 
        imgpoints, 
        gray.shape[::-1],
        K_fisheye, 
        D_fisheye, 
        rvecs_fisheye, 
        tvecs_fisheye,
        flags=fisheye_calibration_flags,
        criteria=subpix_criteria)

    print("Fisheye Camera Matrix:\n", K_fisheye)
    print("Fisheye Distortion Coefficients:\n", D_fisheye)
    return K_fisheye, D_fisheye

def undistort_fisheye_images(K_fisheye, D_fisheye):
    # Load images
    image_file_names = glob.glob(args.path + '/*.' + args.format)

    # Undistort fisheye images
    for file_name in image_file_names:
        img = cv2.imread(file_name)
        img_resized = cv2.resize(img, (0, 0), fx=args.resize_factor, fy=args.resize_factor)
        h, w = img_resized.shape[:2]
        K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K_fisheye, D_fisheye, (w, h), np.eye(3), balance=1)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K_fisheye, D_fisheye, np.eye(3), K_new, (w, h), cv2.CV_16SC2)
        img_undistorted = cv2.remap(img_resized, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        directory_name = os.path.dirname(file_name)
        base_name = os.path.basename(file_name)
        name, ext = os.path.splitext(base_name)
        cv2.imwrite(os.path.join(directory_name, name + '_undistorted' + ext), img_undistorted)

def recalibrate_undistorted_images():
    grid_size = args.grid_size
    square_size = args.square_size

    # Prepare object points and image points for undistorted images
    objp = np.zeros((grid_size[0] * grid_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    # Load undistorted images
    undistorted_images = glob.glob(args.path + '/*_undistorted.' + args.format)

    for file_name in undistorted_images:
        img = cv2.imread(file_name)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, grid_size, None)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

    # Calibrate using the general pinhole camera model
    ret, K_pinhole, dist_pinhole, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, 
        imgpoints, 
        gray.shape[::-1], None, None)

    print("Pinhole Camera Matrix:\n", K_pinhole)
    print("Distortion Coefficients (k1, k2, p1, p2, k3):\n", dist_pinhole)
    return K_pinhole, dist_pinhole


if __name__=="__main__":
    K_fisheye, D_fisheye = calibrate_fisheye_images()
    undistort_fisheye_images(K_fisheye=K_fisheye, D_fisheye=D_fisheye)
    K_virtual_pinhole, D_virtual_pinhole = recalibrate_undistorted_images()