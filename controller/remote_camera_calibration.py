from PIL import Image

import cv2
import numpy
import numpy as np
import requests
import yaml

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*4,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:4].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

gray = None
found = 0

while found < 10:  # Here, 10 can be changed to whatever number you like to choose

    r = requests.get('http://192.168.1.99:8080/?action=snapshot', stream=True)
    r.raise_for_status()
    r.raw.decode_content = True  # Required to decompress gzip/deflate compressed responses

    img = Image.open(r.raw)
    gray = cv2.cvtColor(numpy.array(img), cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (6,4), None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)  # Certainly, every loop objp is the same, in 3D.

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        gray = cv2.drawChessboardCorners(gray, (6,4), corners2, ret)
        found += 1

    cv2.imshow('img', gray)
    cv2.waitKey(10)

# When everything done, release the capture
cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# It's very important to transform the matrix to list.
data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}

with open("calibration.yaml", "w") as f:
    yaml.dump(data, f)

