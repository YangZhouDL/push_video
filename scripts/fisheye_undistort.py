#!/usr/bin/env python
import cv2
import numpy as np
import glob

def undistort(img_path,K,D,DIM,scale=1.0,imshow=False):
    img = cv2.imread(img_path)
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if dim1[0]!=DIM[0]:
        img = cv2.resize(img,DIM,interpolation=cv2.INTER_AREA)
    Knew = K.copy()
    if scale:#change fov
        Knew[(0,1), (0,1)] = scale * Knew[(0,1), (0,1)]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), Knew, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    if imshow:
        cv2.imshow("undistorted", undistorted_img)
    return undistorted_img

if __name__ == '__main__':

    DIM=(848, 800)
    K=np.array([[286.1809997558594, 0.0, 421.6383972167969,], [0.0, 286.3576965332031, 403.9013977050781], [0.0, 0.0, 1.0]])
    D=np.array([[-0.008326118811964989], [0.04620290920138359], [-0.04403631016612053], [0.00837636087089777]])

    img = undistort('../image/raw.jpg',K,D,DIM)
    cv2.imwrite('../image/processed.jpg', img)