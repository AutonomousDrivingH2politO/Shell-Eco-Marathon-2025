import cv2
import matplotlib.pyplot as plt
import numpy as np


def eye_bird_view(img, mtx, dist, src, d):
    ysize = img.shape[0]
    xsize = img.shape[1]
    
    undist = undistort(img, mtx, dist)

    dst = np.float32([
        (xsize - d, 0),
        (d, 0),
        (d, ysize),
        (xsize - d, ysize)
    ])

    warped, M, invM = warp_image(undist, (xsize, ysize), src, dst)
    return warped

def load_camera_calib1(sim=False):
    if not sim:
 # ――― ZED 2 K  (LEFT_CAM_HD) ―――
        # [LEFT_CAM_HD]
        fx=532.655
        fy=532.595
        cx=621.155
        cy=349.6815
        k1=-0.0435025
        k2=0.0158395
        p1=0.0012899
        p2=-0.00129192
        k3=-0.00655099
        mtx = [[fx,     0, cx],
               [0,      fy,  cy],
               [0,          0,      1]]
        # k1, k2, p1, p2, k3
        dist = [k1,
                k2,
                p1,
                p2,
                k3]
    else:
        # for the simulation
        mtx = [[1395.35, 0, 640],
                [0, 1395.35, 360],
                [0, 0, 1]]
        dist = [0, 0, 0, 0, 0]
    return np.array(mtx), np.array(dist)
def load_camera_calib(sim=False):
    if not sim:                                    # Intel RealSense D435i
        mtx  = [[914.0581, 0.0, 647.0607],
                [0.0,      912.9447, 364.1458],
                [0.0,      0.0,      1.0]]
        dist = [0, 0, 0, 0, 0]
    else:                                          # Gazebo / CARLA sim
        mtx  = [[1395.35, 0, 640],
                [0, 1395.35, 360],
                [0, 0, 1]]
        dist = [0, 0, 0, 0, 0]
    return np.array(mtx, dtype=np.float32), np.array(dist, dtype=np.float32)

def undistort(img, mtx, dist):
    '''
    Undistorts an image
    :param img (ndarray): Image, represented an a numpy array
    :param mtx: Camera calibration matrix
    :param dist: Distortion coeff's
    :return : Undistorted image
    '''
    
    undistort = cv2.undistort(img, mtx, dist, None, mtx)
    return undistort

def warp_image(img, warp_shape, src, dst):
    '''
    Performs perspective transformation (PT)
    :param img (ndarray): Image
    :param warp_shape: Shape of the warped image
    :param src (ndarray): Source points
    :param dst (ndarray): Destination points
    :return : Tuple (Transformed image, PT matrix, PT inverse matrix)
    '''
    
    # Get the perspective transformation matrix and its inverse
    M = cv2.getPerspectiveTransform(src, dst)
    invM = cv2.getPerspectiveTransform(dst, src)
    
    # Warp the image
    warped = cv2.warpPerspective(img, M, warp_shape, flags=cv2.INTER_LINEAR)
    return warped, M, invM

src = np.float32([                   # Simulation (1280, 720)
    (694.0, 450.0),
    (586.0, 450.0),
    (50.0, 675.0),
    (1230.0, 675.0)
])

img = cv2.cvtColor(cv2.imread('/home/bylogix/Shell-Eco-Marathon-2025/calibration_setup/..png'), cv2.COLOR_BGR2RGB)
img = cv2.resize(img, (1280, 720))

ysize = img.shape[0]
xsize = img.shape[1]
print(ysize, xsize)

cv2.polylines(img, [np.int32(src)], True, (255, 255, 255), 3)

cv2.imshow("Original Image", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)


mtx, dist = load_camera_calib()
mtx1, dist1 = load_camera_calib1()

d = 530
warped = eye_bird_view(img, mtx, dist, src, d=d)
warped1 = eye_bird_view(img, mtx1, dist1, src, d=d)

dst = np.float32([
    (xsize - d, 0),
    (d, 0),
    (d, ysize),
    (xsize - d, ysize)
])

#cv2.polylines(warped, [np.int32(dst)], True, (255, 0, 0), 3)
#cv2.imshow("Warped Realsense Image", cv2.cvtColor(warped, cv2.COLOR_RGB2BGR))
cv2.polylines(warped1, [np.int32(dst)], True, (255, 0, 0), 3)
cv2.imshow("Warped ZED Image", cv2.cvtColor(warped1, cv2.COLOR_RGB2BGR))

cv2.waitKey(0)
cv2.destroyAllWindows()
