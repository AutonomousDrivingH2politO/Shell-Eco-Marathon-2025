import cv2
import matplotlib.pyplot as plt
import numpy as np
import time

# v8 - oltre alle modifiche di v5, (situazioni critiche), il cambio della longitudinal distance avviene 
#       per il prossimo frame se la longitudinal distance è maggiore di una threshold, ciò significa che
#       ci stiamo avvicinando ad una curva      
#       E' stato aggiunto una funzione in grado di rimuovere i punti spuri dei bordi
#       Fixati alcuni errori su v4 e resa migliore la rimozione dei bordi spuri
#       Eliminati i fake edges.
#       Aggiunta una funzione in grado di calcolare la curvatura della traiettoria (da migliorare con anomaly detection)
#       Modifica della longitudinal distance quando ci approcciamo ad una curva

SIMULATION = True
LANE_METERS = 12
Y_METERS = { 10.0 : 565,
             7.5 : 565
            }
LANE_PIXELS = None
LATERAL_DISTANCE = 0
scale_factor = None
black_regions = None
y_black = None
prev_curvature = None

def load_camera_calib(sim=True):
    if not sim:
        # for the D435i camera
        mtx = [[914.05810546875, 0.0, 647.0606689453125],
            [0.0, 912.9447021484375, 364.1457824707031],
            [0.0, 0.0, 1.0 ]]
        dist = [0.0, 0.0, 0.0, 0.0, 0.0]
    else:
        # for the simulation
        mtx = [[1395.35, 0, 640],
                [0, 1395.35, 360],
                [0, 0, 1]]
        dist = [0, 0, 0, 0, 0]
    return np.array(mtx), np.array(dist)

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
    M = cv2.getPerspectiveTransform(src, dst)
    invM = cv2.getPerspectiveTransform(dst, src)
    
    warped = cv2.warpPerspective(img, M, warp_shape, flags=cv2.INTER_CUBIC)
    return warped, M, invM

def eye_bird_view(img, mtx, dist, d=530):
    ysize = img.shape[0]
    xsize = img.shape[1]
    
    undist = undistort(img, mtx, dist)
    src = np.float32([                   # ROI Rectangle
            (694.0, 375.0),
            (586.0, 375.0),
            (50.0, 675.0),
            (1230.0, 675.0)
        ])
    dst = np.float32([
        (xsize - d, 0),
        (d, 0),
        (d, ysize),
        (xsize - d, ysize)
    ])

    warped, _, _ = warp_image(undist, (xsize, ysize), src, dst)
    return warped

def processing_mask(mask, img, show=False, d=530):
    global black_regions, y_black
    mtx, dist = load_camera_calib(sim=SIMULATION)
    warped = eye_bird_view(mask, mtx, dist, d=d)

    if black_regions is None:
        img_warped = eye_bird_view(img, mtx, dist, d=d)
        black_regions = cv2.inRange(img_warped, np.array([0, 0, 0]), np.array([0, 0, 0]))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        black_regions = cv2.dilate(black_regions, kernel, iterations=1)
        y_black = np.min(np.nonzero(black_regions[:, 0] == 255))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (14, 14))
    res_morph = cv2.morphologyEx(warped, cv2.MORPH_CLOSE, kernel)
    
    _, res_morph_th = cv2.threshold(res_morph, 0, 255, cv2.THRESH_BINARY)
    line_edges = cv2.Canny(res_morph_th, 100, 100)
    
    vertical_edges = np.zeros_like(line_edges)
    vertical_edges[:, [0, -1]] = 255

    combined_edges = cv2.bitwise_and(warped, vertical_edges)
    _, combined_edges = cv2.threshold(combined_edges, 0, 255, cv2.THRESH_BINARY)
    line_edges -= black_regions
    line_edges = cv2.bitwise_or(line_edges, combined_edges)
    if any(combined_edges[[y_black, y_black-10], 0] == 255):
        line_edges[:, 0] = 255
    elif any(combined_edges[[y_black, y_black-10], -1] == 255):
        line_edges[:, -1] = 255
    _, line_edges = cv2.threshold(line_edges, 2, 255, cv2.THRESH_BINARY)
    if show:
        # fig, ax = plt.subplots(1, 2)
        # ax[0].imshow(combined_edges)
        plt.imshow(line_edges)
        plt.show()
    return line_edges

def merge_close_edges(lst, tol=10):
    result = []
    temp = []
    for x in lst:
        if temp and abs(x - temp[0]) > tol:  # Early temp clearing
            if len(temp) == 1:
                result.append(temp[0])
            else:
                result.append(sum(temp) // len(temp))  # Avoid np.sum for efficiency
            temp = []
        temp.append(x)

    if temp:  # Handle remaining elements after loop
        result.append(sum(temp) // len(temp))  # Avoid np.sum for efficiency
    return result


def computing_mid_point(line_edges, y):
    white_pixels = np.nonzero(line_edges[y, :])[0]
    white_pixels = merge_close_edges(white_pixels)
    if len(white_pixels) == 0:
        return None
    elif len(white_pixels) == 1:
        if LANE_PIXELS is not None:
            white_pixels = np.nonzero(line_edges[y, :])[0]
            if white_pixels[0] > line_edges.shape[1]//2:
                x_coords_points = white_pixels[0]-LANE_PIXELS, white_pixels[0]
            else:
                x_coords_points = white_pixels[0], white_pixels[0]+LANE_PIXELS
        else:
            return None
    elif len(white_pixels) == 2 and (0 in white_pixels or line_edges.shape[1]-1 in white_pixels):
        if 0 in white_pixels and white_pixels[-1] >= line_edges.shape[1]//2:
            x_coords_points = white_pixels[-1]-LANE_PIXELS, white_pixels[-1]
        elif 0 in white_pixels:
            return -np.inf
        if line_edges.shape[1]-1 in white_pixels and white_pixels[0] <= line_edges.shape[1]//2:
            x_coords_points = white_pixels[0], white_pixels[0]+LANE_PIXELS
        elif line_edges.shape[1]-1 in white_pixels :
            return +np.inf

    elif len(white_pixels) >= 2:
        max_diff = float('-inf')
        max_diff_indices = None
        
        for i in range(len(white_pixels) - 1):
            diff = abs(white_pixels[i] - white_pixels[i+1])
            if diff > max_diff:
                max_diff = diff
                max_diff_indices = (i, i+1)
        x_coords_points = white_pixels[max_diff_indices[0]], white_pixels[max_diff_indices[1]]
        # x_coords_points = white_pixels[0], white_pixels[-1]
    else:
        x_coords_points = white_pixels[0], white_pixels[1]
    return x_coords_points 

def computing_mid_pointS(line_edges, y, th_y=300, n_point=6):
    y_values = [int(x) for x in np.linspace(th_y, y, n_point)[:-1]]
    midpoints = []
    for y_act in y_values:
        x_coords_points = computing_mid_point(line_edges, y_act)
        if x_coords_points is not None and x_coords_points != +np.inf and x_coords_points != -np.inf:
            posm = y_act, (x_coords_points[1] + x_coords_points[0])//2
            midpoints.append(posm)      
    return midpoints

def computing_delta(midpoints, th_straight=20):
    global prev_curvature
    midpoints = np.array(midpoints)
    next_point = midpoints[-1]

    x = midpoints[:, 1] 
    x_mean = np.mean(x)
    x_stdev = np.sqrt((np.var(x)))
    midpoints = np.stack([p for p in midpoints if abs(p[1] - x_mean) < 1.5*x_stdev])
    if next_point[1] not in midpoints[:, 1]:
        midpoints = np.vstack((midpoints, next_point))

    # midpoints = np.array(normal_points)

    delta_x = next_point[1] - midpoints[:, 1]

    mean_delta_x = np.mean(delta_x)
    print("\t ---------- \t")
    print('Sum of delta_x =', -mean_delta_x)

    if prev_curvature is not None:
        if (prev_curvature == 'left' and mean_delta_x < 0) or (prev_curvature == 'right' and mean_delta_x > 0):
            return prev_curvature, midpoints

    if abs(mean_delta_x) < th_straight:
        curvature = 'straight'
    elif mean_delta_x > 0:
        curvature = 'left'
    elif mean_delta_x < 0:
        curvature = 'right'

    prev_curvature = curvature

    print(f"{curvature = }")
    print(f"{midpoints = }")
    return curvature, midpoints

def computing_lateral_distance(line_edges, show=False):
    global LANE_PIXELS
    global LATERAL_DISTANCE
    global scale_factor
    if prev_curvature is not None:
        if prev_curvature != "straight":
            y = Y_METERS[7.5]
            long_dist = 7.5
        else:
            y = Y_METERS[7.5]
            long_dist = 7.5
    else:
        y = Y_METERS[7.5]
        long_dist = 7.5
    x_coords_points = computing_mid_point(line_edges, y)

    if x_coords_points is None:
        return LATERAL_DISTANCE, long_dist, None, None
    if x_coords_points == -np.inf:
        return -np.inf, long_dist, None, None
    elif x_coords_points == np.inf:
        return np.inf, long_dist, None, None

    posm = y, (x_coords_points[1] + x_coords_points[0])//2

    middle_image = line_edges.shape[1]//2
    lateral_distance = posm[1] - middle_image
    if not LANE_PIXELS:
        LANE_PIXELS = x_coords_points[1] - x_coords_points[0]
        scale_factor = LANE_METERS / LANE_PIXELS

    later_distance_meters = lateral_distance * scale_factor
    LATERAL_DISTANCE = later_distance_meters

    midpoints = computing_mid_pointS(line_edges, y)
    midpoints.append(posm)
    # if len(midpoints) > 1:
    #     curvature, midpoints = computing_delta(midpoints)
    # else:
    #     curvature = None                # OCCHIO! Non può uscire come None, mettere la precedente

    if show:
        for p in midpoints:
            cv2.circle(line_edges, p[::-1], 2, (255, 255, 255), 2)
        plt.imshow(line_edges)
        plt.show()

    return later_distance_meters, long_dist, midpoints
