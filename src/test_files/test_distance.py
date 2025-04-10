#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Definizione delle coordinate del pixel di interesse
pixel_x = 100
pixel_y = 50

# Funzione di callback per l'elaborazione dell'immagine di profondità
def callback(data):
    # Conversione dell'immagine in un array NumPy
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    # Estrazione della profondità del pixel di interesse
    depth = depth_image[pixel_y][pixel_x] * 0.001 # Conversione da millimetri a metri
    # Stampa della distanza del pixel di interesse
    print("Distanza del pixel ({},{}): {:.3f} metri".format(pixel_x, pixel_y, depth))

# Funzione principale
def main():
    # Sottoscrizione al topic dell'immagine di profondità
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback)
    # Loop di ROS
    rospy.spin()

if __name__ == '__main__':
    # Inizializzazione del nodo ROS
    rospy.init_node('depth_sensor', anonymous=True)
    main()