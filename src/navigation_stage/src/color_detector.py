# -*- coding: utf-8 -*-
# from __future__ import print_function
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32


UMBRAL_PIXELS = 100

class ColorDetector:
    def __init__(self):
        # Nos suscribimos los topics necesarios
        self.bridge = cv_bridge.CvBridge()
        self.pub_color = rospy.Publisher('/color_detected', Int32, queue_size=5)
        self.image_sub = rospy.Subscriber('/image', Image, self.image_callback)

    def image_callback(self, msg):
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Pasamos la imagen de RGB a HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Definimos los rangos de color
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        lower_orange = np.array([10, 100, 20])
        upper_orange = np.array([25, 255, 255])
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])
        lower_pink = np.array([150,10,75])
        upper_pink = np.array([344,255,255])

        # Creamos mascaras para cada color
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)
        
        # Combinamos las máscaras
        mask_combined = cv2.bitwise_or(mask_red, mask_green)
        mask_combined = cv2.bitwise_or(mask_combined, mask_orange)
        mask_combined = cv2.bitwise_or(mask_combined, mask_blue)
        mask_combined = cv2.bitwise_or(mask_combined, mask_pink)

        # Mostramos la máscara combinada
        cv2.imshow("Image window", mask_combined)
        cv2.waitKey(3)

        # Contamos los píxeles para cada color
        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        orange_pixels = cv2.countNonZero(mask_orange)
        blue_pixels = cv2.countNonZero(mask_blue)
        pink_pixels = cv2.countNonZero(mask_pink)

        # Determinamos que color tiene más píxeles, así, en caso de haber
        # dos colores, solo nos quedamos con el que este mas cerca
        max_color = max(red_pixels, green_pixels, orange_pixels, blue_pixels, pink_pixels)

        # Publicamos en el topic el color que mas pixeles tenga
        if max_color < UMBRAL_PIXELS:
            color_final = 0

        elif max_color>UMBRAL_PIXELS:
            if max_color == red_pixels:
                color_final = 1
            elif max_color == green_pixels:
                color_final = 2
            elif max_color == orange_pixels:
                color_final = 3
            elif max_color == blue_pixels:
                color_final = 4
            elif max_color == pink_pixels:
                color_final = 5

        self.pub_color.publish(color_final)


rospy.init_node('color_detector')
cd  = ColorDetector()
rospy.spin()   