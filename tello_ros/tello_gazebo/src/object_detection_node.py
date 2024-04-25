#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from tello_msgs.msg import GateData
from cv_bridge import CvBridge


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        qos_profile = QoSProfile(depth=10, durability=rclpy.qos.DurabilityPolicy.VOLATILE, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, qos_profile)
        self.cv_bridge = CvBridge()
        self.publisher = self.create_publisher(GateData, '/drone1/gates', 1)

    def image_callback(self, msg):
        # Define the lower and upper bounds of the gate color in HSV
        lower_r = np.array([0, 60, 60])
        upper_r = np.array([30, 255, 255])
        lower_b = np.array([90, 40, 40])
        upper_b = np.array([160, 255, 255])
        lower_g = np.array([40, 60, 60])
        upper_g = np.array([80, 255, 255])

        print("Received an image")
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_r, gate_r, distance_r, ratio_r, height_diff_r = self.detect_gates(cv_image, lower_r, upper_r, "red")
        image_b, gate_b, distance_b, ratio_b, height_diff_b = self.detect_gates(cv_image, lower_b, upper_b, "blue")
        image_g, gate_g, distance_g, ratio_g, height_diff_g = self.detect_gates(cv_image, lower_g, upper_g, "green")
        
        msg = GateData()
        if gate_r == None:
            msg.red_detected = False
        else:
            msg.red_detected = True
            msg.red_data = gate_r + [distance_r] + [ratio_r] + [height_diff_r]
        
        if gate_b == None:
            msg.blue_detected = False
        else:
            msg.blue_detected = True
            msg.blue_data = gate_b + [distance_b] + [ratio_b] + [height_diff_b]
        
        if gate_g == None:
            msg.green_detected = False
        else:
            msg.green_detected = True
            msg.green_data = gate_g + [distance_g] + [ratio_g] + [height_diff_g]
            
        self.publisher.publish(msg)
        print(msg)

        # Display gates for debugging purposes
        cv_image[image_r==255] = (0,0,255)
        cv_image[image_b==255] = (255,0,0)
        cv_image[image_g==255] = (0,255,0)
        cv2.imshow('Detected gates', cv_image)
        cv2.waitKey(3)

    def detect_gates(self, image, lower, upper, name):
        gate_center = None

        # Convert the image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get mask
        mask = cv2.inRange(hsv_image, lower, upper)

        # Inverse mask
        mask_i = cv2.bitwise_not(mask)

        # Find contours in masks
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Select masked areas in images
        imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_masked = cv2.bitwise_and(imgray,imgray, mask=mask)
        
        # Fill masked areas
        for cnt in contours:
            cv2.drawContours(image_masked,[cnt],0,255,-1)

        # Keep only those areas (holes), that are not in original masks
        image_masked = cv2.bitwise_and(image_masked, image_masked, mask=mask_i)
        contours, _ = cv2.findContours(image_masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour
        distance = 0
        ratio = 0
        height_diff = 0
        image_masked[:] = 0
        if (len(contours) > 0):
            largest_cnt = max(contours, key=cv2.contourArea)
            
            # Ignore extremely small contours
            if (cv2.contourArea(largest_cnt) >= 500):
                cv2.drawContours(image_masked,[largest_cnt],0,255,-1)
                M = cv2.moments(image_masked)
                
                # Find contour center point in relation to image so that (0,0) is middle
                cX = int(M["m10"] / M["m00"]) - int(image.shape[1]/2)
                cY = int(M["m01"] / M["m00"]) - int(image.shape[0]/2)
                gate_center = [cX, cY]
                print(f"Center {name}: {gate_center[0]}, {gate_center[1]}")
                
                # Determine distance from the width of largest contour
                _, _, width, height = cv2.boundingRect(largest_cnt)
                print("width: ",width,"height: ",height)
                # Width of gate = 1000 units
                distance = 1000 / ( 2 * np.tan((width * np.pi)/(960 * 3 * 2)))
                distance = int(distance)
                ratio = width * 1000 / height
                ratio = int(ratio)
                
                #test code for detecting where the ratio went wrong
                if ratio < 750:
                    try:
                        p = cv2.arcLength(largest_cnt, True) # largest_cnt is the rect Contours
                        appr = cv2.approxPolyDP(largest_cnt, 0.02*p, True) # appr contains the 4 points
                    
                        appr = sorted(appr, key=lambda c: c[0][0])
                
                        #pa = top lef point
                        #pb = bottom left point
                        #pc = top right point
                        #pd = bottom right point

                        pa, pb = sorted(appr[:2], key=lambda c: c[0][1])
                        pc, pd = sorted(appr[2:], key=lambda c: c[0][1])

                        # the points are x, y in list of list [[x, y]]
                        # contour sides heights
                        height_left_side = abs(pa[0][1] - pb[0][1])
                        height_right_side = abs(pc[0][1] - pd[0][1])
                
                        height_diff = int(height_left_side - height_right_side)
                    except:
                        pass
        
        return image_masked, gate_center, distance, ratio, height_diff

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        pass
    
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
