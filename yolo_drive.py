#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np
from erp_driver.msg import erpCmdMsg

class Yolo_drive:
    def __init__(self):
        rospy.init_node("Yolo_drive_node")
        self.sub = rospy.Subscriber("/usb_cam_2/image_raw/compressed", CompressedImage, self.image_callback)
        self.cmd_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size=10)

        self.bridge = CvBridge()  
        self.current_speed = 30

        # YOLOv5 load
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt 있는 경로')
        self.class_names = ['1', '10', '11', '12', '13', '14', '15', '3', '4',
                            '5', '6', '7', '8', '9', 'Green', 'red', 'yellow'] 

    def image_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        detections = self.yolo.detect(cv_image)
        
        for detection in detections:
            label = detection['label']
            if label == '5':
                self.current_speed = 30
                self.publish_speed()
            elif label == '6':
                self.increase_brake()
            elif label == '7':
                self.current_speed = 30
                self.publish_speed()
            elif label == 'Green':
                self.accelerate_speed(55, 5)
            elif label == 'Red':
                self.increase_brake()

    def increase_brake(self):
        # Increase the brake value from 1 to 33 over 1.5 seconds
        for brake_value in range(1, 34):
            self.brake = brake_value
            self.publish_speed()
            rospy.sleep(1.5 / 33)

    def publish_speed(self):
        speed_msg = erpCmdMsg()
        speed_msg.speed = self.current_speed
        self.speed_pub.publish(speed_msg)

    def accelerate_speed(self, target_speed, duration):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.current_speed = target_speed
            self.publish_speed()
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        yolo_node = Yolo_drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
