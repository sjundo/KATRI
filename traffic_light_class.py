#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np

class TrafficLightDetectionNode:
    def __init__(self):
        rospy.init_node("traffic_detection_node")
        self.sub = rospy.Subscriber("/usb_cam_2/image_raw/compressed", CompressedImage, self.img_CB)
        self.pub = rospy.Publisher("/traffic_light_class", String, queue_size=10)

        self.bridge = CvBridge()  # Create CvBridge instance

        # YOLOv5 load
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/shj/yolov5/runs/train/v1_result/weights/best.pt')  # Replace with your path
        self.class_names = ["Traffic_Light_Red", "Traffic_Light_Red_Left", "Traffic_Light_Green",
                            "Traffic_Light_Green_Left", "Traffic_Light_Yellow"]
        self.image_msg = CompressedImage()

    def img_CB(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # detect by YOLOv5
        results = self.model(image)

        # Publish traffic light detection result
        detected_classes = []
        for result in results.pred[0]:
            class_idx = int(result[5])
            class_name = self.class_names[class_idx]
            detected_classes.append(class_name)

        # Publish detected classes as a single string
        detected_class_str = ", ".join(detected_classes)
        self.pub.publish(detected_class_str)

def main():
    name = TrafficLightDetectionNode()
    rospy.spin()

if __name__ == '__main__':
    main()
