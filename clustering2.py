#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool
import numpy as np
from sklearn.cluster import DBSCAN

def scan_callback(msg):
    # Convert LaserScan to PointCloud2
    header = msg.header
    ranges = msg.ranges
    angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
    points = [(ranges[i] * np.cos(angles[i]), ranges[i] * np.sin(angles[i]), 0.0) for i in range(len(ranges))]
    point_cloud = pc2.create_cloud_xyz32(header, points)

    # Perform DBSCAN clustering
    point_list = list(pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True))
    point_array = np.array(point_list)
    clustering = DBSCAN(eps=0.5, min_samples=10).fit(point_array)

    # Check clusters within 1 meters
    obstacle_detected = False
    for label in set(clustering.labels_):
        cluster_points = point_array[clustering.labels_ == label]
        if len(cluster_points) > 0:
            centroid = np.mean(cluster_points, axis=0)
            distance = np.linalg.norm(centroid[:2])
            if distance < 1.0:
                obstacle_detected = True
                break

    obstacle_pub.publish(obstacle_detected)

def main():
    rospy.init_node('obstacle_detection')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    global obstacle_pub
    obstacle_pub = rospy.Publisher('/obstacle_detection', Bool, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()

