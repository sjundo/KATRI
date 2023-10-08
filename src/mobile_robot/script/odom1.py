#!/usr/bin/env python3

import rospy
from erp_driver.msg import erpStatusMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from sensor_msgs.msg import Imu
import tf2_ros

rospy.init_node('odometry_publisher')

# Constants for encoder and wheel properties
TICKS_PER_REVOLUTION = 100  # Number of encoder ticks per wheel revolution
WHEEL_RADIUS = 0.27  # Wheel radius in meters
WHEEL_BASE = 0.97  # Wheelbase (distance between two wheels) in meters

# Initialize variables
prev_encoder_left = 0
prev_time = rospy.Time.now()

x = 0.0
y = 0.0
theta = 0.0

odom_pub = rospy.Publisher('/wheel/odom', Odometry, queue_size=10)

# TF2 broadcaster for imu_link to base_link transform
tf_broadcaster = tf2_ros.TransformBroadcaster()

def encoder_callback(msg):
    global prev_encoder_left, prev_time, x, y, theta

    # Extract encoder data (assuming only left encoder is used)
    encoder_left = msg.encoder

    # Calculate delta encoder values
    delta_left = encoder_left - prev_encoder_left

    # Update previous encoder value
    prev_encoder_left = encoder_left

    # Calculate time difference
    current_time = rospy.Time.now()
    dt = (current_time - prev_time).to_sec()
    prev_time = current_time

    # Calculate linear and angular velocities
    delta_s = delta_left * (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REVOLUTION
    linear_x = delta_s / dt

    # Integrate linear velocity to update position
    x += linear_x * math.cos(theta) * dt
    y += linear_x * math.sin(theta) * dt

    # Create Odometry message
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y

    # Convert yaw (theta) to quaternion
    quaternion = quaternion_from_euler(0, 0, theta)
    odom.pose.pose.orientation = Quaternion(*quaternion)

    # Set linear velocity (angular velocity is 0 since we're not using right encoder)
    odom.twist.twist.linear.x = linear_x
    odom.twist.twist.angular.z = 0.0

    # Publish the Odometry message
    odom_pub.publish(odom)

    # Create and broadcast the TF transform
    transform = TransformStamped()
    transform.header.stamp = current_time
    transform.header.frame_id = 'imu_link'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation = Quaternion(*quaternion)
    tf_broadcaster.sendTransform(transform)

def imu_callback(msg):
    global theta

    # Extract yaw from IMU message (assuming it's in radians)
    yaw = msg.orientation.z

    # Convert yaw to the range -pi to pi
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi

    # Update the orientation (theta) based on IMU yaw
    theta = -yaw

# Subscribe to encoder and IMU messages
rospy.Subscriber('/erp42_status', erpStatusMsg, encoder_callback)
rospy.Subscriber('/imu/data', Imu, imu_callback)

rospy.spin()
