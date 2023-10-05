#!/usr/bin/env python3

import rospy
from erp42_driver.msg import erpStatusMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

rospy.init_node('odometry_publisher')

# Constants for encoder and wheel properties
TICKS_PER_REVOLUTION = 100  # Number of encoder ticks per wheel revolution
WHEEL_RADIUS = 0.27  # Wheel radius in meters
WHEEL_BASE = 0.97  # Wheelbase (distance between two wheels) in meters

# Initialize variables
prev_encoder_left = 0
prev_encoder_right = 0
prev_time = rospy.Time.now()

x = 0.0
y = 0.0
theta = 0.0

odom_pub = rospy.Publisher('wheel/odom', Odometry, queue_size=10)

def encoder_callback(msg):
    global prev_encoder_left, prev_encoder_right, prev_time, x, y, theta

    # Extract encoder data
    encoder_left = msg.encoder_left
    encoder_right = msg.encoder_right

    # Calculate delta encoder values
    delta_left = encoder_left - prev_encoder_left
    delta_right = encoder_right - prev_encoder_right

    # Update previous encoder values
    prev_encoder_left = encoder_left
    prev_encoder_right = encoder_right

    # Calculate time difference
    current_time = rospy.Time.now()
    dt = (current_time - prev_time).to_sec()
    prev_time = current_time

    # Calculate linear and angular velocities
    delta_s = (delta_left + delta_right) * 0.5 * (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REVOLUTION
    delta_theta = (delta_right - delta_left) * (2 * math.pi * WHEEL_RADIUS) / (TICKS_PER_REVOLUTION * WHEEL_BASE)

    linear_x = delta_s / dt
    angular_z = delta_theta / dt

    # Integrate linear velocity to update position
    x += linear_x * math.cos(theta) * dt
    y += linear_x * math.sin(theta) * dt

    # Integrate angular velocity to update orientation
    theta += angular_z * dt

    # Convert theta to the range -pi to pi
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi

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

    # Set linear and angular velocities
    odom.twist.twist = Twist(linear=linear_x, angular=angular_z)

    # Publish the Odometry message
    odom_pub.publish(odom)

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
    theta = yaw

# Subscribe to encoder and IMU messages
rospy.Subscriber('erp42_status', erpStatusMsg, encoder_callback)
rospy.Subscriber('imu_data', Imu, imu_callback)

rospy.spin()
