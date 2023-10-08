#!/usr/bin/env python3

import rospy
from erp_driver.msg import erpStatusMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import math
import tf2_ros
import geometry_msgs.msg

# Create odometry data publishers
odom_data_pub = None
odom_data_pub_quat = None
odomNew = Odometry()
odomOld = Odometry()

# Create a TF broadcaster for IMU
tf_broadcaster_imu = None

# Initial pose
initialX = 0.0
initialY = 0.0
initialTheta = 0.00000000001
PI = 3.141592

# Robot physical constants
TICKS_PER_REVOLUTION = 100
WHEEL_RADIUS = 0.27
WHEEL_BASE = 0.97
TICKS_PER_METER = 47.75

# Distance both wheels have traveled
distanceLeft = 0

# Flag to see if initial pose has been received
initialPoseReceived = False

# Get initial_2d message from either Rviz clicks or a manual pose publisher
def set_initial_2d(rvizClick):
    global odomOld, initialPoseReceived

    odomOld.pose.pose.position.x = rvizClick.pose.position.x
    odomOld.pose.pose.position.y = rvizClick.pose.position.y
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z
    initialPoseReceived = True

# Calculate the distance the left wheel has traveled since the last cycle
def calc_left(erpStatus):
    global distanceLeft

    leftTicks = erpStatus.encoder

    if leftTicks != 0:
        distanceLeft = leftTicks / TICKS_PER_METER

# Publish a nav_msgs/Odometry message in quaternion format
def publish_quat():
    global odomNew, odom_data_pub_quat

    q = quaternion_from_euler(0, 0, odomNew.pose.pose.orientation.z)

    quatOdom = Odometry()
    quatOdom.header.stamp = odomNew.header.stamp
    quatOdom.header.frame_id = "odom"
    quatOdom.child_frame_id = "base_link"
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z
    quatOdom.pose.pose.orientation.x = q[0]
    quatOdom.pose.pose.orientation.y = q[1]
    quatOdom.pose.pose.orientation.z = q[2]
    quatOdom.pose.pose.orientation.w = q[3]
    quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x
    quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y
    quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z
    quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x
    quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y
    quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z

    quatOdom.pose.covariance = [0.01 if i in [0, 7, 14] else 0.1 if i in [21, 28, 35] else 0 for i in range(36)]

    odom_data_pub_quat.publish(quatOdom)

# Publish the odom frame
def publish_odom_frame():
    global odomNew

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Create a TransformStamped message
    transform_stamped = geometry_msgs.msg.TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "odom"
    transform_stamped.child_frame_id = "base_link"
    transform_stamped.transform.translation.x = odomNew.pose.pose.position.x
    transform_stamped.transform.translation.y = odomNew.pose.pose.position.y
    transform_stamped.transform.translation.z = odomNew.pose.pose.position.z
    transform_stamped.transform.rotation = odomNew.pose.pose.orientation

    # Publish the transform
    tf_broadcaster.sendTransform(transform_stamped)

def imu_callback(msg):
    # Extract orientation information from IMU message
    quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    # Create a TF broadcaster for imu_link
    tf_broadcaster_imu = tf2_ros.TransformBroadcaster()

    # Create a TransformStamped message for imu_link
    transform_stamped = geometry_msgs.msg.TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "base_link"  # Your base_link frame
    transform_stamped.child_frame_id = "imu_link"    # Your imu_link frame
    transform_stamped.transform.translation.x = 0.0   # No translation along x-axis
    transform_stamped.transform.translation.y = 0.0   # No translation along y-axis
    transform_stamped.transform.translation.z = 0.0   # No translation along z-axis
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    # Publish the transform for imu_link
    tf_broadcaster_imu.sendTransform(transform_stamped)

    # Create and publish a new IMU message for imu/data_link
    imu_msg_link = Imu()
    imu_msg_link.header.stamp = rospy.Time.now()
    imu_msg_link.header.frame_id = "imu_link"  # Use the imu_link frame for IMU data
    imu_msg_link.orientation = msg.orientation  # Copy orientation from the original IMU message
    # Fill in other fields of imu_msg_link as needed

    # Publish the IMU data with the appropriate frame_id
    imu_data_pub.publish(imu_msg_link)

def update_odom():
    global distanceLeft, odomNew, odomOld

    cycleDistance = distanceLeft
    cycleAngle = 0
    avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z

    if avgAngle > PI:
        avgAngle -= 2 * PI
    elif avgAngle < -PI:
        avgAngle += 2 * PI

    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
    odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z

    if math.isnan(odomNew.pose.pose.position.x) or math.isnan(odomNew.pose.pose.position.y) or math.isnan(
            odomNew.pose.pose.position.z):
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z

    if odomNew.pose.pose.orientation.z > PI:
        odomNew.pose.pose.orientation.z -= 2 * PI
    elif odomNew.pose.pose.orientation.z < -PI:
        odomNew.pose.pose.orientation.z += 2 * PI

    cycleTime = odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec()
    odomNew.header.stamp = rospy.Time.now()
    odomNew.twist.twist.linear.x = cycleDistance / cycleTime
    odomNew.twist.twist.angular.z = cycleAngle / cycleTime

    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z
    odomOld.header.stamp = odomNew.header.stamp

    odom_data_pub.publish(odomNew)

def main():
    global odomNew, odomOld, odom_data_pub, odom_data_pub_quat, tf_broadcaster_imu

    rospy.init_node("ekf_odom_imu_pub")

    rospy.Subscriber("/erp42_status", erpStatusMsg, calc_left)
    rospy.Subscriber("initial_2d", PoseStamped, set_initial_2d)

    odom_data_pub = rospy.Publisher("odom_data_euler", Odometry, queue_size=100)
    odom_data_pub_quat = rospy.Publisher("odom_data_quat", Odometry, queue_size=100)
    imu_data_pub = rospy.Publisher("imu/data_link", Imu, queue_size=10)
    
    # Initialize TF broadcaster for IMU
    tf_broadcaster_imu = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if initialPoseReceived:
            update_odom()
            publish_quat()
            publish_odom_frame()  # Publish the odom frame
        rate.sleep()

if __name__ == "__main__":
    main()
