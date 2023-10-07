#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from math import sin, cos, asin
from erp_driver.msg import erpStatusMsg
# Initialize ROS node
rospy.init_node('ekf_odom_pub')

# Create odometry data publishers
odom_data_pub = rospy.Publisher('odom123', Odometry, queue_size=100)
odom_data_pub_quat = rospy.Publisher('odom', Odometry, queue_size=100)

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

# Distance left wheel has traveled
distanceLeft = 0

# Flag to check if initial pose has been received
initialPoseReceived = False

# Create odometry message
odomNew = Odometry()
odomOld = Odometry()

# Get initial_2d message from either Rviz clicks or a manual pose publisher
def set_initial_2d(rvizClick):
    global odomOld, initialPoseReceived

    odomOld.pose.pose.position.x = rvizClick.pose.position.x
    odomOld.pose.pose.position.y = rvizClick.pose.position.y
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z
    initialPoseReceived = True

# Calculate the distance the left wheel has traveled since the last cycle
def calc_left(erp_msg):
    global distanceLeft

    static_lastCountL = 0
    if erp_msg.encoder != 0 and static_lastCountL != 0:
        leftTicks = (erp_msg.encoder - static_lastCountL)

        if leftTicks > 10000:
            leftTicks = 0 - (65535 - leftTicks)
        elif leftTicks < -10000:
            leftTicks = 65535 - leftTicks
        else:
            pass

        distanceLeft = leftTicks / TICKS_PER_METER

    static_lastCountL = erp_msg.encoder

# Publish a nav_msgs/Odometry message in quaternion format
def publish_quat():
    global odomNew

    q = tf2_geometry_msgs.Quaternion()
    q.setRPY(0, 0, odomNew.pose.pose.orientation.z)

    quatOdom = Odometry()
    quatOdom.header.stamp = odomNew.header.stamp
    quatOdom.header.frame_id = "odom"
    quatOdom.child_frame_id = "base_link"
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z
    quatOdom.pose.pose.orientation.x = q.x
    quatOdom.pose.pose.orientation.y = q.y
    quatOdom.pose.pose.orientation.z = q.z
    quatOdom.pose.pose.orientation.w = q.w
    quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x
    quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y
    quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z
    quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x
    quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y
    quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z

    for i in range(36):
        if i == 0 or i == 7 or i == 14:
            quatOdom.pose.covariance[i] = 0.01
        elif i == 21 or i == 28 or i == 35:
            quatOdom.pose.covariance[i] += 0.1
        else:
            quatOdom.pose.covariance[i] = 0

    odom_data_pub_quat.publish(quatOdom)

# Update odometry information
def update_odom():
    global odomNew, odomOld, distanceLeft

    # Calculate the average distance
    cycleDistance = distanceLeft

    # Calculate the number of radians the robot has turned since the last cycle
    cycleAngle = asin((distanceLeft - distanceLeft) / WHEEL_BASE)

    # Average angle during the last cycle
    avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z

    if avgAngle > PI:
        avgAngle -= 2 * PI
    elif avgAngle < -PI:
        avgAngle += 2 * PI
    else:
        pass

    # Calculate the new pose (x, y, and theta)
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance
    odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z

    # Prevent lockup from a single bad cycle
    if isnan(odomNew.pose.pose.position.x) or isnan(odomNew.pose.pose.position.y) or isnan(odomNew.pose.pose.position.z):
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z

    # Make sure theta stays in the correct range
    if odomNew.pose.pose.orientation.z > PI:
        odomNew.pose.pose.orientation.z -= 2 * PI
    elif odomNew.pose.pose.orientation.z < -PI:
        odomNew.pose.pose.orientation.z += 2 * PI
    else:
        pass

    # Compute the velocity
    odomNew.header.stamp = rospy.Time.now()
    odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())
    odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())

    # Save the pose data for the next cycle
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z
    odomOld.header.stamp = odomNew.header.stamp

    # Publish the odometry message
    odom_data_pub.publish(odomNew)

# Subscribe to ROS topics
rospy.Subscriber('left_ticks', erpStatusMsg, calc_left, queue_size=100)
rospy.Subscriber('initial_2d', PoseStamped, set_initial_2d, queue_size=1)

# Set the loop rate
loop_rate = rospy.Rate(30)

while not rospy.is_shutdown():
    if initialPoseReceived:
        update_odom()
        publish_quat()
    loop_rate.sleep()
