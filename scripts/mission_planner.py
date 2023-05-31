#!/usr/bin/env python

import numpy as np
from math import pi as PI
import rospy

from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandTOL

from tf.transformations import *
import tf

# Velocity 
traj_profile = np.array([ [ 6,  0.01,  0.01,  0.01, 0.0],
                          [ 3,  0.2,  0.0,  0.0,  0.0],
                          [ 3,  0.0,  0.0,  0.0,  PI/4],
                          [ 3,  0.2,  0.0,  0.0,  0.0],
                          [ 3,  0.0,  0.0,  0.0,  PI/4],
                          [ 3,  0.2,  0.0,  0.0,  0.0],
                          [ 3,  0.0,  0.0,  0.0,  PI/4],
                          [ 3,  0.2,  0.0,  0.0,  0.0],
                          [ 2,  0.0,  0.0, -0.5,  0.0]])

# traj_profile = np.array([ [ 6,  0.01,  0.01,  0.01, 0.0],
#                           [ 3,  0.2,  0.0,  0.0],
#                           [ 3,  0.0,  0.2,  0.0],
#                           [ 3, -0.2,  0.0,  0.0],
#                           [ 3,  0.0, -0.2,  0.0],
#                           [ 2,  0.0,  0.0, -0.5]])


battery_percentage = 0
odom = Odometry()

pos_sp_pub = None

def battery_callback(msg):
    global battery_percentage
    battery_percentage = msg.percentage

def pose_callback(msg):
    odom.pose.pose = msg.pose

def velocity_callback(msg):
    odom.twist.twist = msg.twist

# Init the node
rospy.init_node("mission_planner", anonymous=True)

# Susbcribe to pose and velocity topic
batt_sub     = rospy.Subscriber("/mavros/battery", BatteryState, battery_callback)
pose_sub     = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
velocity_sub = rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, velocity_callback)

# Advertise the position setpoint
pos_setpoint = PositionTarget()
pos_setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

# Service to request takeoff
takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
# Command drone to takeoff
takeoff_srv(min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=1.0)

current_setpoint = 0
last_setpoint_time = rospy.Time.now()
start_time = rospy.Time.now()
rate = rospy.Rate(10)
while not rospy.is_shutdown():

    # Extract the states in vector forms
    pos = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
    eul = euler_from_quaternion([odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z])
    vel = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z])

    # # Update the setpoint by time
    # if (rospy.Time.now() - last_setpoint_time).to_sec() > traj_profile[current_setpoint, 0]:
        
    #     last_setpoint_time = rospy.Time.now()
    #     current_setpoint += 1

    #     if current_setpoint == traj_profile.shape[0]:
    #         current_setpoint = traj_profile.shape[0]-1
    #         land_srv(min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=0.0)
    #     else:
    #         # Publish the setpoint position
    #         pos_setpoint.header.stamp = rospy.Time.now()
    #         pos_setpoint.header.frame_id = "map"
    #         pos_setpoint.coordinate_frame = 1
    #         pos_setpoint.velocity.x = traj_profile[current_setpoint, 1]
    #         pos_setpoint.velocity.y = traj_profile[current_setpoint, 2]
    #         pos_setpoint.velocity.z = traj_profile[current_setpoint, 3]
    #         pos_setpoint.yaw_rate   = traj_profile[current_setpoint, 4]

    # Compose a text report
    info_text = f'Time: {(rospy.Time.now() - start_time).to_sec():6.3f}. ' + \
                f'Batt: {battery_percentage}. ' + \
                f'Pos: {pos[0]:4.1f}, {pos[1]:4.1f}, {pos[2]:4.1f}. ' + \
                f'Eul: {eul[0]:4.1f}, {eul[1]:4.1f}, {eul[2]:4.1f}. ' + \
                f'Vel: {vel[0]:4.1f}, {vel[1]:4.1f}, {vel[2]:4.1f}. ' + \
                f'VSP: {traj_profile[current_setpoint, 1]:4.1f}, {traj_profile[current_setpoint, 2]:4.1f}, {traj_profile[current_setpoint, 3]:4.1f} , {traj_profile[current_setpoint, 4]:4.1f}'
    
    # Print the report to the terminal
    print(info_text)

    # Publish the setpoint
    pos_setpoint_pub.publish(pos_setpoint)

    rate.sleep()
    # Print the report to the terminal
    print(info_text)

    # Publish the setpoint
    pos_setpoint_pub.publish(pos_setpoint)

    rate.sleep()