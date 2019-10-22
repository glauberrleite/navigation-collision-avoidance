#!/usr/bin/env python2

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, Vector3
from RVO import RVO_update, compute_V_des

scenario = dict()
scenario['robot_radius'] = 0.5
scenario['circular_obstacles'] = []
scenario['boundary'] = []

Ts = 0.01
X = [[-10., 0.], [10., 0.], [0.0, 10.], [0., -10.], [-10., 10.], [-10., -10.], [10.0, -10.], [10., 10.]]
orientation = [0, np.pi, -np.pi/2, np.pi/2, -np.pi/4, np.pi/4, 3*np.pi/4, -3*np.pi/4]
V = [[0., 0.] for _ in xrange(len(X))]
V_min = [-1.0 for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [[10., 0.], [-10., 0.], [0.0, -10.], [0., 10.], [10., -10.], [10., 10.], [-10.0, 10.], [-10., -10.]]

def velocityTransform(v, theta_0):
    angular = np.arctan2(v[1], v[0]) - theta_0 
    linear = np.sqrt(v[0]**2 + v[1]**2)

    # Handling singularity
    if np.abs(angular) > np.pi:
        angular -= np.sign(angular) * 2 * np.pi

    return [linear, angular]

def callback_0(msg):
    X[0] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[0] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
def callback_1(msg):
    X[1] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[1] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
def callback_2(msg):
    X[2] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[2] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
def callback_3(msg):
    X[3] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[3] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
def callback_4(msg):
    X[4] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[4] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
def callback_5(msg):
    X[5] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[5] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
def callback_6(msg):
    X[6] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[6] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
def callback_7(msg):
    X[7] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[7] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))


rospy.init_node('diff_controller')

sub_0 = rospy.Subscriber('/robot_0/odom', Odometry, callback_0)
sub_1 = rospy.Subscriber('/robot_1/odom', Odometry, callback_1)
sub_2 = rospy.Subscriber('/robot_2/odom', Odometry, callback_2)
sub_3 = rospy.Subscriber('/robot_3/odom', Odometry, callback_3)
sub_4 = rospy.Subscriber('/robot_4/odom', Odometry, callback_4)
sub_5 = rospy.Subscriber('/robot_5/odom', Odometry, callback_5)
sub_6 = rospy.Subscriber('/robot_6/odom', Odometry, callback_6)
sub_7 = rospy.Subscriber('/robot_7/odom', Odometry, callback_7)
pub = []
pub.append(rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_4/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_5/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_6/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_7/cmd_vel', Twist, queue_size=10))
pub_setpoint = rospy.Publisher('/setpoint', Vector3, queue_size=10)

setpoint = Vector3()
setpoint.x = goal[4][0]
setpoint.y = goal[4][1]

rospy.wait_for_message('/clock', Clock)
while not rospy.is_shutdown():
    
    V_des = compute_V_des(X, goal, V_max)
    V = RVO_update(X, V_des, V, scenario)


    for i in xrange(len(X)):
        vel = Twist()
        [vel.linear.x, vel.angular.z] = velocityTransform(V[i], orientation[i])

        pub[i].publish(vel)

    pub_setpoint.publish(setpoint)
    rospy.sleep(Ts)
