#!/usr/bin/env python2

import rospy
import sys
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from RVO import RVO_update, compute_V_des
from rosgraph_msgs.msg import Clock

scenario = dict()
scenario['robot_radius'] = 0.5
scenario['circular_obstacles'] = []
scenario['boundary'] = []

Ts = 0.01
X = [[-10., 0.], [10., 0.], [0.0, 10.], [0., -10.], [-10., 10.], [-10., -10.], [10.0, -10.], [10., 10.]]
V = [[0., 0.] for _ in xrange(len(X))]
V_min = [-1.0 for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [[10., 0.], [-10., 0.], [0.0, -10.], [0., 10.], [10., -10.], [10., 10.], [-10.0, 10.], [-10., -10.]]

def callback_0(msg):
    X[0] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_1(msg):
    X[1] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_2(msg):
    X[2] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_3(msg):
    X[3] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_4(msg):
    X[4] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_5(msg):
    X[5] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_6(msg):
    X[6] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_7(msg):
    X[7] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))


rospy.init_node('omni_controller')

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
        vel.linear.x = V[i][0]
        vel.linear.y = V[i][1]

        pub[i].publish(vel)

    pub_setpoint.publish(setpoint)
    rospy.sleep(Ts)
