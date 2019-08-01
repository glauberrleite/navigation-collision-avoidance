#!/usr/bin/env python2

import rospy
import sys
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from RVO import RVO_update, reach, compute_V_des, reach

scenario = dict()
scenario['robot_radius'] = 0.4
scenario['circular_obstacles'] = []
scenario['boundary'] = []

Ts = 0.01
X = [[-5,0], [5,0], [0, 5], [0, -5]]
V = [[0,0] for _ in xrange(len(X))]
V_max = [1.0, 1.0, 1.0, 1.0]
goal = [[5, 0], [-5,0], [0, -5], [0, 5]]

def callback_0(msg):
    X[0] = [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)]
def callback_1(msg):
    X[1] = [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)]
def callback_2(msg):
    X[2] = [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)]
def callback_3(msg):
    X[3] = [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)]


rospy.init_node('omni_controller')

sub_0 = rospy.Subscriber('/robot_0/odom', Odometry, callback_0)
sub_1 = rospy.Subscriber('/robot_1/odom', Odometry, callback_1)
sub_2 = rospy.Subscriber('/robot_2/odom', Odometry, callback_2)
sub_3 = rospy.Subscriber('/robot_3/odom', Odometry, callback_3)
pub_0 = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
pub_1 = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
pub_2 = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)
pub_3 = rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=10)
t = 0
while not rospy.is_shutdown():
    
    V_des = compute_V_des(X, goal, V_max)
    V = RVO_update(X, V_des, V, scenario)

    vel_0 = Twist()
    vel_0.linear.x = V[0][0]
    vel_0.linear.y = V[0][1]

    vel_1 = Twist()
    vel_1.linear.x = V[1][0]
    vel_1.linear.y = V[1][1]
    
    vel_2 = Twist()
    vel_2.linear.x = V[2][0]
    vel_2.linear.y = V[2][1]

    vel_3 = Twist()
    vel_3.linear.x = V[3][0]
    vel_3.linear.y = V[3][1]


    pub_0.publish(vel_0)
    pub_1.publish(vel_1)
    pub_2.publish(vel_2)
    pub_3.publish(vel_3)


    if t%100:
        print(X)
    t += 1
    rospy.sleep(Ts)
