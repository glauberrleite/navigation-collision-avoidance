#!/usr/bin/env python2

import rospy
import sys
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from RVO import RVO_update, compute_V_des

scenario = dict()
scenario['robot_radius'] = 0.4
scenario['circular_obstacles'] = []
scenario['boundary'] = []

Ts = 0.01
X = [[-5., 0.], [5., 0.], [0.0, 5.], [0., -5.]]
V = [[0., 0.] for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [[5., 0.], [-5., 0.], [0.0, -5.], [0., 5.]]

def callback_0(msg):
    X[0] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_1(msg):
    X[1] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_2(msg):
    X[2] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
def callback_3(msg):
    X[3] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

rospy.init_node('omni_controller')

sub_0 = rospy.Subscriber('/robot_0/odom', Odometry, callback_0)
sub_1 = rospy.Subscriber('/robot_1/odom', Odometry, callback_1)
sub_2 = rospy.Subscriber('/robot_2/odom', Odometry, callback_2)
sub_3 = rospy.Subscriber('/robot_3/odom', Odometry, callback_3)
pub = []
pub.append(rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=10))
t = 0
while not rospy.is_shutdown():
    
    V_des = compute_V_des(X, goal, V_max)
    V = RVO_update(X, V_des, V, scenario)

    for i in xrange(len(X)):
        vel = Twist()
        vel.linear.x = V[i][0]
        vel.linear.y = V[i][1]

        pub[i].publish(vel)


    if t%100:
        print(X)
        print('--------------')
    t += 1
    rospy.sleep(Ts)
