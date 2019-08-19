#!/usr/bin/env python2

import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from MPC_ORCA import MPC_ORCA

N_AGENTS = 0
RADIUS = 0.7
tau = 5

Ts = 0.1
X = [(-5, 0)]
V = [(0, 0) for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [(5.0, 5.0)]


def callback_0(msg):
    X[0] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    V[0] = (float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y))

rospy.init_node('test_controller')

sub_0 = rospy.Subscriber('/odom', Odometry, callback_0)
pub_0 = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
t = 0

controller = MPC_ORCA(goal[0], X[0], -V_max[0], V_max[0], 10, Ts)

while not rospy.is_shutdown():
    
    a_new = controller.getNewAcceleration(X[0], V[0])
    velocity = V[0] + a_new * Ts

    vel_0 = Twist()
    vel_0.linear.x = velocity[0]
    vel_0.linear.y = velocity[1]

    
    pub_0.publish(vel_0)
    
    if t%100:
        print(velocity)
        print(X[0])
    t += 1
    rospy.sleep(Ts)
