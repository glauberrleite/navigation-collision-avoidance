#!/usr/bin/env python2

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from MPC_ORCA import MPC_ORCA
from pyorca import Agent

RADIUS = 0.5
tau = 15

N = 10
Ts = 0.1
X = [[-10., 0.], [10., 0.], [0.0, 10.], [0., -10.], [-10., 10.], [-10., -10.], [10.0, -10.], [10., 10.]]
V = [[0., 0.] for _ in xrange(len(X))]
V_min = [-1.0 for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [[10., 0.], [-10., 0.], [0.0, -10.], [0., 10.], [10., -10.], [10., 10.], [-10.0, 10.], [-10., -10.]]

agents = []

for i in xrange(len(X)):
    agents.append(Agent(X[i], [0., 0.], RADIUS))

def update_positions(agents):
    for i in xrange(len(X)):
        agents[i].position = np.array(X[i])
    return agents

def callback_0(msg):
    X[0] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
def callback_1(msg):
    X[1] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
def callback_2(msg):
    X[2] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
def callback_3(msg):
    X[3] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
def callback_4(msg):
    X[4] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
def callback_5(msg):
    X[5] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
def callback_6(msg):
    X[6] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
def callback_7(msg):
    X[7] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])


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
pub_setpoint_pos = rospy.Publisher('/setpoint_pos', Vector3, queue_size=10)
pub_setpoint_vel = rospy.Publisher('/setpoint_vel', Vector3, queue_size=10)

setpoint_pos = Vector3()
setpoint_vel = Vector3()

# Initializing Controllers
controller = []
for i, agent in enumerate(agents):
    colliders = agents[:i] + agents[i + 1:]
    controller.append(MPC_ORCA(agent.position, V_min[i], V_max[i], N, Ts, colliders, tau, agent.radius))

rospy.wait_for_message('/clock', Clock)

while not rospy.is_shutdown():
    
    agents = update_positions(agents)

    for i, agent in enumerate(agents):
        # Computing desired velocity
        V_des = goal[i] - X[i]
        P_des = X[i] + V_des * Ts

        controller[i].agent = agents[i]
        controller[i].colliders = agents[:i] + agents[i + 1:]

        agents[i].velocity = controller[i].getNewVelocity(P_des, V_des)

        if i == 2:
            setpoint_pos.x = P_des[0]
            setpoint_pos.y = P_des[1]

            setpoint_vel.x = V_des[0]
            setpoint_vel.y = V_des[1]

    for i in xrange(len(X)):
        vel = Twist()
        vel.linear.x = agents[i].velocity[0]
        vel.linear.y = agents[i].velocity[1]

        pub[i].publish(vel)

    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)

    rospy.sleep(Ts)