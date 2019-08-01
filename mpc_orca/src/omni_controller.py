#!/usr/bin/env python2

import rospy
import sys
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp

N_AGENTS = 4
RADIUS = 0.7
tau = 5

Ts = 0.1
X = [(-5, 0.25), (5, 0), (0.00, 5), (-0.25, -5)]
V = [(0, 0) for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [(5.0, 0.0), (-5.0, 0.0), (0.0, -5.0), (0.0, 5.0)]

agents = []

for i in xrange(len(X)):
        vel = (numpy.array(goal[i]) - numpy.array(X[i]))
        if numpy.linalg.norm(vel) > V_max[i]:
            vel = normalized(vel) * V_max[i]
        agents.append(Agent(X[i], (0., 0.), RADIUS, V_max[i], vel))

def update_agents(agents):
    for i in xrange(len(X)):        
        agents[i].pref_velocity = (numpy.array(goal[i]) - numpy.array(X[i]))
        if numpy.linalg.norm(agents[i].pref_velocity) > V_max[i]:
            agents[i].pref_velocity = normalized(agents[i].pref_velocity) * V_max[i]
    return agents

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
pub_0 = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
pub_1 = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
pub_2 = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)
pub_3 = rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=10)
t = 0

while not rospy.is_shutdown():
    agents = update_agents(agents)
    for i, agent in enumerate(agents):
        candidates = agents[:i] + agents[i + 1:]
        agent.velocity, _ = orca(agent, candidates, tau, Ts)

    vel_0 = Twist()
    vel_0.linear.x = agents[0].velocity[0]
    vel_0.linear.y = agents[0].velocity[1]

    vel_1 = Twist()
    vel_1.linear.x = agents[1].velocity[0]
    vel_1.linear.y = agents[1].velocity[1]
    
    vel_2 = Twist()
    vel_2.linear.x = agents[2].velocity[0]
    vel_2.linear.y = agents[2].velocity[1]
    
    vel_3 = Twist()
    vel_3.linear.x = agents[3].velocity[0]
    vel_3.linear.y = agents[3].velocity[1]

    pub_0.publish(vel_0)
    pub_1.publish(vel_1)
    pub_2.publish(vel_2)
    pub_3.publish(vel_3)

    if t%100:
        print(X)
        #print(agents[0].velocity)
    t += 1
    rospy.sleep(Ts)
