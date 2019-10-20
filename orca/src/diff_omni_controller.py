#!/usr/bin/env python2

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from pyorca import Agent, orca, normalized

N_AGENTS = 4
RADIUS = 0.7
tau = 5

Ts = 0.01
X = [(-5, 0.25), (5, 0), (0.00, 5), (-0.25, -5)]
orientation = [0, np.pi, -np.pi/2, np.pi/2]
V = [(0, 0) for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [(5.0, 0.0), (-5.0, 0.0), (0.0, -5.0), (0.0, 5.0)]

agents = []

for i in xrange(len(X)):
        vel = (np.array(goal[i]) - np.array(X[i]))
        if np.linalg.norm(vel) > V_max[i]:
            vel = normalized(vel) * V_max[i]
        agents.append(Agent(X[i], (0., 0.), RADIUS, V_max[i], vel))

def update_agents(agents):
    for i in xrange(len(X)):        
        agents[i].pref_velocity = (np.array(goal[i]) - np.array(X[i]))
        if np.linalg.norm(agents[i].pref_velocity) > V_max[i]:
            agents[i].pref_velocity = normalized(agents[i].pref_velocity) * V_max[i]
    return agents

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
    
    angular_vel_0 = (np.arctan2(agents[0].velocity[1], agents[0].velocity[0]) - orientation[0])
    if np.abs(angular_vel_0) > np.pi:
        angular_vel_0 = (orientation[0] - np.arctan2(agents[0].velocity[1], agents[0].velocity[0]))
    linear_vel_0 = np.sqrt(agents[0].velocity[0]**2 + agents[0].velocity[1]**2)
    
    vel_0.linear.x = linear_vel_0
    vel_0.angular.z = angular_vel_0

    vel_1 = Twist()
    
    angular_vel_1 = (np.arctan2(agents[1].velocity[1], agents[1].velocity[0]) - orientation[1])
    if np.abs(angular_vel_1) > np.pi:
        angular_vel_1 = (orientation[1] - np.arctan2(agents[1].velocity[1], agents[1].velocity[0]))
    linear_vel_1 = np.sqrt(agents[1].velocity[0]**2 + agents[1].velocity[1]**2)
    
    vel_1.linear.x = linear_vel_1
    vel_1.angular.z = angular_vel_1
    
    vel_2 = Twist()
    
    angular_vel_2 = (np.arctan2(agents[2].velocity[1], agents[2].velocity[0]) - orientation[2]) 
    if np.abs(angular_vel_2) > np.pi:
        angular_vel_2 = (orientation[2] - np.arctan2(agents[2].velocity[1], agents[2].velocity[0]))
    linear_vel_2 = np.sqrt(agents[2].velocity[0]**2 + agents[2].velocity[1]**2)
    
    vel_2.linear.x = linear_vel_2
    vel_2.angular.z = angular_vel_2
    
    vel_3 = Twist()
    
    angular_vel_3 = (np.arctan2(agents[3].velocity[1], agents[3].velocity[0]) - orientation[3]) 
    if np.abs(angular_vel_3) > np.pi:
        angular_vel_3 = (orientation[3] - np.arctan2(agents[3].velocity[1], agents[3].velocity[0]))
    linear_vel_3 = np.sqrt(agents[3].velocity[0]**2 + agents[3].velocity[1]**2)
    
    vel_3.linear.x = linear_vel_3
    vel_3.angular.z = angular_vel_3
    
    pub_0.publish(vel_0)
    pub_1.publish(vel_1)
    pub_2.publish(vel_2)
    pub_3.publish(vel_3)
    
    if t%100:
        print(X)
        #print(agents[0].velocity)
    t += 1
    rospy.sleep(Ts)
