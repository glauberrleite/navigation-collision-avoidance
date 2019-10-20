#!/usr/bin/env python2

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from MPC_ORCA import MPC_ORCA
from pyorca import Agent

RADIUS = 0.5
tau = 5

N = 10
Ts = 0.1
X = [[-5., 0.], [5., 0.], [0.0, 5.], [0., -5.]]
orientation = [0, np.pi, -np.pi/2, np.pi/2]
V = [[0., 0.] for _ in xrange(len(X))]
V_min = [-1.0 for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [[5.0, 0.0], [-5.0, 0.0], [0.0, -5.0], [0.0, 5.0]]

agents = []

for i in xrange(len(X)):
    agents.append(Agent(X[i], [0., 0.], RADIUS))

def velocityTransform(v, theta_0):
    angular = np.arctan2(v[1], v[0]) - theta_0 
    linear = np.sqrt(v[0]**2 + v[1]**2)

    # Handling singularity
    if np.abs(angular) > np.pi:
        angular -= np.sign(angular) * 2 * np.pi

    return [linear, angular]

def update_positions(agents):
    for i in xrange(len(X)):
        agents[i].position = np.array(X[i])
    return agents

def callback_0(msg):
    X[0] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[0] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[0] > np.pi):
        # For gazebo odom quaternion
        orientation[0] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_1(msg):
    X[1] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[1] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[1] > np.pi):
        # For gazebo odom quaternion
        orientation[1] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_2(msg):
    X[2] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[2] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[2] > np.pi):
        # For gazebo odom quaternion
        orientation[2] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_3(msg):
    X[3] = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
    orientation[3] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[3] > np.pi):
        # For gazebo odom quaternion
        orientation[3] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))


rospy.init_node('omni_controller')
#rospy.on_shutdown(plot_result)

sub_0 = rospy.Subscriber('/robot_0/odom', Odometry, callback_0)
sub_1 = rospy.Subscriber('/robot_1/odom', Odometry, callback_1)
sub_2 = rospy.Subscriber('/robot_2/odom', Odometry, callback_2)
sub_3 = rospy.Subscriber('/robot_3/odom', Odometry, callback_3)
pub_0 = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
pub_1 = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
pub_2 = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)
pub_3 = rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=10)
t = 0

# Initializing Controllers
controller = []
for i, agent in enumerate(agents):
    colliders = agents[:i] + agents[i + 1:]
    controller.append(MPC_ORCA(goal[i], agent.position, V_min[i], V_max[i], N, Ts, colliders, tau, agent.radius))

while not rospy.is_shutdown():
    
    agents = update_positions(agents)

    for i, agent in enumerate(agents):
        controller[i].agent = agents[i]
        controller[i].colliders = agents[:i] + agents[i + 1:]
        agents[i].velocity = controller[i].getNewVelocity()

    vel_0 = Twist()
    
    [linear_vel_0, angular_vel_0] = velocityTransform(agents[0].velocity, orientation[0])
    
    vel_0.linear.x = linear_vel_0
    vel_0.angular.z = angular_vel_0

    vel_1 = Twist()
    
    [linear_vel_1, angular_vel_1] = velocityTransform(agents[1].velocity, orientation[1])
    
    vel_1.linear.x = linear_vel_1
    vel_1.angular.z = angular_vel_1
    
    vel_2 = Twist()
    
    [linear_vel_2, angular_vel_2] = velocityTransform(agents[2].velocity, orientation[2])
    
    vel_2.linear.x = linear_vel_2
    vel_2.angular.z = angular_vel_2
    
    vel_3 = Twist()
    
    [linear_vel_3, angular_vel_3] = velocityTransform(agents[3].velocity, orientation[3])

    vel_3.linear.x = linear_vel_3
    vel_3.angular.z = angular_vel_3

    pub_0.publish(vel_0)
    pub_1.publish(vel_1)
    pub_2.publish(vel_2)
    pub_3.publish(vel_3)

    #if t%100:
    #    print(X)
    #t += 1
    rospy.sleep(Ts)
