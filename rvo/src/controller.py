#!/usr/bin/env python2

# @author glauberrleite

import rospy

import numpy as np

from RVO import RVO_update, reach, compute_V_des, reach
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from kobuki_msgs.msg import MotorPower

scenario = dict()
scenario['robot_radius'] = 0.8
scenario['circular_obstacles'] = []
scenario['boundary'] = []

Ts = 0.01
X = [[0,0], [10,0]]
orientation = [0, np.pi]
V = [[0,0] for _ in xrange(len(X))]
V_max = [1.0 for i in xrange(len(X))]
goal = [[10, 0], [0,0]]

def updateWorld(msg):
    X[1] = [float(msg.pose[1].position.x), float(msg.pose[1].position.y)]
    orientation[1] = 2* np.arctan2(float(msg.pose[1].orientation.z), float(msg.pose[1].orientation.w))
    if np.abs(orientation[1]) > np.pi:
        orientation[1] = orientation[1] - np.sign(orientation[1]) * 2 * np.pi

    X[0] = [float(msg.pose[2].position.x), float(msg.pose[2].position.y)]
    orientation[0] = 2* np.arctan2(float(msg.pose[2].orientation.z), float(msg.pose[2].orientation.w)) 
    if np.abs(orientation[0]) > np.pi:
        orientation[0] = orientation[0] - np.sign(orientation[0]) * 2 * np.pi


rospy.init_node('rvo_controller')
pub_vel_r0 = rospy.Publisher("robot0/mobile_base/commands/velocity", Twist, queue_size=10);
pub_vel_r1 = rospy.Publisher("robot1/mobile_base/commands/velocity", Twist, queue_size=10);
pub_mot_r0 = rospy.Publisher("robot0/mobile_base/commands/motor_power", MotorPower, queue_size=10);
pub_mot_r1 = rospy.Publisher("robot1/mobile_base/commands/motor_power", MotorPower, queue_size=10);

rospy.Subscriber('/gazebo/model_states', ModelStates, updateWorld)

motor0 = MotorPower()
motor1 = MotorPower()
motor0.state = motor0.ON
motor1.state = motor1.ON

pub_mot_r0.publish(motor0)
pub_mot_r1.publish(motor1)

t = 0

while not rospy.is_shutdown():
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, scenario)

    # Compute ang_vel considering singularities
    angular_vel_0 = (np.arctan2(V[0][1], V[0][0]) - orientation[0]) 
    if np.abs(angular_vel_0) > np.pi:
        angular_vel_0 = (orientation[0] - np.arctan2(V[0][1], V[0][0]))

    linear_vel_0 = np.sqrt(V[0][0]**2 + V[0][1]**2)
    vel_rob0 = Twist()
    vel_rob0.linear.x = linear_vel_0
    vel_rob0.linear.y = 0.0
    vel_rob0.linear.z = 0.0
    vel_rob0.angular.x = 0.0
    vel_rob0.angular.y = 0.0
    vel_rob0.angular.z = angular_vel_0

    angular_vel_1 = (np.arctan2(V[1][1], V[1][0]) - orientation[1])
    if np.abs(angular_vel_1) > np.pi:
        angular_vel_1 = (orientation[0] - np.arctan2(V[1][1], V[1][0]))

    linear_vel_1 = np.sqrt(V[1][0]**2 + V[1][1]**2)
    vel_rob1 = Twist()
    vel_rob1.linear.x = linear_vel_1
    vel_rob1.linear.y = 0.0;
    vel_rob1.linear.z = 0.0;
    vel_rob1.angular.x = 0.0;
    vel_rob1.angular.y = 0.0;
    vel_rob1.angular.z = angular_vel_1

    pub_vel_r0.publish(vel_rob0)
    pub_vel_r1.publish(vel_rob1)

    #if t%100 == 0:
    #    print(V_des[1])
    #    print(vel_rob1.linear.x)
    #    print(vel_rob1.angular.z)
    #    print(orientation)
    #    print('---------')
    #t = t + 1

    rospy.sleep(Ts)
