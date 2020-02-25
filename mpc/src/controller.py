#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.msg import ModelStates
from MPC import MPC

N = 10
N_c = 5
Ts = 0.1
X = np.array([0., 0.])
orientation = 0
V = np.array([0., 0.])
V_min = -1
V_max = 1

goal = np.array([float(sys.argv[1]), float(sys.argv[2])])

def accelerationTransform(a, v, w, theta_0):
    """This function applies the linearization transformation on acceleration values 
    based on equation 2.11
    """
    d = 0.2
    cos_theta = np.cos(theta_0)
    sin_theta = np.sin(theta_0)
    inverse = np.linalg.inv(np.array([[cos_theta, -d * sin_theta],[sin_theta, d * cos_theta]]))
    term1 = a[0] + v * w * sin_theta + d * (w**2) * cos_theta
    term2 = a[1] - v * w * cos_theta + d * (w**2) * sin_theta
    acc = np.matmul(inverse, np.vstack([term1, term2]))
    acc = acc.T

    return acc[0]

def updateWorld(msg):
    """This funcion is called whenever the gazebo/model_states publishes. This function
    updates the world variables as fast as possible"""
    global X, V, orientation
    X = np.array([float(msg.pose[1].position.x), float(msg.pose[1].position.y)])
    V = np.array([float(msg.twist[1].linear.x), float(msg.twist[1].linear.y)])
    orientation = np.arctan2(2 * float(msg.pose[1].orientation.w) * float(msg.pose[1].orientation.z), \
        1 - 2 * float(msg.pose[1].orientation.z)**2)

rospy.init_node('mpc_controller')

# Subscribing on model_states instead of robot/odom, to avoid unnecessary noise
rospy.Subscriber('/gazebo/model_states', ModelStates, updateWorld)

# Velocity publishers
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Setpoint Publishers
pub_setpoint_pos = rospy.Publisher('/setpoint_pos', Vector3, queue_size=10)
pub_setpoint_vel = rospy.Publisher('/setpoint_vel', Vector3, queue_size=10)

setpoint_pos = Vector3()
setpoint_vel = Vector3()

# Initializing Controllers
controller = MPC(X, V_min, V_max, N, N_c, Ts)

# Global path planning
initial = np.copy(X)
t0 = 10.0
growth = 0.5
logistic = lambda t: 1/(1 + np.exp(- growth * (t - t0)))
d_logistic = lambda t: growth * logistic(t) * (1 - logistic(t))
P_des = lambda t: goal * logistic(t) + initial * (1 - logistic(t))
V_des = lambda t: goal * d_logistic(t) - initial * d_logistic(t)

t = 0

vel = Twist()

while not rospy.is_shutdown():

    # Updating setpoint trajectory
    setpoint = np.ravel([np.append(P_des(t + k * Ts), V_des(t + k * Ts)) for k in range(0, N + 1)])

    # Updating initial conditions
    controller.x_0 = np.array([X[0], X[1], V[0], V[1]])

    # Computing optimal input values
    [_, acceleration] = controller.getNewVelocity(setpoint)

    [setpoint_pos.x, setpoint_pos.y] = P_des(t)

    [setpoint_vel.x, setpoint_vel.y] = V_des(t)

    acc = accelerationTransform(acceleration, vel.linear.x, vel.angular.z, orientation)

    vel.linear.x = vel.linear.x + acc[0] * Ts
    vel.angular.z = vel.angular.z + acc[1] * Ts

    pub.publish(vel)

    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)
    rospy.sleep(Ts)

    t += Ts
