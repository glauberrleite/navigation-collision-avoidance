#!/usr/bin/env python2

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from MPC_ORCA import MPC_ORCA
from pyorca import Agent

RADIUS = 0.4
tau = 10

N = 10
Ts = 0.1
X = []
X.append(np.array([-7., 0.]))
X.append(np.array([7., 0.]))
X.append(np.array([0, 7.]))
X.append(np.array([0., -7.]))
orientation = [0, np.pi, -np.pi/2, np.pi/2]
V = [[0., 0.] for _ in xrange(len(X))]
V_min = [-1 for _ in xrange(len(X))]
V_max = [1 for _ in xrange(len(X))]
goal = []
goal.append(np.array([7., 0.]))
goal.append(np.array([-7., 0.]))
goal.append(np.array([0., -7.]))
goal.append(np.array([0., 7.]))
model = [i+1 for i in xrange(len(X))]

agents = []

for i in xrange(len(X)):
    agents.append(Agent(X[i], np.zeros(2), np.zeros(2), RADIUS))

def velocityTransform(v, a, theta_0):
    
    linear = np.sqrt(v[0]**2 + v[1]**2)
    #angular = (v[0]*a[1] - v[1]*a[0])/linear
    angular = np.arctan2(v[1], v[0]) - theta_0 

    # Handling singularity
    if np.abs(angular) > np.pi:
        angular -= np.sign(angular) * 2 * np.pi
 
    return [linear, angular]

def update_positions(agents):
    for i in xrange(len(X)):
        agents[i].position = np.array(X[i])
    return agents

def updateWorld(msg):
    for i in xrange(len(X)):
        X[i] = np.array([float(msg.pose[model[i]].position.x), float(msg.pose[model[i]].position.y)])
        orientation[i] = 2 * np.arctan2(float(msg.pose[model[i]].orientation.z), float(msg.pose[model[i]].orientation.w))
    if (orientation[i] > np.pi):
        # For gazebo odom quaternion
        orientation[i] = 2 * np.arctan2(-float(msg.pose[model[i]].orientation.z), -float(msg.pose[model[i]].orientation.w))

rospy.init_node('diff_controller')

# Getting robot model order on gazebo model_states
data = rospy.wait_for_message('/gazebo/model_states', ModelStates)
for i, value in enumerate(data.name):
    # Skipping i == 0 because it's the ground_plane state
    if i > 0:
        idx = value.split('_')
        model[int(idx[1])] = i

# Subscribing on model_states instead of robot/odom, to avoid unnecessary noise
rospy.Subscriber('/gazebo/model_states', ModelStates, updateWorld)
pub = []

# Velocity publishers
pub.append(rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10))
pub.append(rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=10))

# Setpoint Publishers
pub_setpoint_pos = rospy.Publisher('/setpoint_pos', Vector3, queue_size=10)
pub_setpoint_vel = rospy.Publisher('/setpoint_vel', Vector3, queue_size=10)

setpoint_pos = Vector3()
setpoint_vel = Vector3()

# Initializing Controllers
controller = []
for i, agent in enumerate(agents):
    colliders = agents[:i] + agents[i + 1:]
    controller.append(MPC_ORCA(agent.position, V_min[i], V_max[i], N, Ts, colliders, tau, agent.radius))

# Global path planning
initial = np.copy(X)
t0 = 3.0
growth = 0.9
logistic = lambda t, t0, growth: 1/(1 + np.exp(- growth * (t - t0)))
d_logistic = lambda t, t0, growth: growth * logistic(t, t0, growth) * (1 - logistic(t, t0, growth))
P_des = lambda t, i: goal[i] * logistic(t, t0, growth) + initial[i] * (1 - logistic(t, t0, growth))
V_des = lambda t, i: goal[i] * d_logistic(t, t0, growth) - initial[i] * d_logistic(t, t0, growth)

t = 0
while not rospy.is_shutdown():
    
    agents = update_positions(agents)

    for i, agent in enumerate(agents):
        # Computing desired velocity
        controller[i].agent = agents[i]
        controller[i].colliders = agents[:i] + agents[i + 1:]

        [agents[i].velocity, agents[i].acceleration] = controller[i].getNewVelocity(P_des(t, i), V_des(t, i))
    
        if i == 0:
            [setpoint_pos.x, setpoint_pos.y] = P_des(t, i)

            [setpoint_vel.x, setpoint_vel.y] = V_des(t, i)

    for i in xrange(len(X)):
        vel = Twist()
        [vel.linear.x, vel.angular.z] = velocityTransform(agents[i].velocity, agents[i].acceleration, orientation[i])

        pub[i].publish(vel)
    
    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)
    rospy.sleep(Ts)

    t += Ts
