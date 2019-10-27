#!/usr/bin/env python2

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from MPC_ORCA import MPC_ORCA
from pyorca import Agent

RADIUS = 0.5
tau = 5

N = 10
Ts = 0.1
X = [[-7., 0.], [7., 0.], [0, 7.], [0., -7.]]
orientation = [0, np.pi, -np.pi/2, np.pi/2]
#X = [[-7., 0.], [7., 0.]]
#orientation = [0, np.pi]
V = [[0., 0.] for _ in xrange(len(X))]
V_min = [-1.0 for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = [[7., 0.], [-7., 0.], [0.0, -7.], [0., 7.]]
#goal = [[7., 0.], [-7., 0.]]
model = [i+1 for i in xrange(len(X))]

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

def updateWorld(msg):
    for i in xrange(len(X)):
        X[i] = np.array([float(msg.pose[model[i]].position.x), float(msg.pose[model[i]].position.y)])
        orientation[i] = 2 * np.arctan2(float(msg.pose[model[i]].orientation.z), float(msg.pose[model[i]].orientation.w))
    if (orientation[i] > np.pi):
        # For gazebo odom quaternion
        orientation[i] = 2 * np.arctan2(-float(msg.pose[model[i]].orientation.z), -float(msg.pose[model[i]].orientation.w))

rospy.init_node('diff_controller')

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

# Getting robot model order on gazebo model_states
data = rospy.wait_for_message('/gazebo/model_states', ModelStates)
for i, value in enumerate(data.name):
    # Skipping i == 0 because it's the ground_plane state
    if i > 0:
        idx = value.split('_')
        model[int(idx[1])] = i

while not rospy.is_shutdown():
    
    agents = update_positions(agents)

    for i, agent in enumerate(agents):
        # Computing desired velocity
        V_des = goal[i] - X[i]
        P_des = X[i] + V_des * Ts

        controller[i].agent = agents[i]
        controller[i].colliders = agents[:i] + agents[i + 1:]

        agents[i].velocity = controller[i].getNewVelocity(P_des, V_des)
    
        if i == 1:
            setpoint_pos.x = P_des[0]
            setpoint_pos.y = P_des[1]

            setpoint_vel.x = V_des[0]
            setpoint_vel.y = V_des[1]

    for i in xrange(len(X)):
        vel = Twist()
        [vel.linear.x, vel.angular.z] = velocityTransform(agents[i].velocity, orientation[i])

        pub[i].publish(vel)
    
    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)
    rospy.sleep(Ts)
