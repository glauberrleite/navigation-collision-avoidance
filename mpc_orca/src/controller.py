#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
from MPC_ORCA import MPC_ORCA
from pyorca import Agent

# Reads robot index and goal values
robot = int(sys.argv[1])
goal = np.array([float(sys.argv[2]), float(sys.argv[3])])

# Robot and controller parameters
RADIUS = 0.4
tau = 10
N = 10
Ts = 0.1
V_min = -1
V_max = 1

# Defining functions
def updateWorld(msg):
    """This funcion is called whenever the gazebo/model_states publishes. This function
    updates the world variables as fast as possible"""
    for i in range(n_robots):
        X[i] = np.array([float(msg.pose[model[i]].position.x), float(msg.pose[model[i]].position.y)])
        V[i] = np.array([float(msg.twist[model[i]].linear.x)/2, float(msg.twist[model[i]].linear.y)/2])
        orientation[i] = np.arctan2(2 * float(msg.pose[model[i]].orientation.w) * float(msg.pose[model[i]].orientation.z), 1 - 2 * float(msg.pose[model[i]].orientation.z)**2)

def accelerationTransform(a, v, w, theta_0):
    """This function applies the linearization transformation on acceleration values 
    based on equation 2.11
    """
    d = 0.2
    cos_theta = np.cos(theta_0)
    sin_theta = np.sin(theta_0)
    term1 = a[0] + v * w * sin_theta + d * (w**2) * cos_theta
    term2 = a[1] - v * w * cos_theta + d * (w**2) * sin_theta
    acc_linear = cos_theta * term1 + sin_theta * term2
    acc_angular = -(sin_theta/d) * term1 + (cos_theta/d) * term2

    return [acc_linear, acc_angular]

def update_positions(agents):
    """Each robot has a list of all agents in its neighborhood, including itself. This 
    function updates that list, not as fast as updateWorld().
    """
    for i in range(n_robots):
        agents[i].position = np.array(X[i])
        agents[i].velocity = np.array(V[i])
    return agents

# Initializing ros node
rospy.init_node('mpc_orca_controller_robot_' + str(robot))

# Waiting gazebo first message
data = rospy.wait_for_message('/gazebo/model_states', ModelStates)

n_robots = len(data.name) - 1

X = [np.zeros(2) for _ in range(n_robots)]
V = [np.zeros(2) for _ in range(n_robots)]
orientation = [0.0 for _ in range(n_robots)]
model = [i+1 for i in range(n_robots)]

# Getting robot model order on gazebo model_states
for i, value in enumerate(data.name):
    # Skipping i == 0 because it's the ground_plane state
    if i > 0:
        idx = value.split('_')
        model[int(idx[1])] = i

# Update initial X, V and orientation
updateWorld(data)

# Agents list
agents = []
for i in range(n_robots):
    agents.append(Agent(X[i], np.zeros(2), np.zeros(2), RADIUS))
     
# Subscribing on model_states instead of robot/odom, to avoid unnecessary noise
rospy.Subscriber('/gazebo/model_states', ModelStates, updateWorld)

# Velocity publisher
pub = rospy.Publisher('/robot_' + str(robot) + '/cmd_vel', Twist, queue_size=10)

# Setpoint Publishers
pub_setpoint_pos = rospy.Publisher('/robot_' + str(robot) + '/setpoint_pos', Vector3, queue_size=10)
pub_setpoint_vel = rospy.Publisher('/robot_' + str(robot) + '/setpoint_vel', Vector3, queue_size=10)

setpoint_pos = Vector3()
setpoint_vel = Vector3()

# Initializing Controllers
colliders = agents[:robot] + agents[robot + 1:]
controller = MPC_ORCA(agents[robot].position, V_min, V_max, N, Ts, colliders, tau, RADIUS)

# Global path planning
initial = np.copy(X[robot])
t_max = 10.0
growth = 0.5
logistic = lambda t: 1/(1 + np.exp(- growth * (t - t_max)))
d_logistic = lambda t: growth * logistic(t) * (1 - logistic(t))
P_des = lambda t: goal * logistic(t) + initial * (1 - logistic(t))
V_des = lambda t: goal * d_logistic(t) - initial * d_logistic(t)

t = 0

vel = Twist()
while not rospy.is_shutdown():

    # Updating controller list of agents
    agents = update_positions(agents)
    controller.agent = agents[robot]
    controller.colliders = agents[:robot] + agents[robot + 1:]
    
    # Updating setpoint trajectory
    setpoint = np.ravel([np.append(P_des(t + k * Ts), V_des(t + k * Ts)) for k in range(0, N + 1)])

    # Computing optimal input values
    [agents[robot].velocity, agents[robot].acceleration] = controller.compute(setpoint)

    # Saving setpoints
    [setpoint_pos.x, setpoint_pos.y] = P_des(t)
    [setpoint_vel.x, setpoint_vel.y] = V_des(t)

    # Linearization transformation for linear and angular acceleration
    [acc_linear, acc_angular] = accelerationTransform(agents[robot].acceleration, vel.linear.x, vel.angular.z, orientation[robot])

    # Integrating acceleration to get velocity commands
    vel.linear.x = vel.linear.x + acc_linear * Ts
    vel.angular.z = vel.angular.z + acc_angular * Ts

    # Publishing to topics    
    pub.publish(vel)    
    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)

    # Sleep until the next iteration
    rospy.sleep(Ts)
    t += Ts