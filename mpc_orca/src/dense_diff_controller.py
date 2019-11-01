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
X = []
X.append(np.array([-10., 0.]))
X.append(np.array([10., 0.]))
X.append(np.array([0, 10.]))
X.append(np.array([0., -10.]))
X.append(np.array([-10., 10.]))
X.append(np.array([-10., -10.]))
X.append(np.array([10, -10.]))
X.append(np.array([10., 10.]))
orientation = [0, np.pi, -np.pi/2, np.pi/2, -np.pi/4, np.pi/4, 3*np.pi/4, -3*np.pi/4]
V = [[0., 0.] for _ in xrange(len(X))]
V_min = [-1.0 for _ in xrange(len(X))]
V_max = [1.0 for _ in xrange(len(X))]
goal = []
goal.append(np.array([10., 0.]))
goal.append(np.array([-10., 0.]))
goal.append(np.array([0., -10.]))
goal.append(np.array([0., 10.]))
goal.append(np.array([10., -10.]))
goal.append(np.array([10., 10.]))
goal.append(np.array([-10., 10.]))
goal.append(np.array([-10., -10.]))

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
    X[0] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
    orientation[0] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[0] > np.pi):
        # For gazebo odom quaternion
        orientation[0] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_1(msg):
    X[1] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
    orientation[1] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[1] > np.pi):
        # For gazebo odom quaternion
        orientation[1] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_2(msg):
    X[2] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
    orientation[2] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[2] > np.pi):
        # For gazebo odom quaternion
        orientation[2] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_3(msg):
    X[3] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
    orientation[3] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[3] > np.pi):
        # For gazebo odom quaternion
        orientation[3] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_4(msg):
    X[4] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
    orientation[4] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[4] > np.pi):
        # For gazebo odom quaternion
        orientation[4] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_5(msg):
    X[5] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
    orientation[5] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[5] > np.pi):
        # For gazebo odom quaternion
        orientation[5] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_6(msg):
    X[6] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
    orientation[6] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[6] > np.pi):
        # For gazebo odom quaternion
        orientation[6] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))
def callback_7(msg):
    X[7] = np.array([float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)])
    orientation[7] = 2 * np.arctan2(float(msg.pose.pose.orientation.z), float(msg.pose.pose.orientation.w))
    if (orientation[7] > np.pi):
        # For gazebo odom quaternion
        orientation[7] = 2 * np.arctan2(-float(msg.pose.pose.orientation.z), -float(msg.pose.pose.orientation.w))

rospy.init_node('diff_controller')

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

# Global path planning
initial = np.copy(X)
setting_time = 20.0
P_des = lambda t, i: (t > setting_time) * goal[i] + (t <= setting_time) * (goal[i] * (t/setting_time) + initial[i] * (1 - t/setting_time))
V_des = lambda t, i: (t > setting_time) * np.zeros(2) + (t <= setting_time) * (goal[i] * (1/setting_time) - initial[i] * (1/setting_time))
#V_des = lambda t, i: P_des(t, i) - X[i]

t = 0

while not rospy.is_shutdown():
    
    agents = update_positions(agents)

    for i, agent in enumerate(agents):

        controller[i].agent = agents[i]
        controller[i].colliders = agents[:i] + agents[i + 1:]

        agents[i].velocity = controller[i].getNewVelocity(P_des(t, i), V_des(t, i))
    
        if i == 4:
            setpoint_pos.x = P_des(t, i)[0]
            setpoint_pos.y = P_des(t, i)[1]

            setpoint_vel.x = V_des(t, i)[0]
            setpoint_vel.y = V_des(t, i)[1]

    for i in xrange(len(X)):
        vel = Twist()
        [vel.linear.x, vel.angular.z] = velocityTransform(agents[i].velocity, orientation[i])

        pub[i].publish(vel)
    
    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)

    rospy.sleep(Ts)
    t += Ts