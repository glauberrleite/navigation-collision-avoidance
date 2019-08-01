# Local Collision Avoidance Techniques on Multi-Agent Robot Navigation Systems

This repository covers a set of algorithms that provides local control strategies for autonomous robots navigating on a multi-agent scenario, where each robot tries to reach its goal as fast as it can, while mantaining a collision-free trajectory, based only on other agents velocity readings without any direct communication between them.

## About the code

The algorithms were created to work over the [ROS](https://www.ros.org/) environment, specifically the *Melodic* distribution running over Ubuntu 18.04, but they may work with other distributions.

## Dependencies

* [ROS Melodic](https://www.ros.org/)
* [Python 2.7](https://www.python.org/)
* [Gazebo](http://gazebosim.org/) *
* [Stage](http://wiki.ros.org/stage) *
* [OSQP (Operator Splitting Quadratic Program)](https://osqp.org/)

\* Generally comes with a full ROS install

## Simulation scenarios

Firstly, the ROS MASTER needed to be started:

`$ roscore`

### ROS Stage

There are worlds created for Stage simulation inside the *stage_worlds* folder. To run a simulation, e.g. *world1.world*:

`$ rosrun stage_ros stageros <PATH-TO-REPOSITORY-FOLDER>/stage_worlds/world1.world`

### Gazebo

For this simulation, a [Yujin Robot Kobuki](http://kobuki.yujinrobot.com/) was used, so it needs the driver URDF description and components for that robot. If it is not found within the default ROS install, it can be downloaded and built inside the catkin workspace using this [repository](https://github.com/yujinrobot/kobuki) (select the right ROS distribution branch).

With all set, the gazebo environment with the robots can be started using the launch file inside *mpc_orca* folder:

`$ roslaunch mpc_orca kobuki_duo.launch`