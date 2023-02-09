#!/usr/bin/env python3

#* Example file for controling the robot with Moveit

#%%

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Initilazation of the moveit_commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorials", anonymous=True)

robot = moveit_commander.RobotCommander() # Provides information such as the robot’s kinematic model and the robot’s current joint states
scene = moveit_commander.PlanningSceneInterface() # This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
groupe_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(groupe_name) # This object is an interface to a planning group (group of joints). In this tutorial the group is the primary arm joints in the Panda robot, so we set the group’s name to “panda_arm”. If you are using a different robot, change this value to the name of your robot arm planning group. 

# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# Getting Basic Information

# # We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

# Cartesian Paths
# You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through. If executing interactively in a Python shell, set scale = 1.0.
#%%

input("Press Enter when ready to start calibration...")
# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = pi/2   # Joint_1 = 0°
joint_goal[1] = -pi/3   # Joint_2 = -60°
joint_goal[2] = 2.1293    # Joint_3 = 122°
joint_goal[3] = -2.6529   # Joint_4 = -152°
joint_goal[4] = pi/2 # Joint_5 = 90°
joint_goal[5] = 0   # Joint_6 = 0°

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()


waypoints = []
input("Press Enter to go to next pose...")
wpose = move_group.get_current_pose().pose
wpose.position.x += -0.4
wpose.position.y += 0.4 
wpose.position.z += 0
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

# Displaying a Trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

# Executing a Plan
move_group.execute(plan, wait=True)
# END

input("Press Enter to go to next pose...")
waypoints.clear()

wpose = move_group.get_current_pose().pose

wpose.position.x += 0.8
wpose.position.y += 0   
wpose.position.z += 0
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

move_group.execute(plan, wait=True)

input("Press Enter to go to next pose...")
waypoints.clear()
wpose = move_group.get_current_pose().pose

wpose.position.x += 0
wpose.position.y += -0.8
wpose.position.z += 0
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

move_group.execute(plan, wait=True)

input("Press Enter to go to next pose...")
waypoints.clear()
wpose = move_group.get_current_pose().pose

wpose.position.x += -0.8
wpose.position.y += 0
wpose.position.z += 0
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

move_group.execute(plan, wait=True)

