#!/usr/bin/env python3

#* Example file for controling the robot with Moveit

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

# Planning to a Joint Goal - Moving a robot to different desired states
# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 6*pi/180
joint_goal[1] = -54 * pi / 180
joint_goal[2] = 17 * pi/180
joint_goal[3] = 10*pi/180
joint_goal[4] = 34*pi/180
joint_goal[5] = 0  # 1/6 of a turn
#joint_goal[6] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()
# END

# Planning to a Pose Goal
# We can plan a motion for this group to a desired pose for the end-effector
# (planing based on the desired motion of the end-effector)

# First planning the pose goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4 # 0.4
pose_goal.position.y = 0.1 # 0.1
pose_goal.position.z = 0.4 # 0.4

move_group.set_pose_target(pose_goal)

# Now, we call the planner to compute the plan and execute it.
plan = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()
# END

# Cartesian Paths
# You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through. If executing interactively in a Python shell, set scale = 1.0.
waypoints = []
scale=1

wpose = move_group.get_current_pose().pose
wpose.position.z -= scale * 0.1  # First move up (z)
wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.1  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:
# return plan, fraction

# Displaying a Trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

# Executing a Plan
move_group.execute(plan, wait=True)
# END

# Adding Objects to the Planning Scene
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "tool0"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.z = 0.11  # above the panda_hand frame
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

# Ensuring Collision Updates Are Received
start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
box_is_known=False
box_is_attached=False
while (seconds - start < timeout) and not rospy.is_shutdown():
    # Test if the box is in attached objects
    attached_objects = scene.get_attached_objects([box_name])
    is_attached = len(attached_objects.keys()) > 0

    # Test if the box is in the scene.
    # Note that attaching the box will remove it from known_objects
    is_known = box_name in scene.get_known_object_names()

    # Test if we are in the expected state
    if (box_is_attached == is_attached) and (box_is_known == is_known):
        Update =  True
    else:
        Update =  False


    # Sleep so that we give other threads time on the processor
    rospy.sleep(0.1)
    seconds = rospy.get_time()

# Attaching Objects to the Robot
grasping_group = "manipulator"
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(eef_link, box_name, touch_links=touch_links)

# If we exited the while loop without returning then we timed out
#return False

# Planning to a Joint Goal - Moving a robot to different desired states
# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
joint_goal[5] = 0  # 1/6 of a turn
#joint_goal[6] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()
# END

# Detaching Objects from the Robot
scene.remove_attached_object(eef_link, name=box_name)

# Removing Objects from the Planning Scene
scene.remove_world_object(box_name)