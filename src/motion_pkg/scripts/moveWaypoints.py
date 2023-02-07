#!/usr/bin/env python3  

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveRobot", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

#up = [-2.5890535847263143, -2.180785651606902, 0.7321361632517993, -1.6947975508798923, 2.5890364566936235, 0.0004057191601143373]
#group_variable_values = up


# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)


start = rospy.get_time
def moveRobot(x, y, z): # Moves robot in given directions. 
    scale = 1
    waypoints = []

    
    wpose = move_group.get_current_pose().pose
    
    if (z!=0):
        wpose.position.z -= scale * z  # First move up (z)   
        waypoints.append(copy.deepcopy(wpose))

    if (y!=0):
        wpose.position.y += scale * y  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))
    if(x!=0):
        wpose.position.x += scale * x  # and forward/backwards (x)
        waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    #Displaying trajectory in Rviz

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    #publish
    display_trajectory_publisher.publish(display_trajectory)

    rospy.sleep(2)
    move_group.execute(plan, wait=True)
    print("moved!")

moveRobot(0.1,0,-0.2)
    
moveRobot(-0.1, 0.2, 0)

moveRobot(0, 0 , 0.2)

