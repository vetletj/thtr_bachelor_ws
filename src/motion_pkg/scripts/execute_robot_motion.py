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
import copy

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32


# Initilazation of the moveit_commander and rospy node
rospy.init_node("move_group_python_interface_tutorials", anonymous=True)

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander() # Provides information such as the robot’s kinematic model and the robot’s current joint states
scene = moveit_commander.PlanningSceneInterface() # This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
groupe = moveit_commander.MoveGroupCommander("manipulator") # here defining a planing group



def postion_tracking(msg):
    rospy.loginfo("recived data from x: %f, y: %f", msg.position.x, msg.position.y)
    waypoints = []
    scale=1
    #groupe.set_position_target([msg.position.x,msg.position.y,msg.position.z])
    wpose = groupe.get_current_pose().pose
    wpose.position.x += scale * msg.position.x 
    wpose.position.y += scale * msg.position.y
    wpose.position.z += scale * msg.position.z
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = groupe.compute_cartesian_path(waypoints, 0.01, 0.0)  # jump_threshold
    groupe.execute(plan, wait=True)
    #plan = groupe.go(wait=True)
    #groupe.stop()
    #groupe.clear_pose_targets()

rospy.Subscriber("follow_object", Pose, postion_tracking, queue_size=10)

rospy.loginfo("move_group_python_interface_tutorials started and subsribed to tracking_object")

rospy.spin()