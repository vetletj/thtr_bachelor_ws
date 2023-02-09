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
import tf
import math

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveRobot", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

waypoints = []

def moveRobot(x, y, z): # Moves robot in given directions.
    if (x != 0 or y != 0 or z != 0):
        waypoints.clear()
        wpose = move_group.get_current_pose().pose
        print(wpose)
        wpose.position.x += x  # and forward/backwards (x)   
        wpose.position.y += y  # and sideways (y)
        wpose.position.z += z  # First move up (z)     
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
        #Execute plan
        move_group.execute(plan, wait=True)
        print("Robot moved!")

def rotateRobot(roll, pitch, yaw):
    wpose = move_group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))
    
    # Calculating from quaternions to euler cordinates
    orientation = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
    (existing_roll, existing_pitch, existing_yaw) = tf.transformations.euler_from_quaternion(orientation)
    
    print("roll: " + str(existing_roll) + " pitch: " + str(existing_pitch) + " yaw: " + str(existing_yaw)    )
    
    if(roll != 0 or pitch != 0 or yaw != 0):
        
        roll = existing_roll + roll
        if(roll > 2*math.pi):
            roll = roll - 2*math.pi
        
        pitch = existing_pitch + pitch
        if(pitch > 2*math.pi):
            pitch = pitch - 2*math.pi    

        yaw = existing_yaw + yaw
        if(yaw > 2*math.pi):
            yaw = yaw - 2*math.pi
        
        quaternions = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='ryxz')
        
        wpose.orientation.x = quaternions[0]
        wpose.orientation.y = quaternions[1]
        wpose.orientation.z = quaternions[2]
        wpose.orientation.w = quaternions[3]
        #waypoints.append(copy.deepcopy(wpose))
    
        move_group.set_pose_target(wpose)
        
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()
        print("moved!")
    
    
move = input("Want new pose (y/n)? ")
while (move == "y"):
    x = float(input("Enter x: "))
    y = float(input("Enter y: "))
    z = float(input("Enter z: "))
    moveRobot(x,y,z)
    move = input("Want new pose (y/n)? ")

    

moveRobot(0.2, -0.1, 0)    
rotateRobot(math.pi/2, -math.pi/2, 0)
