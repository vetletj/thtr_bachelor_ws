#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

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

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
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
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, j1, j2, j3, j4, j5, j6):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = j1*pi/180  
        joint_goal[1] = j2*pi/180 
        joint_goal[2] = j3*pi/180  
        joint_goal[3] = j4*pi/180  
        joint_goal[4] = j5*pi/180 
        joint_goal[5] = j6*pi/180

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def joint_rotate_set(self, joint, rotate):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[joint] = rotate
        move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def joint_rotate_offset(self, joint, rotation):
        '''
        Function that rotates given joint with given rotation
        Joint 0-5
        '''  
        goal_of_joints = self.move_group.get_current_joint_values()
        goal_of_joints[joint] = goal_of_joints[joint] + rotation
        
        if(goal_of_joints[joint] > 2*3.14):
            goal_of_joints[joint] = goal_of_joints[joint] - 2*3.14
            
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(goal_of_joints, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
                
    def go_to_pose_goal(self, x, y, z, w):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, x, y, z):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z += z  # First move up (z)
        wpose.position.x += x  # Second move forward/backwards in (x)
        wpose.position.y += y  # and sideways (y)
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
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL
    
    def move_cartesian(self, x, y, z):
        if (x != 0 or y != 0 or z != 0):    
            cartesian_plan, fraction = self.plan_cartesian_path(x, y, z)
            self.display_trajectory(cartesian_plan)
            self.execute_plan(cartesian_plan)
            
        
    def rotationCalibration(self):
        '''
        Rotates for image calibration skew.
        '''
        self.joint_rotate_offset(4, -pi/6)
        self.joint_rotate_offset(4, 2*pi/6)
        self.joint_rotate_offset(4, -pi/6)
        
        self.joint_rotate_offset(3, -pi/6)
        self.joint_rotate_offset(3, 2*pi/6)
        self.joint_rotate_offset(3, -pi/6)    
        #start = rospy.get_time    

    def moveSquare(self):       
        self.rotationCalibration()
        self.move_cartesian(0.25, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(0, 0, 0.25)
        
        self.rotationCalibration()
        self.move_cartesian(-0.25, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(-0.25, 0, 0)
    
        self.rotationCalibration()
        self.move_cartesian(0, 0, -0.25)
        
        self.rotationCalibration()
        self.move_cartesian(0, 0, -0.25)
        
        self.rotationCalibration()
        self.move_cartesian(0.25 ,0 ,0)
        
        self.rotationCalibration()
        self.move_cartesian(0.25 ,0 ,0)
        
        self.rotationCalibration()
        self.move_cartesian(0, 0, 0.25)
        self.move_cartesian(-0.25 ,0 ,0)

def main():
    tutorial = MoveGroupPythonInterfaceTutorial()
    
    def print_main_menu():
        menu_options = {
        1: 'Calibration',
        2: 'Joint goal',
        3: 'Joint rotation',
        4: 'Pose goal',
        5: 'Cartesian paths',
        6: 'Exit'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
            
    def print_menu_2():
        menu_options = {
        1: 'Home position',
        2: 'Up position',
        3: 'Calibration position',
        4: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
            
    def print_menu_3():
        menu_options = {
        1: 'Set joint value',
        2: 'Offset joint value',
        3: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
            
    def print_menu_4():
        menu_options = {
        1: 'Print pose',
        2: 'Set pose',
        3: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
            
    def print_menu_5():
        menu_options = {
        1: 'Print pose',
        2: 'Offset position',
        3: 'Offset rotation',
        4: 'Offset pos and rot',
        5: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )

    def print_cali_menu():
        menu_options = {
        1: 'Run whole calibration',
        2: 'Run calibration for x',
        3: 'Run calibration for y',
        4: 'Run calibration for desitance',
        5: 'Run calibration for scew',
        6: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )

    def option1():
        print("")
        print('Calibration sub-menu: ')
        while (True):
            print("")
            print_cali_menu()
            option = ''
            try:
                option = int(input('Enter your choice: '))
            except:
                print('Wrong input. Please enter a number ...')
            #Check what choice was entered and act accordingly
            if option == 1:                
                print('Calibration start..')
                tutorial.go_to_joint_state(100, -160, 130, -150, -90, 0)
                print("Start position for calibration reached..")
                tutorial.moveSquare()
                tutorial.go_to_joint_state(100, -160, 130, -150, -90, 0)
                print("Calibration done! Check accuracy")
            elif option == 2:
                tutorial.go_to_joint_state(100, -160, 130, -150, -90, 0)
                print("Start position for calibration reached..")
                tutorial.move_cartesian(0.25, 0, 0)
                tutorial.move_cartesian(-0.50, 0, 0)
                tutorial.go_to_joint_state(100, -160, 130, -150, -90, 0)
            elif option == 6:
                break
            else:
                print('Invalid option. Please enter a number between 1 and 6.')
       
    def option2():
        print("")
        print('Joint goal sub-menu: ')
        while(True):
            print_menu_2()
            option = ''
            try:
                option = int(input('Enter your choice: '))
            except:
                print('Wrong input. Please enter a number ...')
            #Check what choice was entered and act accordingly
            if option == 1:
                tutorial.go_to_joint_state(0, 0, 0, 0, 0, 0)
            elif option == 2:
                tutorial.go_to_joint_state(0, -90, 0, -90, 0, 0)
            elif option == 3:
                tutorial.go_to_joint_state(90, -180, 122, -152, 90, 0)
            elif option == 4:
                break
            else:
                print('Invalid option. Please enter a number between 1 and 4.')
        
    def option3():
        while(True):
            print("")
            print('Joint rotation sub-menu: ')
            print_menu_3()
            option = ''
            try:
                option = int(input('Enter your choice: '))
            except:
                print('Wrong input. Please enter a number ...')
            #Check what choice was entered and act accordingly
            if option == 1:
                while True:
                    try:
                        joint = int(input("Enter which joint you want to rotate around (1-6): "))
                        if joint < 1 or joint > 6:
                            raise ValueError
                        rotate_degrees = int(input("Enter value (-360 to 360) you want to rotate joint {joint}: "))
                        if rotate_degrees < -360 or rotate_degrees > 360:
                            raise ValueError                      
                        rotate_radians = rotate_degrees * pi/180
                        tutorial.joint_rotate_set(joint, rotate_radians)
                        break
                    except ValueError:
                        print("Invalid input, try again")
            elif option == 2:
                while True:
                    try:
                        joint = int(input("Enter which joint you want to rotate around (1-6): "))
                        if joint < 1 or joint > 6:
                            raise ValueError
                        rotate_degrees = int(input("Enter value (-360 to 360) you want to rotate joint {joint}: "))
                        if rotate_degrees < -360 or rotate_degrees > 360:
                            raise ValueError                      
                        rotate_radians = rotate_degrees * pi/180
                        tutorial.joint_rotate_offset(joint, rotate_radians)
                        break
                    except ValueError:
                        print("Invalid input, try again")
            elif option == 3:
                break
            else:
                print('Invalid option. Please enter a number between 1 and 3.')
        
    def option4():
        print("")
        print("Pose goal sub-menu: ")
        while(True):
            print_menu_4()
            option = ''
            try:
                option = int(input('Enter your choice: '))
            except:
                print('Wrong input. Please enter a number ...')
            #Check what choice was entered and act accordingly
            if option == 1:
                print(tutorial.move_group.get_current_pose().pose)
            elif option == 2:
                # Go to pose
                x = float(input("Enter x-position: "))
                y = float(input("Enter y-position: "))
                z = float(input("Enter z-position: "))
                tutorial.go_to_pose_goal(x, y, z, 0)
            elif option == 3:
                break
            else:
                print('Invalid option. Please enter a number between 1 and 3.')
                
    def option5():
        print("Cartesian paths sub-menu: ")
        x = float(input("Enter x-offest: "))
        y = float(input("Enter y-offset: "))
        z = float(input("Enter z-offset: "))
        cartesian_plan, fraction = tutorial.plan_cartesian_path(x, y, z)
        tutorial.display_trajectory(cartesian_plan)
        tutorial.execute_plan(cartesian_plan)
    
    print("----------------------------------------------------------")
    print("Welcome to tht robotics calibration menu")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    
    try:    
        while(True):
            print("")
            print_main_menu()
            option = ''
            try:
                option = int(input('Enter your choice: '))
            except:
                print('Wrong input. Please enter a number ...')
            #Check what choice was entered and act accordingly
            if option == 1:
                option1()
            elif option == 2:
                option2()
            elif option == 3:
                option3()
            elif option == 4:
                option4()
            elif option == 5:
                option5()    
            elif option == 6:
                print("")
                print('Thank you! Come again!')
                exit()
            else:
                print('Invalid option. Please enter a number between 1 and 6.')
                    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()