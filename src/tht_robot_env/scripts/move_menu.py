#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
from math import pi

class MoveGroupPythonInterface:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_interface')

        # Get parameters
        self.end_effector_link = rospy.get_param('~end_effector_link', 'end_effector_2')
        self.move_group_name = rospy.get_param('~move_group_name', 'manipulator')

        # Instantiate objects
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander(self.move_group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # Set object variables
        move_group.set_end_effector_link(self.end_effector_link)
        
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
        '''
        Function that rotates all joints with given rotations
        Joint 0-5
        ''' 
        move_group = self.move_group

        ## Planning to a Joint Goal
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
        return
            
    def joint_rotate_set(self, joint, rotate):
        '''
        Function that rotates given joint with given rotation
        Joint 0-5
        '''  
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[joint] = rotate*pi/180
        if(joint_goal[joint] > 2*pi):
            joint_goal[joint] = joint_goal[joint] - 2*pi
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        move_group
        return
    
    def joint_rotate_offset(self, joint, rotation):
        '''
        Function that offsets rotation on given joint with given rotation
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
        
    def go_to_pose_goal(self, x, y, z, qx, qy, qz, qw):
        '''
        Function that plans and execute motion to given pose
        ''' 
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy        
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
            
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

    def go_to_marker_pose(self, reference_frame, target_frame):
        '''
        Function that plans and execute motion to given pose with pose planner
        ''' 
        move_group = self.move_group
        
        # Converting from PoseStamped to pose for moveit_commander.MoveGroupCommander
        pose_goal = self.get_marker_transform(reference_frame, target_frame).pose
        
        # Set the target pose for the MoveIt! planning group
        move_group.set_pose_target(pose_goal) 
        
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()        
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()
        
    def get_marker_transform(self, reference_frame, target_frame):
        # Set up TF buffer and listener
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        
        if (reference_frame != self.move_group.get_planning_frame()):
            print("-------Given reference frame not equal to move_group reference frame-------") 
    
        try:
            marker_transform = tfBuffer.lookup_transform(reference_frame, target_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
            
        # Create a PoseStamped message with the marker pose in the base_link frame
        marker_pose = geometry_msgs.msg.PoseStamped()
        marker_pose.header.stamp = rospy.Time.now()
        marker_pose.header.frame_id = reference_frame
        marker_pose.pose.position.x = marker_transform.transform.translation.x
        marker_pose.pose.position.y = marker_transform.transform.translation.y
        marker_pose.pose.position.z = marker_transform.transform.translation.z
        marker_pose.pose.orientation.x = marker_transform.transform.rotation.x
        marker_pose.pose.orientation.y = marker_transform.transform.rotation.y
        marker_pose.pose.orientation.z = marker_transform.transform.rotation.z
        marker_pose.pose.orientation.w = marker_transform.transform.rotation.w
        
        return marker_pose
        
            
    def plan_cartesian_path(self, x, y, z, qx, qy, qz, qw):
        '''
        Function that plans and execute motion to given pose with cartesian paths
        ''' 
        try:
            move_group = self.move_group
            # Cartesian Paths
            waypoints = []
            current_pose = move_group.get_current_pose().pose
            wpose = geometry_msgs.msg.Pose()
            wpose.position.x = current_pose.position.x + x
            wpose.position.y = current_pose.position.y + y
            wpose.position.z = current_pose.position.z + z
            wpose.orientation.x = current_pose.orientation.x + qx
            wpose.orientation.y = current_pose.orientation.y + qy
            wpose.orientation.z = current_pose.orientation.z + qz
            wpose.orientation.w = current_pose.orientation.w + qw
            waypoints.append(wpose)

            # Plan cartesian path
            plan, fraction = move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0
            )

            return plan, fraction
        except KeyboardInterrupt:
            print("Interrupted!")
            return
    
    def display_trajectory(self, plan):
        '''
        Function that takes given plan and display it's trajectory in Rviz
        ''' 
        ## Displaying a Trajectory ---->
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory_publisher = self.display_trajectory_publisher

        # Create a `DisplayTrajectory` msg
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        # Populate `trajectory_start` with the current robot state
        display_trajectory.trajectory_start = self.robot.get_current_state()

        # Add the plan to the trajectory
        display_trajectory.trajectory.append(plan)

        # Publish the trajectory to RViz
        display_trajectory_publisher.publish(display_trajectory)
    
    def move_cartesian(self, x, y, z, qx, qy, qz, qw):
        if (x != 0 or y != 0 or z != 0):    
            try:
                cartesian_plan, fraction = self.plan_cartesian_path(x, y, z, qx, qy, qz, qw)
                self.display_trajectory(cartesian_plan)
                self.move_group.execute(cartesian_plan, wait=True)
            except KeyboardInterrupt:
                return
                
    def rotationCalibration(self):
        '''
        Rotates for image calibration skew.
        '''
        try:       
            self.joint_rotate_offset(4, -pi/6)
            self.joint_rotate_offset(4, 2*pi/6)
            self.joint_rotate_offset(4, -pi/6)
            
            self.joint_rotate_offset(3, -pi/6)
            self.joint_rotate_offset(3, 2*pi/6)
            self.joint_rotate_offset(3, -pi/6)
        except KeyboardInterrupt:
            exit()    

    def moveSquare(self):       
        self.move_cartesian(0.25, 0, 0, 0, 0, 0, 0)
        self.move_cartesian(0, 0, 0.25, 0, 0, 0, 0)
        self.move_cartesian(-0.25, 0, 0, 0, 0, 0, 0)
        self.move_cartesian(-0.25, 0, 0, 0, 0, 0, 0)
        self.move_cartesian(0, 0, -0.25, 0, 0, 0, 0)
        self.move_cartesian(0, 0, -0.25, 0, 0, 0, 0)
        self.move_cartesian(0.25 ,0 ,0, 0, 0, 0, 0)
        self.move_cartesian(0.25 ,0 ,0, 0, 0, 0, 0)
        self.move_cartesian(0, 0, 0.25, 0, 0, 0, 0)
        self.move_cartesian(-0.25 ,0 ,0, 0, 0, 0, 0)
    
    def cameraCalibration(self):       
        self.rotationCalibration()
        self.move_cartesian(0.25, 0, 0, 0, 0, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(0, 0, 0.25, 0, 0, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(-0.25, 0, 0, 0, 0, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(-0.25, 0, 0, 0, 0, 0, 0)
    
        self.rotationCalibration()
        self.move_cartesian(0, 0, -0.25, 0, 0, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(0, 0, -0.25, 0, 0, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(0.25 ,0 ,0, 0, 0, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(0.25 ,0 ,0, 0, 0, 0, 0)
        
        self.rotationCalibration()
        self.move_cartesian(0, 0, 0.25, 0, 0, 0, 0)
        self.move_cartesian(-0.25 ,0 ,0, 0, 0, 0, 0)
    
    
class PrintMenu:
    def __init__(self):
        pass
        
    def print_main_menu(self):
        '''Prints main menu.'''
        print("")
        menu_options = {
        1: 'Camera calibration',
        2: 'Joint goal',
        3: 'Joint rotation',
        4: 'Pose goal',
        5: 'Cartesian paths',
        6: 'Exit'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
    
    def print_cam_cali_menu(self): 
        '''Prints camera calibration menu.'''
        print("")
        print('Camera calibration sub-menu: ')
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
            
    def print_menu_2(self):
        '''
        Prints joint goal menu.
        1: 'Home position',
        2: 'Up position',
        3: 'Calibration position',
        4: 'Testing position',
        5: 'Back to main menu'
        '''
        print("")
        print('Joint goal sub-menu: ')
        menu_options = {
        1: 'Home position',
        2: 'Up position',
        3: 'Calibration position',
        4: 'Testing position',
        5: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
            
    def print_menu_3(self):
        '''Prints joint rotation menu.'''
        print("")
        print('Joint rotation sub-menu: ')
        menu_options = {
        1: 'Set joint value',
        2: 'Offset joint value',
        3: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
            
    def print_menu_4(self):
        '''Prints pose planner menu.'''
        print("")
        print("Pose goal sub-menu: ")
        menu_options = {
        1: 'Print pose',
        2: 'Set manual pose',
        3: 'Go to marker pose',
        4: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
            
    def print_menu_5(self):
        '''Prints cartesian planner menu.'''
        print("")
        print("Cartesian paths sub-menu: ")
        menu_options = {
        1: 'Print pose',
        2: 'Offset position',
        3: 'Offset rotation',
        4: 'Offset pos and rot',
        5: 'Back to main menu'
        }
        for key in menu_options.keys():
            print (key, '--', menu_options[key] )
            
def main():
    move_robot = MoveGroupPythonInterface()
    print_menu = PrintMenu()

    def option1():
        while (True):
            print("")
            print_menu.print_cam_cali_menu()
            option = ''
            try:
                option = int(input('Enter your choice: '))
            except:
                print('Wrong input. Please enter a number ...')
            #Check what choice was entered and act accordingly
            if option == 1:                
                print('Calibration start..')
                move_robot.go_to_joint_state(100, -160, 130, -150, -90, 0)
                print("Start position for calibration reached..")
                move_robot.moveSquare()
                move_robot.go_to_joint_state(100, -160, 130, -150, -90, 0)
                print("Calibration done! Check accuracy")
            elif option == 2:
                move_robot.go_to_joint_state(100, -160, 130, -150, -90, 0)
                print("Start position for calibration reached..")
                move_robot.move_cartesian(0.25, 0, 0)
                move_robot.move_cartesian(-0.50, 0, 0)
                move_robot.go_to_joint_state(100, -160, 130, -150, -90, 0)
            elif option == 4:
                move_robot.moveSquare()
            elif option == 5:
                move_robot.rotationCalibration()    
            elif option == 6:
                break
            else:
                print('Invalid option. Please enter a number between 1 and 6.')
    
    def option2():
        
        while(True):
            print_menu.print_menu_2()
            option = ''
            try:
                option = int(input('Enter your choice: '))
            except:
                print('Wrong input. Please enter a number ...')
            #Check what choice was entered and act accordingly
            if option == 1:
                move_robot.go_to_joint_state(0, 0, 0, 0, 0, 0)
            elif option == 2:
                move_robot.go_to_joint_state(0, -90, 0, -90, 0, 0)
            elif option == 3:
                move_robot.go_to_joint_state(90, -60, 90, -120, 90, 90)
            elif option == 4:
                move_robot.go_to_joint_state(-90, -35, -100, -110, 90, 90)
            elif option == 5:
                break
            else:
                print('Invalid option. Please enter a number between 1 and 5.')
    def option3():
        while(True):
            print_menu.print_menu_3()
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
                        move_robot.joint_rotate_set(joint, rotate_radians)
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
                        move_robot.joint_rotate_offset(joint, rotate_radians)
                        break
                    except ValueError:
                        print("Invalid input, try again")
            elif option == 3:
                break
            else:
                print('Invalid option. Please enter a number between 1 and 3.')
        
    def option4():
        while(True):
            print_menu.print_menu_4()
            option = ''
            try:
                option = int(input('Enter your choice: '))
            except:
                print('Wrong input. Please enter a number ...')
            #Check what choice was entered and act accordingly
            if option == 1:
                print(move_robot.move_group.get_current_pose().pose)
            elif option == 2:
                # Go to pose
                x = float(input("Enter x-position: "))
                y = float(input("Enter y-position: "))
                z = float(input("Enter z-position: "))
                qx = float(input("Enter x-orientation: "))
                qy = float(input("Enter y-orientation: "))
                qz = float(input("Enter z-orientation: "))
                qw = float(input("Enter w-orientation: "))
                move_robot.go_to_pose_goal(x, y, z, qx, qy, qz, qw)
            elif option == 3:
                move_robot.go_to_marker_pose('world', 'target_marker')
                
            elif option == 4:
                break
            else:
                print('Invalid option. Please enter a number between 1 and 3.')
                
    def option5():
        x = float(input("Enter x-offest: "))
        y = float(input("Enter y-offset: "))
        z = float(input("Enter z-offset: "))
        cartesian_plan, fraction = move_robot.plan_cartesian_path(x, y, z)
        move_robot.display_trajectory(cartesian_plan)
        move_robot.move_group.execute(cartesian_plan, wait=True)
        
    def exit_program():
        print("")
        print('Thank you! Come again!')
        exit()
    
    
    
    print("")
    print("----------------------------------------------------------")
    print("Welcome to tht robotics calibration menu")
    print("----------------------------------------------------------")
    print("Press Ctrl+C to exit at any time")
    print("")    
    
    #Dictionary with all options
    options = {
        1: option1,
        2: option2,
        3: option3,
        4: option4,
        5: option5,
        6: exit_program
    }
    
    # Main loop
    while(True):
        print_menu.print_main_menu()
        option = ''
        
        try:
            option = int(input('Enter your choice: '))
        except:
            print('Wrong input. Please enter a number ...')
            
        if option in options:
            options[option]()
        else:
            print('Invalid option. Please enter a number between 1 and 6.')



if __name__ == '__main__':
    try:
        while True:
            main()
            rospy.spin()
    except KeyboardInterrupt:
        print('Keyboard interrupt. Exiting program...')
    except rospy.ROSInterruptException:
        pass
    
