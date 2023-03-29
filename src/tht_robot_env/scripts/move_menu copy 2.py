#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class MoveGroupInterface:
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

            # For testing:
            current_joints = move_group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 0.01)

def main():
    tutorial = MoveGroupPythonInterface()
    
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


if __name__ == '__main__':
    try:
        MoveGroupInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
