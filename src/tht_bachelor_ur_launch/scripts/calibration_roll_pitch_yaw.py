#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
from math import pi
import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


class MoveGroupPythonInterface:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('calib_move')

        # Get parameters
        self.end_effector_link = rospy.get_param('~end_effector_link', 'end_effector_1')
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

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        success = move_group.go(joint_goal, wait=True)
        
        while not success and not rospy.is_shutdown():
            rospy.loginfo("Planning failed. Trying again...")
            success = move_group.go(joint_goal, wait=True)

            if rospy.is_shutdown():
                rospy.loginfo("ROS is shutting down. Stopping planning...")
                move_group.stop()
                return
            
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
        
    def offset_pose_goal(self, x, y, z, roll, pitch, yaw):
        '''
        Function that offsets from current pose
        '''
        roll = roll*pi/180
        pitch = pitch*pi/180
        yaw = yaw*pi/180
        
        current_pose = self.move_group.get_current_pose().pose
        current_orientation = euler_from_quaternion([current_pose.orientation.x, 
                                                    current_pose.orientation.y, 
                                                    current_pose.orientation.z, 
                                                    current_pose.orientation.w])
        
        roll = current_orientation[0] + roll
        pitch = current_orientation[1] + pitch
        yaw = current_orientation[2] + yaw
         
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = current_pose.position.x + x
        pose_goal.position.y = current_pose.position.y + y
        pose_goal.position.z = current_pose.position.z + z
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]        
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        
        while not success and not rospy.is_shutdown():
            rospy.loginfo("Planning failed. Trying again...")
            success = move_group.go(wait=True)

            if rospy.is_shutdown():
                rospy.loginfo("ROS is shutting down. Stopping planning...")
                move_group.stop()
                return
                
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

    def go_to_marker_pose(self, reference_frame, target_frame):
        '''
        Function that plans and execute motion to given pose with pose planner
        ''' 
        # Converting from PoseStamped to pose for moveit_commander.MoveGroupCommander
        pose_goal = self.get_marker_transform(reference_frame, target_frame).pose
        
        self.go_to_pose_goal(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z,
                            pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w)
        
    def get_marker_transform(self, reference_frame, target_frame):
        '''
        Function that calculates the pose of target_frame in the reference frame coordinate system
        ''' 
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
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z += z  # First move up (z)
        wpose.position.x += x  # Second move forward/backwards in (x)
        wpose.position.y += y  # and sideways (y)
        wpose.orientation.x += qx
        wpose.orientation.y += qy
        wpose.orientation.z += qz
        wpose.orientation.w += qw
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
    
    def plan_cartesian_path_pose(self, target_pose):
        '''
        Function that plans cartesian path to pose goal
        ''' 
        move_group = self.move_group
        
        waypoints = []
        
        # Get the current pose of the end-effector
        start_pose = move_group.get_current_pose().pose

        # Append the current pose as the first waypoint
        waypoints.append(copy.deepcopy(start_pose))

        # Append the target pose as the second waypoint
        waypoints.append(copy.deepcopy(target_pose))
        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)  # waypoints to follow, eef_step, jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

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

    def execute_plan(self, plan):
        move_group = self.move_group
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    
    def execute_poses(self, pose_num, x, y, z, roll, pitch, yaw):
        print(f"Go to pose {pose_num}")
        self.offset_pose_goal(x, y, z, roll, pitch, yaw)
        print("Take sample! 5 seconds until next pose.")
        rospy.sleep(5)
        print(" ")
    
    def six_poses_ik(self):
        try:
            poses  = [
                ("ROLL", 0, 0, 0, 45, 0, 0), # 45 degrees around x-axis
                ("ROLL", 0, 0, 0, -90, 0, 0), # -45 degrees around x-axis
                ("PITCH", 0, 0, 0, 45, 45, 0), # 45 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -90, 0), # -45 degrees around y-axis
                ("YAW", 0, 0, 0, 0, 45, 45), # 45 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -90), # -45 degrees around z-axis
                ]
            
            current_pose = ""
            for i, pose in enumerate(poses):
                pose_name, x, y, z, roll, pitch, yaw = pose
                if rospy.is_shutdown():
                    print("ROS interrupt. Exiting program...")
                    return
                elif pose_name != current_pose:
                    print(f"\n----------{pose_name}----------")
                    current_pose = pose_name
                self.execute_poses(i+1, x, y, z, roll, pitch, yaw)
                
            print("Calibration done!")
        
        except rospy.ROSInterruptException:
            print("ROS interrupt. Exiting program...")
            
    def twelve_poses_ik(self):
        try:
            poses  = [
                ("ROLL", 0, 0, 0, 45, 0, 0), # 45 degrees around x-axis
                ("ROLL", 0, 0, 0, -15, 0, 0), # 30 degrees around x-axis
                ("ROLL", 0, 0, 0, -60, 0, 0), # -30 degrees around x-axis
                ("ROLL", 0, 0, 0, -15, 0, 0), # -45 degrees around x-axis
                ("PITCH", 0, 0, 0, 45, 45, 0), # 45 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -15, 0), # 30 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -60, 0), # -30 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -15, 0), # -45 degrees around y-axis
                ("YAW", 0, 0, 0, 0, 45, 45), # 45 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -15), # 30 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -60), # -30 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -15), # -45 degrees around z-axis
                ]
            
            current_pose = ""
            for i, pose in enumerate(poses):
                pose_name, x, y, z, roll, pitch, yaw = pose
                if rospy.is_shutdown():
                    print("ROS interrupt. Exiting program...")
                    return
                elif pose_name != current_pose:
                    print(f"\n----------{pose_name}----------")
                    current_pose = pose_name
                self.execute_poses(i+1, x, y, z, roll, pitch, yaw)
                
            print("Calibration done!")
                       
        except rospy.ROSInterruptException:
            print("ROS interrupt. Exiting program...")
    
    def eighteen_poses_ik(self):
        try:
            poses  = [
                ("ROLL", 0, 0, 0, 45, 0, 0), # 45 degrees around x-axis
                ("ROLL", 0, 0, 0, -15, 0, 0), # 30 degrees around x-axis
                ("ROLL", 0, 0, 0, -60, 0, 0), # -30 degrees around x-axis
                ("ROLL", 0, 0, 0, -15, 0, 0), # -45 degrees around x-axis
                ("PITCH", 0, 0, 0, 45, 45, 0), # 45 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -15, 0), # 30 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -60, 0), # -30 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -15, 0), # -45 degrees around y-axis
                ("YAW", 0, 0, 0, 0, 45, 45), # 45 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -15), # 30 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -60), # -30 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -15), # -45 degrees around z-axis
                ("ROLL + PITCH", 0, 0, 0, 20, 20, 45), # X = 20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 0, -40, 0), # X = 20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -40, 0, 0), # X = -20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 0, 40, 0), # X = -20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 20, 0, 30), # X = 0, Y = 20, Z = 30
                ("ROLL + PITCH", 0, 0, 0, 0, 0, 30), # X = 0, Y = 20, Z = 60
                ]
            
            current_pose = ""
            for i, pose in enumerate(poses):
                pose_name, x, y, z, roll, pitch, yaw = pose
                if rospy.is_shutdown():
                    print("ROS interrupt. Exiting program...")
                    return
                elif pose_name != current_pose:
                    print(f"\n----------{pose_name}----------")
                    current_pose = pose_name
                self.execute_poses(i+1, x, y, z, roll, pitch, yaw)
                
            print("Calibration done!")
                       
        except rospy.ROSInterruptException:
            print("ROS interrupt. Exiting program...")
            
    def twentyfour_poses_ik(self):
        try:
            poses  = [
                ("ROLL", 0, 0, 0, 60, 0, 0), # 60 degrees around x-axis
                ("ROLL", 0, 0, 0, -20, 0, 0), # 40 degrees around x-axis
                ("ROLL", 0, 0, 0, -20, 0, 0), # 20 degrees around x-axis
                ("ROLL", 0, 0, 0, -80, 0, 0), # -60 degrees around x-axis
                ("ROLL", 0, 0, 0, 20, 0, 0), # -40 degrees around x-axis
                ("ROLL", 0, 0, 0, 20, 0, 0), # -20 degrees around x-axis
                ("PITCH", 0, 0, 0, 20, 60, 0), # 60 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -20, 0), # 40 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -20, 0), # 20 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -80, 0), # -60 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, 20, 0), # -40 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, 20, 0), # -20 degrees around y-axis
                ("YAW", 0, 0, 0, 0, 20, 60), # 60 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -20), # 40 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -20), # 20 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -80), # -60 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, 20), # -40 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, 20), # -20 degrees around z-axis
                ("ROLL + PITCH", 0, 0, 0, 20, 20, 20), # X = 20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 0, -40, 0), # X = 20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -40, 0, 0), # X = -20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 0, 40, 0), # X = -20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 70, 30, 30), # X = 50, Y = 50, Z = 30
                ("ROLL + PITCH", 0, 0, 0, 0, -100, 30), # X = 50, Y = -50, Z = 60
                ]
            
            current_pose = ""
            for i, pose in enumerate(poses):
                pose_name, x, y, z, roll, pitch, yaw = pose
                if rospy.is_shutdown():
                    print("ROS interrupt. Exiting program...")
                    return
                elif pose_name != current_pose:
                    print(f"\n----------{pose_name}----------")
                    current_pose = pose_name
                self.execute_poses(i+1, x, y, z, roll, pitch, yaw)
                
            print("Calibration done!")
                       
        except rospy.ROSInterruptException:
            print("ROS interrupt. Exiting program...")    
    
    def thirty_poses_ik(self):
        try:
            poses  = [
                ("ROLL", 0, 0, 0, 60, 0, 0), # 60 degrees around x-axis
                ("ROLL", 0, 0, 0, -20, 0, 0), # 40 degrees around x-axis
                ("ROLL", 0, 0, 0, -20, 0, 0), # 20 degrees around x-axis
                ("ROLL", 0, 0, 0, -80, 0, 0), # -60 degrees around x-axis
                ("ROLL", 0, 0, 0, 20, 0, 0), # -40 degrees around x-axis
                ("ROLL", 0, 0, 0, 20, 0, 0), # -20 degrees around x-axis
                ("PITCH", 0, 0, 0, 20, 60, 0), # 60 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -20, 0), # 40 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -20, 0), # 20 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -80, 0), # -60 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, 20, 0), # -40 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, 20, 0), # -20 degrees around y-axis
                ("YAW", 0, 0, 0, 0, 20, 60), # 60 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -20), # 40 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -20), # 20 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -80), # -60 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, 20), # -40 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, 20), # -20 degrees around z-axis
                
                ("ROLL + PITCH", 0, 0, 0, 20, 20, 0), # X = 20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, 15, 0), # X = 35, Y = 35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, 15, 0), # X = 50, Y = 50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, -30, -70, 0), # X = 20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, -15, 0), # X = 35, Y = -35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, -15, 0), # X = 50, Y = -50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, -70, 30, 0), # X = -20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, -15, 0), # X = -35, Y = -35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, -15, 0), # X = -50, Y = -50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, 30, 70, 0), # X = -20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, 15, 0), # X = -35, Y = 35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, 15, 0), # X = -50, Y = 50, Z = 0
                ]
            
            current_pose = ""
            for i, pose in enumerate(poses):
                pose_name, x, y, z, roll, pitch, yaw = pose
                if rospy.is_shutdown():
                    print("ROS interrupt. Exiting program...")
                    return
                elif pose_name != current_pose:
                    print(f"\n----------{pose_name}----------")
                    current_pose = pose_name
                self.execute_poses(i+1, x, y, z, roll, pitch, yaw)
                
            print("Calibration done!")
            
        except rospy.ROSInterruptException:
            print("ROS interrupt. Exiting program...")
    
    def z_roll_pitch_yaw_poses(self):
        try:
            poses  = [
                ("ROLL", 0, 0, 0, 45, 0, 0), # 45 degrees around x-axis
                ("ROLL", 0, 0, 0, -90, 0, 0), # -45 degrees around x-axis
                ("PITCH", 0, 0, 0, 45, 45, 0), # 45 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -90, 0), # -45 degrees around y-axis
                ("YAW", 0, 0, 0, 0, 45, 45), # 45 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -90), # -45 degrees around z-axis
                
                
                ("ROLL", 0, 0, 0, 60, 0, 0), # 60 degrees around x-axis
                ("ROLL", 0, 0, 0, -20, 0, 0), # 40 degrees around x-axis
                ("ROLL", 0, 0, 0, -20, 0, 0), # 20 degrees around x-axis
                ("ROLL", 0, 0, 0, -80, 0, 0), # -60 degrees around x-axis
                ("ROLL", 0, 0, 0, 20, 0, 0), # -40 degrees around x-axis
                ("ROLL", 0, 0, 0, 20, 0, 0), # -20 degrees around x-axis
                ("PITCH", 0, 0, 0, 20, 60, 0), # 60 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -20, 0), # 40 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -20, 0), # 20 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, -80, 0), # -60 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, 20, 0), # -40 degrees around y-axis
                ("PITCH", 0, 0, 0, 0, 20, 0), # -20 degrees around y-axis
                ("YAW", 0, 0, 0, 0, 20, 60), # 60 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -20), # 40 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -20), # 20 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, -80), # -60 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, 20), # -40 degrees around z-axis
                ("YAW", 0, 0, 0, 0, 0, 20), # -20 degrees around z-axis
                
                ("ROLL + PITCH", 0, 0, 0, 20, 20, 0), # X = 20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, 15, 0), # X = 35, Y = 35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, 15, 0), # X = 50, Y = 50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, -30, -70, 0), # X = 20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, -15, 0), # X = 35, Y = -35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, -15, 0), # X = 50, Y = -50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, -70, 30, 0), # X = -20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, -15, 0), # X = -35, Y = -35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, -15, 0), # X = -50, Y = -50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, 30, 70, 0), # X = -20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, 15, 0), # X = -35, Y = 35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, 15, 0), # X = -50, Y = 50, Z = 0
                ]
            
            current_pose = ""
            for i, pose in enumerate(poses):
                pose_name, x, y, z, roll, pitch, yaw = pose
                if rospy.is_shutdown():
                    print("ROS interrupt. Exiting program...")
                    return
                elif pose_name != current_pose:
                    print(f"\n----------{pose_name}----------")
                    current_pose = pose_name
                self.execute_poses(i+1, x, y, z, roll, pitch, yaw)
                
            print("Calibration done!")
            
        except rospy.ROSInterruptException:
            print("ROS interrupt. Exiting program...")
    
    def test_poses_ik(self):
        try:
            poses  = [
                ("ROLL + PITCH", 0, 0, 0, 20, 20, 0), # X = 20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, 15, 0), # X = 35, Y = 35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, 15, 0), # X = 50, Y = 50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, -30, -70, 0), # X = 20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, -15, 0), # X = 35, Y = -35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, 15, -15, 0), # X = 50, Y = -50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, -70, 30, 0), # X = -20, Y = -20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, -15, 0), # X = -35, Y = -35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, -15, 0), # X = -50, Y = -50, Z = 0
                
                ("ROLL + PITCH", 0, 0, 0, 30, 70, 0), # X = -20, Y = 20, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, 15, 0), # X = -35, Y = 35, Z = 0
                ("ROLL + PITCH", 0, 0, 0, -15, 15, 0), # X = -50, Y = 50, Z = 0
                ]
            
            current_pose = ""
            for i, pose in enumerate(poses):
                pose_name, x, y, z, roll, pitch, yaw = pose
                if rospy.is_shutdown():
                    print("ROS interrupt. Exiting program...")
                    return
                elif pose_name != current_pose:
                    print(f"\n----------{pose_name}----------")
                    current_pose = pose_name
                self.execute_poses(i+1, x, y, z, roll, pitch, yaw)
                
            print("Calibration done!")
                       
        except rospy.ROSInterruptException:
            print("ROS interrupt. Exiting program...")
    
    
    
                                              
def main():
# Main loop
    move_robot = MoveGroupPythonInterface()
    while not rospy.is_shutdown():
        print("Number of poses: ")
        menu_options = {
            1: 'start position 80cm',
            2: 'start position 70cm',
            3: 'start position 60cm',
            4: 'start position 50cm',
            5: 'start position',
            6: 'move out of camera view',
            10: 'roll poses',
            11: 'pitch poses',
            12: 'yaw poses',
            13: 'roll+pitch poses 1',
            14: 'roll+pitch poses 2',
            20: 'test poses IK',
            21: '6 poses IK',
            22: '12 poses IK',
            23: '18 poses IK',
            24: '24 poses IK',
            25: '30 poses IK',
            30: '5 poses',
            31: '10 poses',
            32: '15 poses',
            33: '20 poses',
            9: 'quit'
        }
        # Printing menu options
        for key in menu_options.keys():
            print(key, '--', menu_options[key]) 
        
        option = ""
        try:
            option = int(input('Enter your choice: '))
        except:
            print('Wrong input. Please enter a number ...')
        
        #Check what choice was entered and act accordingly
        if(option == 1):
            move_robot.go_to_joint_state(52, 43, -60, 107, -90, -140)
        elif(option == 2):
            move_robot.go_to_joint_state(51, 32, -73, 131, -89, -135)
        elif(option == 3):
            move_robot.go_to_joint_state(46, 16, -78, 152, -89, -130)
        elif(option == 4):
            move_robot.go_to_joint_state(41, -3, -54, 147, -89, -126)
        elif(option == 5):
            move_robot.go_to_joint_state(40, 13, -69, 146, -89, 53)
        elif(option == 6):
            move_robot.go_to_joint_state(-72, -60, 13, -133, 85, 0)
        elif(option == 10):
            move_robot.roll_poses_ik()
        elif(option == 11):
            move_robot.pitch_poses_ik()
        elif(option == 12):
            move_robot.yaw_poses_ik()
        elif(option == 13):
            move_robot.roll_pitch_poses_ik_1()
        elif(option == 14):
            move_robot.roll_pitch_poses_ik_2()
        elif(option == 20):
            print(" ")
            print("Go to start position")
            move_robot.go_to_joint_state(40, 13, -69, 146, -89, 53)
            print("Take sample! 5 seconds until next pose.")
            rospy.sleep(5)
            move_robot.test_poses_ik()
        elif(option == 21):
            print(" ")
            print("Go to start position")
            move_robot.go_to_joint_state(40, 13, -69, 146, -89, 53)
            print("Take sample! 5 seconds until next pose.")
            rospy.sleep(5)
            move_robot.six_poses_ik()
        elif(option == 22):
            print(" ")
            print("Go to start position")
            move_robot.go_to_joint_state(40, 13, -69, 146, -89, 53)
            print("Take sample! 5 seconds until next pose.")
            rospy.sleep(5)
            move_robot.twelve_poses_ik()
        elif(option == 23):
            print(" ")
            print("Go to start position")
            move_robot.go_to_joint_state(40, 13, -69, 146, -89, 53)
            print("Take sample! 5 seconds until next pose.")
            rospy.sleep(5)
            move_robot.eighteen_poses_ik()
        elif(option == 24):
            print(" ")
            print("Go to start position")
            move_robot.go_to_joint_state(40, 13, -69, 146, -89, 53)
            print("Take sample! 5 seconds until next pose.")
            rospy.sleep(5)
            move_robot.twentyfour_poses_ik()
        elif(option == 25):
            print(" ")
            print("Go to start position")
            move_robot.go_to_joint_state(40, 13, -69, 146, -89, 53)
            print("Take sample! 5 seconds until next pose.")
            rospy.sleep(5)
            move_robot.thirty_poses_ik()
        elif(option == 9):
            return
        else:
            print('Invalid option. Please try again...')
            
        if rospy.is_shutdown():
            print("ROS interrupt. Exiting program...")
            return


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Keyboard interrupt. Exiting program...')