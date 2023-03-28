import sys
import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from math import pi

# Initialize ROS node
rospy.init_node('move_robot_aruco_pose', anonymous=True)

# Set up TF buffer and listener
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# Initialize MoveIt! and get the planning scene interface
moveit_commander.roscpp_initialize(sys.argv)
scene = moveit_commander.PlanningSceneInterface()

# Set up MoveIt! planning group
robot = moveit_commander.RobotCommander()
group_name = 'manipulator'
move_group = moveit_commander.MoveGroupCommander(group_name)

# Set up loop rate
rate = rospy.Rate(20.0)

def deg_to_rad(rad):
    deg = rad/180*pi
    return deg

# Main loop
while not rospy.is_shutdown():
    
    # Get the transform from the ArUco marker to the base_link frame
    try:
        marker_transform = tfBuffer.lookup_transform('world', 'marker_7', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        # If there is an exception while getting the transform, skip to the next iteration of the loop
        continue
        
    # Create a PoseStamped message with the aruco marker pose in the base_link frame
    marker_pose = geometry_msgs.msg.PoseStamped()
    marker_pose.header.stamp = rospy.Time.now()
    marker_pose.header.frame_id = 'world'
    marker_pose.pose.position.x = marker_transform.transform.translation.x
    marker_pose.pose.position.y = marker_transform.transform.translation.y
    marker_pose.pose.position.z = marker_transform.transform.translation.z
    marker_pose.pose.orientation.x = marker_transform.transform.rotation.x
    marker_pose.pose.orientation.y = marker_transform.transform.rotation.y
    marker_pose.pose.orientation.z = marker_transform.transform.rotation.z
    marker_pose.pose.orientation.w = marker_transform.transform.rotation.w
    
    # Converting from PoseStamped to pose for moveit_commander.MoveGroupCommander
    pose = marker_pose.pose 
    
    print("------------------------Aruco 'marker_7' pose from 'world' reference frame-------------------------------")
    print(pose)
    
    
    print("------------------------Robot 'end_effector_2' pose from 'world' reference frame-------------------------------")
    print(move_group.get_current_pose().pose)
    print(move_group.get_current_pose().header.frame_id)

    
    
    # Sleep for the loop rate
    rate.sleep()