import sys
import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from math import pi
from moveit_msgs.msg import Constraints, OrientationConstraint
from geometry_msgs.msg import Pose, Quaternion

# Initialize ROS node
rospy.init_node('move_robot_aruco_pose', anonymous=True)

# Set up TF buffer and listener
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# Initialize MoveIt! and get the planning scene interface
moveit_commander.roscpp_initialize(sys.argv)
scene = moveit_commander.PlanningSceneInterface()

# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# Set up MoveIt! planning group
robot = moveit_commander.RobotCommander()
group_name = 'manipulator'
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_end_effector_link("end_effector_2")

# Set up loop rate
rate = rospy.Rate(10.0)

def deg_to_rad(rad):
    deg = rad/180*pi
    return deg

# Main loop
while not rospy.is_shutdown():
    
    # We get the joint values from the group and change some of the values:
    joint_goal = move_group.get_current_joint_values()
    origPose = move_group.get_current_pose().pose
    joint_goal[0] = deg_to_rad(-90)
    joint_goal[1] = deg_to_rad(-40)
    joint_goal[2] = deg_to_rad(-140)
    joint_goal[3] = deg_to_rad(-60)
    joint_goal[4] = deg_to_rad(90)
    joint_goal[5] = deg_to_rad(90)
    
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    
    # Get the transform from the ArUco marker to the base_link frame
    try:
        marker_transform = tfBuffer.lookup_transform('world', 'marker_7_flipped', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        # If there is an exception while getting the transform, skip to the next iteration of the loop
        continue
        
    # Create a PoseStamped message with the marker pose in the base_link frame
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
    waypoints = []
    waypoints.append(origPose)
    waypoints.append(pose)
    
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
    
    # print("------------------------Pose-------------------------------")
    # print(pose)
    
    # set orientation constraint
    # oc = OrientationConstraint()
    # oc.link_name = "end_effector_2"
    # oc.header.frame_id = "world"
    # oc.orientation = Quaternion(0.707, 0.0, 0.0, 0.707)  # set the orientation to align with the x-axis
    # oc.absolute_x_axis_tolerance = 0.1
    # oc.absolute_y_axis_tolerance = 3.14
    # oc.absolute_z_axis_tolerance = 3.14
    # oc.weight = 1.0
    
    
    
    #Displaying trajectory in Rviz

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    #publish
    display_trajectory_publisher.publish(display_trajectory)

    rospy.sleep(2)
    move_group.execute(plan, wait=True)
    
    # Sleep for the loop rate
    rate.sleep()