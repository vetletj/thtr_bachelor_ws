import sys
import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

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
group = moveit_commander.MoveGroupCommander(group_name)

# Set up loop rate
rate = rospy.Rate(10.0)

# Main loop
while not rospy.is_shutdown():
    # Get the transform from the ArUco marker to the base_link frame
    try:
        marker_transform = tfBuffer.lookup_transform('base_link', 'marker_7', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        # If there is an exception while getting the transform, skip to the next iteration of the loop
        continue
        
    # Create a PoseStamped message with the marker pose in the base_link frame
    marker_pose = geometry_msgs.msg.PoseStamped()
    marker_pose.header.stamp = rospy.Time.now()
    marker_pose.header.frame_id = 'base_link'
    marker_pose.pose.position.x = marker_transform.transform.translation.x
    marker_pose.pose.position.y = marker_transform.transform.translation.y
    marker_pose.pose.position.z = marker_transform.transform.translation.z
    marker_pose.pose.orientation.x = marker_transform.transform.rotation.x
    marker_pose.pose.orientation.y = marker_transform.transform.rotation.y
    marker_pose.pose.orientation.z = marker_transform.transform.rotation.z
    marker_pose.pose.orientation.w = marker_transform.transform.rotation.w
    print("PoseStamped: ")
    print(marker_pose)
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = marker_pose.pose.orientation.w
    pose_goal.position.x = marker_pose.pose.position.x
    pose_goal.position.y = marker_pose.pose.position.y
    pose_goal.position.z = marker_pose.pose.position.z
    print("Pose: ")
    print(pose_goal)
    

    # Set the target pose for the MoveIt! planning group
    group.set_pose_target(pose_goal)
    
    # Plan and execute the trajectory
    plan = group.plan()
    group.execute(plan)
    
    # Sleep for the loop rate
    rate.sleep()