import sys
import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped

# Initialize MoveIt and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('tf_move_group_pose_planning', anonymous=True)

# Instantiate a MoveGroupCommander object
move_group = moveit_commander.MoveGroupCommander('manipulator')

# Define the end-effector link
end_effector_link = 'your_end_effector_link'

# Set the reference frame for the pose
reference_frame = 'base_link'

# TF 
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10.0)
# Transform from end-effector to base_link
robTransform = tfBuffer.lookup_transform('tool0', 'base_link', rospy.Time())
rotation = robTransform.transform.rotation
translation = robTransform.transform.translation

# Create a PoseStamped message from the end-effector pose
ee_pose_stamped = PoseStamped() 
ee_pose_stamped.header.frame_id = reference_frame
ee_pose_stamped.pose.position.x = ee_pose[0, 3]
ee_pose_stamped.pose.position.y = ee_pose[1, 3]
ee_pose_stamped.pose.position.z = ee_pose[2, 3]
ee_pose_stamped.pose.orientation.x = ee_pose[0, 0]
ee_pose_stamped.pose.orientation.y = ee_pose[1, 0]
ee_pose_stamped.pose.orientation.z = ee_pose[2, 0]
ee_pose_stamped.pose.orientation.w = np.sqrt(1 + ee_pose[0, 0] + ee_pose[1, 1] + ee_pose[2, 2]) / 2



# Set the pose of the end-effector as the target
move_group.set_pose_target(ee_pose_stamped, end_effector_link)

# Plan the trajectory to the target
plan = move_group.go(wait=True)

# Stop MoveIt
moveit_commander.roscpp_shutdown()
