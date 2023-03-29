#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('marker_flip_node')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # create the initial transform for marker_7_flipped
    initial_transform = geometry_msgs.msg.TransformStamped()
    initial_transform.header.stamp = rospy.Time.now()
    initial_transform.header.frame_id = "marker_7"
    initial_transform.child_frame_id = "marker_7_flipped"
    initial_transform.transform.translation = geometry_msgs.msg.Vector3(0, 0, 0)
    
    # create a quaternion for a 90-degree rotation around the y-axis
    q = geometry_msgs.msg.Quaternion()
    q.x = 0
    q.y = 0.707
    q.z = 0
    q.w = 0.707

    # apply the rotation to the child frame
    initial_transform.transform.rotation = q

    # publish the initial transform for marker_7_flipped
    broadcaster = tf2_ros.TransformBroadcaster()
    broadcaster.sendTransform(initial_transform)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # get the transform from marker_7 to marker_7_flipped
            trans = tfBuffer.lookup_transform('marker_7', 'marker_7_flipped', rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        # apply the rotation to the child frame
        trans.transform.rotation = q
        
        # create new TransformStamped message
        new_trans = geometry_msgs.msg.TransformStamped()
        new_trans.header.stamp = rospy.Time.now()
        new_trans.header.frame_id = trans.header.frame_id
        new_trans.child_frame_id = 'marker_7_flipped'
        new_trans.transform = trans.transform

        # publish the new TransformStamped message
        broadcaster.sendTransform(new_trans)

        rate.sleep()