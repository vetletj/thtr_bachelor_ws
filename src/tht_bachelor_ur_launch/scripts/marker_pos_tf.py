#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    # Wait for the transformation to become available
    listener.waitForTransform("base_link", "marker_1", rospy.Time(), rospy.Duration(5.0))
    listener.waitForTransform("base_link", "marker_2", rospy.Time(), rospy.Duration(5.0))
    listener.waitForTransform("base_link", "marker_3", rospy.Time(), rospy.Duration(5.0))
    listener.waitForTransform("base_link", "marker_4", rospy.Time(), rospy.Duration(5.0))

    rate = rospy.Rate(10.0)
    
    file_path = "/home/thtr/Desktop/ThtR_Bachelor/hand-to-eye/translation_data.txt"
    
    sample_count = 0
    max_samples = 3
    
    with open(file_path, "w") as file:
        while not rospy.is_shutdown() and sample_count < max_samples:
            try:
                # Get the transformation between base_link and marker_1
                (trans1, rot1) = listener.lookupTransform("base_link", "marker_1", rospy.Time(0))
                (trans2, rot2) = listener.lookupTransform("base_link", "marker_2", rospy.Time(0))
                (trans3, rot3) = listener.lookupTransform("base_link", "marker_3", rospy.Time(0))
                (trans4, rot4) = listener.lookupTransform("base_link", "marker_4", rospy.Time(0))
                
                file.write(f"Sample {sample_count+1}:\n")
                file.write("Marker 1 Translation: [%.6f, %.6f, %.6f]\n" % (trans1[0], trans1[1], trans1[2]))
                file.write("Marker 2 Translation: [%.6f, %.6f, %.6f]\n" % (trans2[0], trans2[1], trans2[2]))
                file.write("Marker 3 Translation: [%.6f, %.6f, %.6f]\n" % (trans3[0], trans3[1], trans3[2]))
                file.write("Marker 4 Translation: [%.6f, %.6f, %.6f]\n" % (trans4[0], trans4[1], trans4[2]))
                file.write("\n")
                sample_count += 1
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get transform")

            rate.sleep()
            
        # Shutdown after collecting the desired number of samples
        print(f"Finished collecting {max_samples} samples")
        rospy.signal_shutdown("Finished collecting samples")
