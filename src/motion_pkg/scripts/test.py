#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


#Set parameters

launchfile = rospy.get_param('~')

if(launchfile.has_param('rows')):
    rows = launchfile.get('rows')
else:
    rospy.loggerr("Failed to load rows. Set to 9.")
    rows = 9;
if(launchfile.has_param("columns")):
    columns = launchfile.get("columns")
else:
    rospy.loggerr("Failed to load columns. Set to 6.")
    columns = 6
    
# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)



# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback_right(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

   
    # Show the converted image
    cv2.imshow("right image", cv_image)
    cv2.waitKey(1)
# Define a callback for the Image message
def image_callback_left(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

   
    # Show the converted image
    cv2.imshow("left image", cv_image)
    cv2.waitKey(1)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
left_image = rospy.Subscriber("/stereo_publisher/left/image", Image, image_callback_right)
right_image = rospy.Subscriber("/stereo_publisher/right/image", Image, image_callback_left)


# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()