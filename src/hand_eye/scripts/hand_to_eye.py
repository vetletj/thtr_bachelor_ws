#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# Import libraries
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import numpy as np

#initiate node
rospy.init_node("hand_to_eye_calibrator", anonymous=True)


#initialize lists
charucoCornersRight = np.array([])
charucoCornersLeft = np.array([])
charucoID = np.array([])
robotPoses = np.array([])



#Set parameters
def setParams():
    if(rospy.has_param("~rows")):
        rows = rospy.get_param('~rows')
    else:
        rospy.logerr("Failed to load rows. Set to 9.")
        rows = 9
        
    if(rospy.has_param("~columns")):
        columns = rospy.get_param("~columns")
    else:
        rospy.logerr("Failed to load columns. Set to 6.")
        columns = 6
        
    if(rospy.has_param("~square_length")):
        square_length = rospy.get_param("~square_length")
    else:
        rospy.logerr("Failed to square_length. Set to 0.02.")
        square_length = 0.02
        
    if(rospy.has_param("~marker_length")):
        marker_length = rospy.get_param("~marker_length")
    else:
        rospy.logerr("Failed to load marker length. Set to 0.05.")
        marker_length = 0.05
        
    if(rospy.has_param("~robot_base")):
        robot_base = rospy.get_param("~robot_base")
    else:
        rospy.logerr("Failed to load robot base. Set to 'base'.")
        robot_base = "base"   
    
    if(rospy.has_param("~gripper")):
        gripper = rospy.get_param("~gripper")
    else:
        rospy.logerr("Failed to load gripper. Set to 'tool0'.")
        gripper = "tool0"  

    # Set up the aruco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)    
    #Creating CharucoBoard
    charuco = aruco.CharucoBoard_create(columns, rows, square_length, marker_length, aruco_dict)   
    
    


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
    '''
    # Show the converted image
    cv2.imshow("right image", cv_image)
    cv2.waitKey(1)
    '''
    
# Define a callback for the Image message
def image_callback_left(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
   
    '''# Show the converted image
    cv2.imshow("left image", cv_image)
    cv2.waitKey(1)'''

def detectCharucoBoardWithoutCalibration(image):
    
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
    board = aruco.CharucoBoard_create(7, 5, 1, 0.5, dictionary)
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
    
    
    while True:
            imageCopy = image
            #markerCorners, markerIds, _ = aruco.detectMarkers(image, aruco.getPredefinedDictionary(board), parameters=params)
            cv2.imshow("image", image)
            cv2.waitKey()
            '''  
            if len(markerIds) > 0:
                aruco.drawDetectedMarkers(imageCopy, markerCorners, markerIds)
                charucoCorners, charucoIds, _ = aruco.interpolateCornersCharuco(markerCorners, markerIds, image, board)
                    
                if len(charucoIds) > 0:
                    aruco.drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, (255, 0, 0))
                
            cv2.imshow('out', imageCopy)
            key = cv2.waitKey(30)
            if key == 27:
                break
                '''
    cv2.destroyAllWindows()

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
left_image = rospy.Subscriber("/stereo_publisher/left/image", Image, image_callback_right)
right_image = rospy.Subscriber("/stereo_publisher/right/image", Image, image_callback_left)
#detectCharucoBoardWithoutCalibration()


    



# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()