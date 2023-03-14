#!/usr/bin/env python3  
# Script for testing the OAK D poe pro camera

# importing modules
import cv2
import depthai as dai
import os
import numpy as np
import cv2.aruco as aruco
import math as m
from tf.transformations import quaternion_matrix
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

#Moveit node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveRobot", anonymous=True)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)


# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

#Tf node
rospy.init_node('tf_listener')
listener = tf.TransformListener

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        robTransform = listener.lookupTransform('/base_lin','/end_effector',rospy.Time(0))
    except:
        continue
    
allCharucoCorners = np.array([])
allCharucoIds = np.array([])
allRotationVectors = np.array([])
allTranlationVectors = np.array([]
                                
                                )
def quat2ang(quat):
    #normalise quaternions
    norm = m.sqrt(x*x + y*y + z*z + w*w)
    x = quat[0]/norm
    y = quat[1]/norm
    z = quat[2]/norm
    w = quat[3]/norm
    #convert to angle
    angle = 2*m.acos(w)
    s = m.sqrt(1-w*w)
    if(s<0.00001):
        #rotational direction can be ignored because of low angle
        rx = 1
        ry = 0
        rz = 0
    else:
        rx = x /s
        ry = y/s
        rz = z/s
    #normalize rotation
    norm = m.sqrt(rx*rx+ry*ry+rz*rz)
    rx = rx /norm*angle
    ry = ry/norm*angle
    rz = rz/norm*angle
    ang = np.array([[rx],[ry],[rz]])
    return ang

def ang2quat(ang):
    rx = ang[0][0]
    ry = ang[1][0]
    rz = ang[2][0]
    
    angle = m.sqrt(rx*rx+ry*ry+rz*rz)
    quat = []
    quat[0] = rx/angle*m.sin(angle/2)
    quat[1] = ry/angle*m.sin(angle/2)
    quat[2] = rz/angle*m.sin(angle/2)
    quat[3] = m.cos(angle/2)
    
    return quat

def transformToRotationVector(transform):
    global allRotationVectors
    global allTranlationVectors
    
    
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    
    rvec = quat2ang(rotation)
    allRotationVectors.append(rvec)
    
    tvec = np.array([[translation.x], [translation.y], [translation.z]])
    allTranlationVectors.append(tvec)
    return


def addSample():
    
    if charucoIds is not None:
        global allCharucoCorners
        global allCharucoIds
        
        allCharucoCorners = np.append(allCharucoCorners, charucoCorners)
        allCharucoIds = np.append(allCharucoIds,charucoIds)
        transformToRotationVector(robTransform)
        print("Sample saved.")
    else:
        print("All CharucoCorners not detected. ")
        
def calibrate():
    
    cameraMatrix, distCoeffs, reprojError, r_camera2end, t_camera2end = cv2.calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imageSize)
    R_camera2gripper, T_camera2gripper = cv2.calibrateHandEye(allRotationVectors, allTranlationVectors, r_camera2end, t_camera2end)
    
    
    return   

# create pipeline
pipeline = dai.Pipeline()

# define sources and output
camRgb = pipeline.createColorCamera()
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
# NOTE: multiple resolutions could be chosen
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setFps(10) # settings the frame rate
camRgb.setIspScale(2, 3)

videoEnc = pipeline.createVideoEncoder()
videoEnc.setDefaultProfilePreset(camRgb.getFps(), dai.VideoEncoderProperties.Profile.MJPEG)
camRgb.video.link(videoEnc.input)

xout = pipeline.createXLinkOut()
xout.setStreamName("mjpeg")
videoEnc.bitstream.link(xout.input)

# TODO: if it is needed to set a manual focus
# camRgb.initialControl.setManualFocus(180)

#* depth map

# define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

camRgb.initialControl.setManualFocus(120)

# properties (resolution can be changed)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

dai.node.MonoCamera.setFps(monoLeft, 10.0)
dai.node.MonoCamera.setFps(monoRight, 10.0)

stereo.initialConfig.setConfidenceThreshold(255)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(False)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")
stereo.depth.link(xoutDepth.input)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("disp")
stereo.disparity.link(xoutDepth.input)
deviceInfo = dai.DeviceInfo('169.254.1.222') #Can change ip to xx.xxx.xxx.200

# connect to device and start pipeline
with dai.Device(pipeline, deviceInfo) as device:
# output queue will be used to get the encoded data from the output defined above
    # RGB image
    q = device.getOutputQueue(name="mjpeg", maxSize=30, blocking=True)
    
    # depth
    depthQueue = device.getOutputQueue(name="depth")
    
    # disparity map
    dispQ = device.getOutputQueue(name="disp")
    
    #device.setIrLaserDotProjectorBrightness(100) # in mA, 0..1200
    #device.setIrFloodLightBrightness(0) # in mA, 0..1500
    
    while True:
        # RGB image data
        data = q.get().getData()  # blocking call, will wait until new data has arrived
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        
        # depth image data
        depthFrame = depthQueue.get().getFrame()
        # depth_frame = cv2.imdecode(depthFrame, cv2.IMREAD_COLOR)
        
        # get disparity frame for nicer depth visualization
        disp = dispQ.get().getFrame()
        disp = (disp * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
        disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

        # show rgb image stream
        #print("[INFO] Loading image stream...")
        #cv2.imshow("MJPEG", frame)
        
        # show depth frame
        #cv2.imshow("depth", disp)
        
        #cv2.imshow("depth_frame", depthFrame)
        
        #print(frame.shape)
        #print(depthFrame.shape)
        
        

        
        frameCopy = frame.copy()
        
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        board = aruco.CharucoBoard_create(7, 5, 0.04, 0.02, dictionary)
        params = aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
        height, width = frameCopy.shape[:2]
        imageSize = [width, height]

        
        markerCorners, markerIds, _ = aruco.detectMarkers(frameCopy, dictionary, parameters=params)
        if markerIds is not None and len(markerIds) > 0:
            aruco.drawDetectedMarkers(frameCopy, markerCorners, markerIds)
            retval, charucoCorners, charucoIds  = aruco.interpolateCornersCharuco(markerCorners, markerIds, frameCopy, board)  
            
            if isinstance(charucoCorners, np.ndarray):
                if charucoIds is not None and charucoCorners is not None and charucoCorners.shape[0] != 0:
                    aruco.drawDetectedCornersCharuco(frameCopy, charucoCorners, charucoIds, (0, 255, 0))
        cv2.imshow('out', frameCopy)        
        
        
            
        
  
            
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('s'): # Add sample 
            addSample()
            print(allCharucoCorners)
            print(allCharucoIds)
        elif key == ord('c'): #Calibrate
            calibrate()
        