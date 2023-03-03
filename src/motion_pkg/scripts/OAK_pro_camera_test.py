# Script for testing the OAK D poe pro camera

# importing modules
import cv2
import depthai as dai
import os
import numpy as np

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
        print("[INFO] Loading image stream...")
        cv2.imshow("MJPEG", frame)
        
        # show depth frame
        #cv2.imshow("depth", disp)
        
        #cv2.imshow("depth_frame", depthFrame)
        
        print(frame.shape)
        print(depthFrame.shape)
        
        
        # Detecting ArUco markers with OpenCV and Python
        arucoType = cv2.aruco.DICT_6X6_50
        arucoDict = cv2.aruco.Dictionary_get(arucoType)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
            parameters=arucoParams)

        if cv2.waitKey(1) == ord('q'):
            break
        
        
        
        