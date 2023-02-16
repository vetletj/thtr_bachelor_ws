#include "ros/ros.h"
#include <iostream>
#include <cstdio>
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

dai::Pipeline createPipeline(int man_foc)
{
    dai::Pipeline pipeline;
    
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();    
    auto controlIn = pipeline.create<dai::node::XLinkIn>();    
    
    colorCam->initialControl.setManualFocus(man_foc);
    //colorCam.initialControl.setAutoFocusMode(dai.RawCameraControl.AutoFocusMode.OFF)

    controlIn->setStreamName("control");
    xlinkOut->setStreamName("video");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->video.link(xlinkOut->input);   

    return pipeline;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "tht_manual_focus_rgb");
    ros::NodeHandle pnh("~");
    
    std::string tfPrefix;
    std::string camera_param_uri;
    int manual_focus;
    int badParams = 0;

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("camera_param_uri", camera_param_uri);
    badParams += !pnh.getParam("manual_focus", manual_focus);

    if (badParams > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }
    
    printf("\n\n\n\n\n*************************\nManual Focus is %d\n*************************\nTHT-ROBOTICS MANUAL FOCUS\n*************************", manual_focus);
    
    dai::Pipeline pipeline = createPipeline(manual_focus);
    dai::Device device(pipeline);    
    //auto controlQueue = device.getInputQueue("control");  
    std::shared_ptr<dai::DataInputQueue> controlQueue = device.getInputQueue("control"); 
    
    dai::CameraControl ctrl;
    ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::OFF);
    ctrl.setAutoFocusTrigger();
    controlQueue->send(ctrl);
    ctrl.setManualFocus(manual_focus);
    controlQueue->send(ctrl);    
    
    std::shared_ptr<dai::DataOutputQueue> imgQueue = device.getOutputQueue("video", 30, false);
    
    std::string color_uri = camera_param_uri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imgQueue,
                                                                                  pnh, 
                                                                                  std::string("color/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                  &rgbConverter, 
                                                                                  std::placeholders::_1, 
                                                                                  std::placeholders::_2) , 
                                                                                  30,
                                                                                  color_uri,
                                                                                  "color");

    rgbPublish.addPublisherCallback();
    ros::spin();

    return 0;
}

