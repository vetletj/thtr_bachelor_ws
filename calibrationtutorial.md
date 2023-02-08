# Camera Calibration Manual

There are two types of cameras that the robotics system works with. Namely, the camera that is attached to the frame of the robot cell [henceforth called `Fixed_Cam` ] and the camera attached to the end-effector of the robot arm [henceforth called `Gripper_Cam` ].

The purpose of the calibration is to obtain a YAML file that contains the camera's starting pose [`yaml file name`] and obtaining the transformation matrix for the transformation from the co-ordinate frame of the camera to the co-ordinate frame of the robot base link.

*Note: Everytime a new terminal tab/window is opened, make sure to source the setup.bash file using the command `source ws_moveit/devel/setup.bash`*


<!--  Maybe make a section with common startup procedure and then spilt into Fixed, gripp and ts-->
## Contents

1. [Gripper Camera Calibration](#gripper-camera-calibration)

2. [Fixed Camera Calibration](#fixed-camera-calibration)

3. [Troubleshooting](#trouble-shooting)

## Gripper Camera Calibration

**A. Switching the Robot on and Setting up control from Computer**

On the UR control panel, power it on. 

Once the software finished loading, on the top banner, click on `Open`. 

Select  `Installation`  and from the list of files shown select: `thtr_station2.installation`

Open it and select `Update now`

On the bottom left select `Power on` and switch the robot on

Once the robot is active, select `Start`

*Safety Note:* Make sure to stay away from the robot arm duing this process. The mild ckracking sounds heard from the arm during this process is the sound of the motors starting up and the electromagnetic brakes on the motor switching off.

You can now exit the window

Back in the UR window, on the left of the top banner select `Program`

In the drop down menu on the left select `URCaps` and then `External Control`

On the computer from which the robot is to be controlled, launch the command
```
 roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=169.254.1.11
 ```
from the computer terminal. 

Once the comand is runnning wait till you see the status 
```
 Started controllers
 ```

Now, back on the UR control Panel, press the play button on the bottom ribbon and select `Play from Beginning Robot Program`

Now check the terminal on the computer, the status:
```
 Robot connected to reverse interface. ready to receive commands 
 ```
must be showing. 

The robot arm is now ready to be controlled from the Computer for the calibration process

**Enabling planning from the Computer**

Open a new terminal tab and launch
```
 roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch 
 ```
This will enable you to plan the robot arm's movements in rviz and then execute them in real life through the computer you are working on. 


**Setting up the Camera**

In a new terminal tab, launch 
```
 roslaunch depthai_examples tht_hndeye_rgb_publisher.launch
```

make sure to listen to the camera very carefully as you launch this command. You will hear a very soft click which is a confirmation that the camera is swtched on. If this click is not heard, Kill the process using `Ctrl+C` and re-launch the command till you can hear the click

**Visualization and Calibration Setup in Rviz**

In a new terminal, start rviz after sourcing the setup file

Set the world frame from map to `base_link`
Add a panel for motion planning. in the panel:
1. Change the planning group from `end_effector` to `manipulator`
2. Select the boxes for `Collision Aware IK` and to use `Cartesian Planner`. 


Add a new panel for image and select the topic as `add info`
You will now be able to see the live feed from the camera. Use this to ensure that there aren't any foreign items in your workspace.

Add a new panel for Hand eye calibration
change the planning group in all tabs from `end_effector` to `manipulator`
measure the side length of the AruCo markers and the separation between two AruCo markers
Input the measurements and set all other paramters
Make sure the change from Eye-to-hand to Eye-in-hand for the Fixed_cam calibration

Now add another panel for tag_detections/image
You will be able to see the tags and their respective reference frames detected and higlighted. Use this to ensure that all the Aruco Markers are detected. Also ensure that the lighting does not reflect off the AruCo markers such that the tags are not detected.

Notice how the z-axis is not aligned to what you would intuitively expect to be the z-axis of the aruco board

To remedy this, open a new terminal tab and after sourcing, run the following file
```
roslaunch depthai_examples tht_rgb_publisher.launch manual_focus:=120
``` 
Back in rviz, change the camera topic from to `/camera_info`

Notice, how the z-axis now is pointing in the correct direction in the Tag detections image. This is because the program launched compensates for a ineherent problem in the OAK Camera firmware.

You are now ready to proceed to the actual Calibration of the camera

**Calibration**

Set the frames as following in the Hand Eye Calibration Panel:
1. `sensor_frame` : `optical_frame`
2. `object_frame` : `handeye_target`
3. `endeffector` : `tool0`
4. `base_link` : `base_link`

Make sure that the planning group has been changed from `end_effector` to `manipulator`

Double check that the AX = XB solver is ` add info`

Now begins the tedious task of collecting samples.
To capture a sample, move the robot arm such that the AruCo board is in a different position and orientation with respect to the gripper camera. This can be done by referring to the tag-detections image in rviz. 

Once an orientation and position which is sufficiently different from the previous position and orientation is obtained, make sure that the tag_detections are recognised correctly and all the axes of the tags are aligned as they should be. If the entire board's axes are skewed due to one tag only, it is allowed that the problematic tag be covered such that the other tags can be detected accurately. 
Click on capture sample.
Again, for the next sample, it is imperative that the orientation and position be significantly different from the previous sample's orientation and position.

After 4 samples are taken, click on solve and note the reprojection error. If the error is less than 0.002 then proceed without concern but keep a watchful eye on the reprojection error. If the error is 0.005 or greater, head to section `add command` to find out how to proceed further.

Continue taking samples, while observing the reprojection error as you do so. Ensure that the reprojecction error does not have any sudden jumps of about 10 times in magnitude for example. If it does, follow section `add command` to correct it and proceed with the process. 

Take samples till you obtain a total of 15 Samples. More samples will be beneficial to the process since the accuracy will be imporved. However, is a a general observation that this process is very testing and taxing on one's patience. Therefore is is recommended to stick to the 15 samples minimum in order to obtain sufficient accuracy as well as preservation of sanity

To finish off, click on save camera pose, this will give us the launch file that we need for the robot arm. In the terminal, make sure to save the transformation matrix, reprojection error and markers spearatation in a text file for reference and further use since the transformation matrix is the transformation from the reference frame of the camera to the base link and this will be used later in further codes.

Finally, save the samples for future reference and analysis should it ever be needed.

The Gripper camera has been calibrated and is now ready for use.

## Fixed Camera Calibration

## Trouble-Shooting

Before looking at the other options here, please double check whether the following settings are set correctly:
1. In the HandEye Calibration panel:
    - a. Eye - to - Hand
    - b. manipulator group planning
    - c. correct topics
    - d. 

**1. Reprojection error is greater than 0.005** : trash everything you have done so far, grab the most kerosene you can and set fire to the world starting with the bot and the computer. Also summon satan's dragon. saves on having to use fossil fuels #savetheplanet

    In this case, click on Save samples to file and select a place for saving the samples collected so far. 
    These samples will be saved in a YAML format. Open the files and delete the position and orientation entries for the sample you beleive is 