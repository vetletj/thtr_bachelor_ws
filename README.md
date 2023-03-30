# tht_bachelor_ws
Workspace for bachelor assignment at tht robotics

## Full launch sequence for hand-to-eye calibration with OAK-D camera
1. Launch robot description.
* For physical robot:
```bash 
roslaunch tht_robot_env tht_bachelor_bringup.launch
```
* For simulated robot:
```bash 
roslaunch tht_robot_env load_rosparam_robot_description.launch
```
2. Launch MoveIt planner:
* For physical robot:
```bash 
roslaunch ur10e_moveit_config moveit_planning_execution.launch
```
* Simulated robot:
```bash 
roslaunch ur10e_moveit_config demo.launch load_robot_description:=false
```
3. Launch camera driver:
* OAK-D: 
```bash 
roslaunch tht_depthai tht_rgb_publisher.launch
```
* Launch camera processing node for OAK-D (Rectification): 
    ```bash
    ROS_NAMESPACE=/tht_rgb_node/color/ rosrun image_proc image_proc
    ```
* Realsense D435i:
```bash 
roslaunch realsense2_camera rs_rgbd.launch color_height:=1080 color_width:=1920 color_fps:=30 publish_tf:=false
```
5. Launch charuco tracker (hand to eye calibration):
* For OAK-D camera: 
```bash
roslaunch easy_aruco track_charuco_board.launch camera_namespace:=/tht_rgb_node/color/ camera_frame:=OAK_camera_rot dictionary:=DICT_6X6_250 square_number_x:=7 square_number_y:=9 square_size:=0.024 marker_size:=0.016
```
* For realsense camera:
```bash
roslaunch easy_aruco track_charuco_board.launch camera_namespace:=/camera/color camera_frame:=OAK_camera_rot dictionary:=DICT_6X6_250 square_number_x:=7 square_number_y:=9 square_size:=0.024 marker_size:=0.016
```
6. Launch easy_handeye calibration and rviz:
```bash 
roslaunch easy_handeye calibrate.launch eye_on_hand:=false freehand_robot_movement:=false robot_effector_frame:=calib_board tracking_base_frame:=OAK_camera_rot tracking_marker_frame:=board publish_dummy:=false start_rviz:=false
```
7. Launch easy_handeye publish calibration:
```bash 
roslaunch easy_handeye publish.launch eye_on_hand:=false tracking_base_frame:=OAK_camera_rot calibration_file:=don't know yet
```
8. Launch aruco tracking for marker target pose:
```bash 
roslaunch easy_aruco track_aruco_marker.launch camera_namespace:=/tht_rgb_node/color camera_frame:=OAK_camera_rot dictionary:=DICT_6X6_250 marker_size:=0.1
```
9. Launch marker flip (for pose planner):
```bash 
roslaunch tht_robot_env marker_flip_y_axis.launch 
```
10. Run calibration move menu:
```bash 
rosrun tht_robot_env move_menu.py
```

## UR10e robot settings
1. Setup connection with physical robot
    * Robot IP-adress: 169.254.1.11
    * Robot Subnet mask: 255.255.0.0
2. Test conntection: `ping 169.254.1.11`

## OAK-D PRO Depth (sterio) Camera
* Camera IP-adress: `169.254.1.222`
