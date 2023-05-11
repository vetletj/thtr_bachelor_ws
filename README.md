# tht_bachelor_ws
Workspace for bachelor assignment at tht robotics

## Installation Instructions and setup of workspace
   ### Step 1: Install the ROS distribution
   - #### Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on Ubuntu 20.04.
   #### Step 2: Clone bachelor repo   
```bash 
git clone -b vetle_dev https://github.com/vetletj/thtr_bachelor_ws.git

# Go to workspace
cd thtr_bachelor_ws
```
   #### Step 3: Clone UR Robot ROS Driver
```bash 
# clone the driver
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone the description. Currently, it is necessary to use the melodic-devel branch.
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot
```
   #### Step 4: Clone Intel RealSense Camera Driver
```bash
git clone https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros/

cd src/realsense-ros/

git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

cd ../..
```
   #### Step 5: Clone Image Pipeline
```bash
git clone -b noetic https://github.com/vetletj/image_pipeline.git src/image_pipeline/
```
   #### Step 6: Clone Easy Aruco Package
```bash
# install script dependencies
pip3 install --user fpdf

# check out the repository, satisfy the dependencies and build
git clone https://github.com/marcoesposito1988/easy_aruco src/easy_aruco/

rosdep install -yir --from-paths src
```
   #### Step 6: Clone Easy Handeye Package
```bash
git clone https://github.com/IFL-CAMP/easy_handeye src/easy_handeye

# now we are inside ~/thtr_bachelor_ws
rosdep install -iyr --from-paths src
```
   #### Step 7: Build workspace
   ```bash
  catkin_make clean
  catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
  catkin_make install
  ```

  ```bash
  echo "source ~/thtr_bachelor_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

## Launch sequences for UR3 
### hand-to-eye calibration
1. Launch robot description.
* Physical robot:
```bash 
roslaunch tht_bachelor_ur_launch ur3_bringup.launch end_effector_type:=1
```
* Simulated robot:
```bash 
roslaunch tht_bachelor_ur_launch ur3_bringup.launch
```
2. Launch MoveIt planner:
* Physical robot:
```bash 
roslaunch ur3_moveit_config moveit_planning_execution.launch
```
* Simulated robot:
```bash 
roslaunch ur3_moveit_config demo.launch load_robot_description:=false
```
3. Launch camera.
* Realsense
```bash 
roslaunch realsense2_camera rs_rgbd.launch color_height:=1080 color_width:=1920 color_fps:=30 publish_tf:=false
```
* OAK-D PRO
```bash 
roslaunch tht_depthai tht_rgb_publisher.launch
```
```bash 
ROS_NAMESPACE=/tht_rgb_node/color/ rosrun image_proc image_proc
```
4. Launch charuco tracker:
* Realsense
```bash 
roslaunch easy_aruco track_charuco_board.launch camera_namespace:=/camera/color/ camera_frame:=OAK_camera dictionary:=DICT_6X6_250 square_number_x:=7 square_number_y:=9 square_size:=0.024 marker_size:=0.016
```
* OAK-D Pro
```bash 
roslaunch easy_aruco track_charuco_board.launch camera_namespace:=/tht_rgb_node/color/ camera_frame:=OAK_camera dictionary:=DICT_6X6_250 square_number_x:=7 square_number_y:=9 square_size:=0.024 marker_size:=0.016
```
5. Launch rviz:
```bash 
roslaunch tht_bachelor_ur_launch load_rviz_bachelor.launch
```
6. Launch hand-to-eye calibration:
```bash 
roslaunch easy_handeye calibrate.launch eye_on_hand:=false freehand_robot_movement:=true robot_effector_frame:=end_effector_1 tracking_base_frame:=OAK_camera tracking_marker_frame:=board publish_dummy:=false start_rviz:=false
```
7. Start move menu with for calibration board (end_effector_1):
```bash 
rosrun tht_bachelor_ur_launch move_menu_1.py
```
```bash 
rosrun tht_bachelor_ur_launch calibration_roll_pitch_yaw.py
```
### FAST hand-to-eye calibration test
1. Launch easy_handeye publish calibration:
```bash 
roslaunch easy_handeye publish.launch eye_on_hand:=false robot_effector_frame:=end_effector_1 robot_base_frame:=world tracking_base_frame:=OAK_camera calibration_file:=/home/thtr/.ros/easy_handeye/21_TsaiLenz.yaml
```
2.

### test hand-to-eye calibration
1. Update /urdf/inc/cameras/cam_info.yaml with calibration data.

2. Launch robot description.
```bash 
roslaunch tht_bachelor_ur_launch ur3_bringup.launch end_effector_type:=2
```
3. Launch MoveIt planner:
```bash 
roslaunch ur3_moveit_config moveit_planning_execution.launch
```
4. Launch camera.
* Intel Realsense
```bash 
roslaunch realsense2_camera rs_rgbd.launch color_height:=1080 color_width:=1920 color_fps:=30 publish_tf:=false
```
* OAK-D PRO
```bash 
roslaunch tht_depthai tht_rgb_publisher.launch
```
```bash 
ROS_NAMESPACE=/tht_rgb_node/color/ rosrun image_proc image_proc
```
5. Launch aruco tracker:
* Intel Realsense
```bash 
roslaunch easy_aruco track_aruco_marker.launch camera_namespace:=/camera/color camera_frame:=OAK_camera dictionary:=DICT_6X6_250 marker_size:=0.1
```
* OAK-D PRO
```bash 
roslaunch easy_aruco track_aruco_marker.launch camera_namespace:=/tht_rgb_node/color camera_frame:=OAK_camera dictionary:=DICT_6X6_250 marker_size:=0.1
```
5. Launch marker flip:
```bash 
roslaunch tht_bachelor_ur_launch marker_flip_y_axis.launch 
```
5. Launch rviz:
```bash 
roslaunch tht_bachelor_ur_launch load_rviz_bachelor.launch
```
7. Start move menu with for pointer tool (end_effector_2):
```bash 
rosrun tht_bachelor_ur_launch move_menu_2.py
```


## Full launch sequence for hand-to-eye calibration with OAK-D camera
1. Launch robot description.
* For physical robot:
```bash 
roslaunch tht_robot_env tht_bachelor_bringup.launch
```
* For simulated robot:
```bash 
roslaunch tht_robot_env load_rosparam_robot_description.launch end_effector_type:=1
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
* For physical robot
```bash 
roslaunch easy_handeye calibrate.launch eye_on_hand:=false freehand_robot_movement:=false robot_effector_frame:=end_effector_1 tracking_base_frame:=OAK_camera_rot tracking_marker_frame:=board publish_dummy:=false start_rviz:=false
```
* For simulated enviorment
```bash 
roslaunch easy_handeye calibrate.launch eye_on_hand:=false freehand_robot_movement:=false robot_base_frame:=world robot_effector_frame:=end_effector_1 tracking_base_frame:=OAK_camera_rot tracking_marker_frame:=target_marker publish_dummy:=false start_rviz:=false
```
7. Launch easy_handeye publish calibration:
```bash 
roslaunch easy_handeye publish.launch eye_on_hand:=false robot_effector_frame:=end_effector_1 robot_base_frame:=world tracking_base_frame:=OAK_camera_rot calibration_file:=/home/thtstation1/.ros/easy_handeye/easy_handeye_eye_on_base.yam
```
8. Launch aruco tracking for marker target pose:
* For physical enviorment
```bash 
roslaunch easy_aruco track_aruco_marker.launch camera_namespace:=/tht_rgb_node/color camera_frame:=OAK_camera_rot dictionary:=DICT_6X6_250 marker_size:=0.1
```
* For simulated enviorment
```bash 
roslaunch tht_robot_env tf_pub.launch x:=-0.2 y:=0.1 z:=0.4 qw:=1 parent_frame_id:=OAK_camera child_frame_id:=target_marker
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
