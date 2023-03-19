# thtr_bachelor_ws
Workspace for bachelor at tht robotics

## Full launch sequence for hand-to-eye calibration with OAK-D camera
1. Launch robot description.
* For physical robot:
```bash 
roslaunch thtr_robot_env tht_bachelor_bringup.launch
```
* For simulated robot:
```bash 
roslaunch thtr_robot_env load_rosparam_robot_description.launch
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
roslaunch realsense2_camera rs_rgbd.launch
```
5. Launch charuco tracker:
* For OAK-D camera: 
```bash
roslaunch easy_aruco track_charuco_board.launch camera_namespace:=/tht_rgb_node/color/ camera_frame:=OAK_camera dictionary:=DICT_6X6_250 square_number_x:=7 square_number_y:=9 square_size:=0.024 marker_size:=0.016
```
* For realsense camera:
```bash
roslaunch easy_aruco track_charuco_board.launch camera_namespace:=/camera/color camera_frame:=OAK_camera dictionary:=DICT_6X6_250 square_number_x:=7 square_number_y:=9 square_size:=0.024 marker_size:=0.016
```
6. Launch easy hand to eye calibration and rviz:
```bash 
roslaunch easy_handeye calibrate.launch eye_on_hand:=false freehand_robot_movement:=false publish_dummy:=false start_rviz:=true
```



## How to launch tht robotics robot description with physical robot
1. Catkin_make and source tht_bachelor_ws
2. Run each command below in a new terminal, sourcing the workspace each time (should happen automatic from .bashrc).
    * Terminal 1: 
        ```bash
        roslaunch thtr_robot_env tht_bachelor_bringup.launch
        ```
    * Terminal 2: 
        ```bash
        roslaunch ur10e_moveit_config moveit_planning_execution.launch
        ```
    * Terminal 3: 
        ```bash
        rviz
        ```
3. Setup computer ip-adress (host IP) on URCaps and start stat program. 
4. Go into RViz window
    * Change "Fixed Frame" to "Base_link"
    * Add "Motion planning"

## Simulating UR10e Robot in ROS Gazebo (no enviorment)
1. Source the workspace where ur drivers and moveit are installed
2. Run each command below in a new terminal, sourcing the workspace each time.
    * Terminal 1: 
        ```bash
        roslaunch ur_gazebo ur10e_bringup.launch
        ```
    * Terminal 2: 
        ```bash
        roslaunch ur10e_moveit_config moveit_planning_execution.launch sim:=true
        ```
    * Terminal 3: 
        ```bash
        roslaunch ur10e_moveit_config moveit_rviz.launch config:=true
        ```
3. Go into RViz window
    * Change "Fixed Frame" to "Base_link"
    * Add "Motion planning"

## Simulated robot and enviorment:
1. Launch tht robot description from xacro files: 
```bash
roslaunch thtr_robot_env load_rosparam_robot_description.launch
```
2. Launch fake driver, controller and moveit planner:
```bash
roslaunch ur10e_moveit_config demo.launch load_robot_description:=false
```
3. Go into RViz window
    * Change "Fixed Frame" to "Base_link"
    * Add "Motion planning"

## UR10e robot settings
1. Setup connection with physical robot
    * Robot IP-adress: 169.254.1.11
    * Robot Subnet mask: 255.255.0.0
2. Test conntection: `ping 169.254.1.11`

## DepthAI OAK-D Camera
* Camera IP-adress: `169.254.1.222`
