# thtr_bachelor_ws
Workspace for bachelor at tht robotics

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
