# thtr_bachelor_ws
Workspace for bachelor at tht robotics

## How to launch tht robotics robot enviorment at office

`$ roslaunch thtr_robot_env tht_view_ur10eworkspace.launch`


## Simulating UR10e Robot in ROS Gazebo (no enviorment)
1. Source the workspace where ur drivers and moveit are installed
2. Run each command below in a new terminal, sourcing the workspace each time.
    * Terminal 1: `roslaunch ur_gazebo ur10e_bringup.launch`
    * Terminal 2: `roslaunch ur10e_moveit_config moveit_planning_execution.launch sim:=true`
    * Terminal 3: `roslaunch ur10e_moveit_config moveit_rviz.launch config:=true`
3. Go into RViz window
    * Change "Fixed Frame" to "Base_link"
    * Add "Motion planning"

## Simulated robot and enviorment:
1. Launch tht robot description from xacro files: 
```bash
roslaunch thtr_robot_env load_rosparam_robot_description.launch
```
2. Launch fake driver, controller and moveit planner: ```roslaunch ur10e_moveit_config demo.launch load_robot_description:=false```

## Physical UR10e robot in ROS
1. Setup connection with physical robot
    * Robot IP-adress: 169.254.1.11
    * Robot Subnet mask: 255.255.0.0
2. Test conntection: `ping 169.254.1.11`
3. 
3. Run each command below in a new terminal, sourcing the workspace each time.
    * Terminal 1: `roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=169.254.1.11`
    * Terminal 2: `roslaunch ur10e_moveit_config moveit_planning_execution.launch`
    * Terminal 3: `roslaunch thtr_robot_env tht_view_ur10eworkspace.launch`


