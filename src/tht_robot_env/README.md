# tht_robot_env
Package for physical robot environment

# ROS (Noetic) Workspace for Noyce

## Dependencies 
1. ROS Noetic is installed from binaries as below

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```bash
sudo apt install curl 
```

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```bash
sudo apt update
```

```bash
sudo apt install ros-noetic-desktop-full
```

```bash
source /opt/ros/noetic/setup.bash
```

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

```bash
sudo apt install python3-rosdep
```

```bash
sudo rosdep init rosdep update
```

2. Source the setup bash file from the ROS directory: (Note that you have to do this every time you open a new bash terminal window. Alternatively, you can add that line in the .bahsrc file)
```bash
source /opt/ros/noetic/setup.bash
```

3. MoveIt is also installed from binaries:

```bash
sudo apt update
```

```bash
sudo apt dist-upgrade
```

```bash
sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon
```

```bash
sudo apt install python3-wstool
```

```bash
sudo apt install ros-noetic-moveit
```

4. For the Intel RealSense camera, the relevant packages are installed from binaries (this will also install the required librealsense library automatically)
```bash
sudo apt install ros-noetic-realsense2-camera
```

5. For the  luxonis OAK-D camera, relevant package is installed from binaries. Instructions come from here: https://github.com/luxonis/depthai-ros
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
```
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
```bash
sudo apt install ros-noetic-depthai-ros
```

6. The robot driver (for the Universal Robots UR10e) needs to be installed from source. The relevant repo is here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git (the packages are also available in the src directory in the unpack_tomatoes repo ready for building) and the instructions are summarized below:

- You should have a catkin workspace already. If not, create one: (feel free to replace `ur_dance` with whatever name you want)
```bash
mkdir -p ur_dance/src && cd ur_dance
```

- Next, we clone the driver:
```bash
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
```

- and then clone the description: (as of now, we need to choose the melodic branch)
```bash
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot
```

- Before building, we need to install the dependencies:
```bash
sudo apt update -qq
```
```bash
rosdep update
```
```bash
rosdep install --from-paths src --ignore-src
```

- Now we can build the workspace
```bash
catkin_make
```

- Before we can use any of the package tools, we need to activate the workspace (ie: source it)
```bash
source devel/setup.bash
```

## Usage
Instructions on how to run the nodes for accomplishing tasks.
### Quick Instructions: (for real robot and whole workspace in URDF)
If you know what you're doing and just want to run the whole thing, here is what you need (if something doesn't work as intended or not sure about what/how to do, please check [this section](#detailed-instructions) ).

Remember that you always need to have the workspace sourced. If it's not in the bashrc, source it:
```bash
source ur_dance/devel/setup.bash
```

1. ROBOT:
```bash
roslaunch ur_robot_driver tht_noyce_bringup.launch
```

```bash
roslaunch ur10e_moveit_config moveit_planning_execution.launch 
```

```bash
roslaunch plan_move tht_viewrviz.launch
```

### Quick Instructions: (for real robot and only robot in URDF)
If you know what you're doing and just want to run the whole thing, here is what you need (if something doesn't work as intended or not sure about what/how to do, please check [this section](#detailed-instructions) )
1. ROBOT:
```bash
source ur_dance/devel/setup.bash
```

```bash
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=169.254.1.11
```

```bash
roslaunch ur10e_moveit_config moveit_planning_execution.launch 
```

```bash
roslaunch plan_move tht_viewrviz.launch
```

## Detailed Instructions:
1. Source the workspace: first thing is to source the workspace which has to be done in all the bash terminal windows or tabs. Otherwise put it in the bashrc file
```bash
source ur_dance/devel/setup.bash
```
2. Loading the robot
	1. Real Robot:
		1. First you should bring up the robot and for this you have 2 options:
			1. If you want only the robot in URDF description, then type this:
			```bash
			roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=169.254.1.11
			```
			2. If you want the the whole workspace in URDF description, then type this: (please note that the robot IP is added to the launch file and you may need to change/check it with new settings/configurations)
			```bash
			roslaunch ur_robot_driver tht_noyce_bringup.launch
			```
		2. Then you should launch the MoveIt planning and execution pipeline. So, in a second window: 
		```bash
		roslaunch ur10e_moveit_config moveit_planning_execution.launch
		```
	2. Simulated robot:
	```bash
	roslaunch ur10e_moveit_config demo.launch load_robot_description:=true
	```
3. Visualize in rviz:

You can simply run `rviz` but then you will have to add the Motion Planning plugin and change various settings for good visualizations. So, all these settings can be saved in a configuration file and then put as an argument to rviz inside one launch file, named `tht_viewrviz`. Please check the name of package you put that launch file in and save your own rviz configuration file and update the launch file accordingly.  So, in a third window:
```bash
roslaunch plan_move tht_viewrviz.launch
```

