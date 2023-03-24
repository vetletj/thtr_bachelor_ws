cmake_minimum_required(VERSION 3.0.2)
project(thtr_robot_env)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
	scripts/movement_camera_calibration.py
	scripts/move_menu.py
	scripts/OAK_pro_camera_test.py
	scripts/test_aruco.py
	scripts/calib_test.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_thtur10eworkspace.xml)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
