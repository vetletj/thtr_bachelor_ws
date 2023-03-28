cmake_minimum_required(VERSION 3.0.2)
project(tht_robot_env)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
	scripts/movement_camera_calibration.py
	scripts/move_menu.py
	scripts/calib_test.py
	scripts/print_tf_pose.py
	scripts/ur10e_test_hand_to_eye_calib.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_thtur10eworkspace.xml)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
