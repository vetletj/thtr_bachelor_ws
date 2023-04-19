#!/usr/bin/env python
import rospy
from easy_handeye_msgs.srv import TakeSample, ComputeCalibration, SaveCalibration
from easy_handeye.rqt_easy_handeye.src.rqt_easy_handeye.rqt_calibrationmovements import CalibrationMovementsGUI
#from easy_handeye.easy_handeye.src.easy_handeye.handeye_client
def main():
    rospy.init_node("automated_calibration")

    # Wait for the easy_handeye services to become available
    rospy.wait_for_service("/take_sample")
    rospy.wait_for_service("/compute_calibration")
    rospy.wait_for_service("/save_calibration")

    # Create service clients
    take_sample = rospy.ServiceProxy("/take_sample", TakeSample)
    compute_calibration = rospy.ServiceProxy("/compute_calibration", ComputeCalibration)
    save_calibration = rospy.ServiceProxy("/save_calibration", SaveCalibration)

    # Create an instance of the CalibrationMovementsGUI class
    calibration_movements = CalibrationMovementsGUI()

    # Check the starting pose
    calibration_movements.handle_check_current_state()

    while not rospy.is_shutdown():
        # Move to the next pose
        calibration_movements.handle_next_pose()

        # Plan the movement
        calibration_movements.handle_plan()

        # Execute the movement
        calibration_movements.handle_execute()

        rospy.sleep(1)  # Give the camera some time to capture the poses

        # Take a sample at the current pose
        take_sample_response = take_sample()
        if not take_sample_response.success:
            rospy.logwarn("Failed to take sample at pose")

        # Compute the final calibration result
        compute_calibration_response = compute_calibration()
        if not compute_calibration_response.success:
            rospy.logerr("Failed to compute calibration")
        else:
            rospy.loginfo("Calibration result: {}".format(compute_calibration_response))

        # Save the calibration result
        save_calibration_response = save_calibration()
        if not save_calibration_response.success:
            rospy.logerr("Failed to save calibration")
        else:
            rospy.loginfo("Calibration saved successfully")

if __name__ == "__main__":
    main()