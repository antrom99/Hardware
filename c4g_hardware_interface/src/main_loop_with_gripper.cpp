/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   main_loop_with_gripper.cpp
 * Author:  Rosanna Coccaro, Bruno Lima
 * Org.:    UNISA
 * Date:    Feb 01, 2023
 *
 * This node implements the generic control loop feedback mechanism to
 * control the Comau Smart-Six robot along with a Gripper, in the same
 * loop. It follows the standard model of ros_control.
 *
 * -------------------------------------------------------------------
 */

#include <c4g_hardware_interface/c4g_hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <gripper_hardware_interface/gripper_hardware_interface.h>

int32_t main(int32_t argc, char **argv)
{
    // Initializing the node
    ros::init(argc, argv, "c4g_main_control_loop_node");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle smartsix_nh;
    ros::NodeHandle gripper_nh("gripper");

    // Create the object of the robot hardware_interface class
    c4g_hardware_interface::C4gHardwareInterface smartsix_hw(smartsix_nh);
    gripper_hardware_interface::GripperHardwareInterface gripper_hw(gripper_nh);

    smartsix_hw.init();
    gripper_hw.init();

    double loop_hz;
    smartsix_nh.getParam("/hardware_control_loop/loop_hz", loop_hz);

    controller_manager::ControllerManager smartsix_cm(&smartsix_hw, smartsix_nh);
    controller_manager::ControllerManager gripper_cm(&gripper_hw, gripper_nh);

    // Set the frequency of the control loop
    ros::Rate r(loop_hz);

    // Run the control loop
    ros::Time last_update = ros::Time::now();

    while (ros::ok())
    {
        smartsix_hw.read();
        gripper_hw.read();

        ros::Time current_time = ros::Time::now();

        smartsix_cm.update(current_time, current_time - last_update);
        gripper_cm.update(current_time, current_time - last_update);

        last_update = current_time;

        smartsix_hw.write();
        gripper_hw.write();

        r.sleep();
    }

    spinner.stop();

    ros::shutdown();
}
