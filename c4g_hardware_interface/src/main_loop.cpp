/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   main_loop.cpp
 * Author:  Giovanni Longobardi, Fabrizio Di Domenico,
 *          Giovanni Mignone
 * Org.:    UNISA
 * Date:    Sep 05, 2019
 *
 * This node implements the generic control loop feedback mechanism to
 * control the Comau Smart-Six robot. It follows the standard model of
 * ros_control.
 *
 * -------------------------------------------------------------------
 */

#include <c4g_hardware_interface/c4g_hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros_extensions_utilities/ros_extensions_utils.h>

int32_t main(int32_t argc, char** argv)
{
  // Initializing the node
  ros::init(argc, argv, "c4g_main_control_loop_node");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;

  // Create the object of the robot hardware_interface class
  c4g_hardware_interface::C4gHardwareInterface hw(nh);
  hw.init();

  // Create the controller manager object
  controller_manager::ControllerManager cm(&hw, nh);

  // Set the frequency of the control loop
  ros::Rate r(ros::getParameterFromParamServer<double>(nh, "hardware_control_loop/loop_hz"));

  // Run the control loop
  ros::Time last_update = ros::Time::now();

  while (ros::ok())
  {
    hw.read();

    ros::Time current_time = ros::Time::now();

    cm.update(current_time, current_time - last_update);

    last_update = current_time;

    hw.write();

    r.sleep();
  }

  spinner.stop();

  ros::shutdown();
}
