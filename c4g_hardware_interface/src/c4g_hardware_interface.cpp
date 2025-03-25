/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   c4g_hardware_interface.cpp
 * Author:  Giovanni Longobardi, Fabrizio Di Domenico,
 *          Giovanni Mignone
 * Org.:    UNISA
 * Date:    Sep 05, 2019
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <c4g_hardware_interface/c4g_hardware_interface.h>
#include <c4g_hardware_interface/exceptions.h>
#include <ros_extensions_utilities/ros_extensions_utils.h>

using namespace c4g_hardware_interface;

C4gHardwareInterface::C4gHardwareInterface(ros::NodeHandle& nh)
  : nh_(nh)
  , c4g_joint_position_state_(NUMBER_OF_JOINTS)
  , c4g_joint_position_command_(NUMBER_OF_JOINTS)
  , kr_(NUMBER_OF_JOINTS)
  , calibration_constants_(NUMBER_OF_JOINTS)
  , joint_position_state_(NUMBER_OF_JOINTS)
  , joint_velocity_state_(NUMBER_OF_JOINTS)
  , joint_effort_state_(NUMBER_OF_JOINTS)
  , joint_position_command_(NUMBER_OF_JOINTS)
{
}

C4gHardwareInterface::~C4gHardwareInterface()
{
}

void C4gHardwareInterface::init()
{
  std::vector<std::string> joint_names =
      ros::getParameterFromParamServer<std::vector<std::string>>(nh_, "hardware_interface/joints");

  if (joint_names.size() != NUMBER_OF_JOINTS)
  {
    throw c4g_hardware_interface::Exception("The number of joints in the configuration is not equal to 7");
  }

  // Register handles
  for (std::size_t i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // State handle
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joint_position_state_[i],
                                                            &joint_velocity_state_[i], &joint_effort_state_[i]);
    joint_state_interface_.registerHandle(joint_state_handle);

    // Position handle
    hardware_interface::JointHandle joint_position_handle(joint_state_handle, &joint_position_command_[i]);
    joint_position_command_interface_.registerHandle(joint_position_handle);
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_position_command_interface_);

  // Start keepAliveThread
  keep_alive_thread_ = std::thread(&C4gHardwareInterface::keepAliveLoop_, this);
}

void C4gHardwareInterface::keepAliveLoop_()
{
  ROS_INFO("Now you can boot the C4G Controller");

  int32_t port_number = ros::getParameterFromParamServer<int32_t>(nh_, "hardware_interface/c4g_port_number");

  C4gOpen c4g_open(port_number);

  if (c4g_open.start())
  {
    // Copies local to this thread in order to avoid race conditions
    std::vector<double> joint_position_state_mr(NUMBER_OF_JOINTS);
    std::vector<double> joint_position_state_rad_m(NUMBER_OF_JOINTS);
    std::vector<double> joint_position_command_mr(NUMBER_OF_JOINTS);
    std::vector<double> joint_position_command_rad_m(NUMBER_OF_JOINTS);
    std::vector<double> joint_delta_position_command_mr(NUMBER_OF_JOINTS, 0.0);

    ROS_INFO("Drive On the robot and set mode 4 via PDL2");

    ROS_INFO("Wait for mode 4");

    while (!c4g_open.waitForOpenMode4(ARM))
      ;

    ROS_INFO("Mode 4 active. Ready to receive commands.");

    k54_ = c4g_open.getKinInflCoeff54(ARM);
    k65_ = c4g_open.getKinInflCoeff65(ARM);
    k64_ = c4g_open.getKinInflCoeff64(ARM);

    for (std::size_t i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      joint_position_state_mr[i] = c4g_open.getActualPosition(ARM, i + 1);
      kr_[i] = c4g_open.getTxRate(ARM, i + 1);
      calibration_constants_[i] = c4g_open.getCalibrationConstant(ARM, i + 1);
    }

    convertMotorRevolutionsToRadiansAndMeters_(joint_position_state_mr, joint_position_state_rad_m);

    // Initialize internal state and command variable with current robot state
    writeSharedVector_(joint_position_state_rad_m, c4g_joint_position_state_);
    writeSharedVector_(joint_position_state_rad_m, c4g_joint_position_command_);

    // Initialize position handle to the current robot state, i.e. command current state
    readSharedVector_(c4g_joint_position_command_, joint_position_command_);

    keep_going_ = true;

    while (keep_going_)
    {
      if (c4g_open.receive())
      {
        // Copy command vector from shared to local variable
        readSharedVector_(c4g_joint_position_command_, joint_position_command_rad_m);

        // Write command to the driver
        convertRadiansAndMetersToMotorRevolutions_(joint_position_command_rad_m, joint_position_command_mr);

        for (std::size_t i = 0; i < NUMBER_OF_JOINTS; i++)
        {
          // For the time being, the delta position is computed wrt the state,
          // but better performance might be achieved if this is computed wrt the
          // previous command, or even better, if it is provided by the planner.
          joint_delta_position_command_mr[i] = joint_position_command_mr[i] - joint_position_state_mr[i];

          c4g_open.setTargetPosition(ARM, i + 1, joint_position_command_mr[i]);
          c4g_open.setTargetVelocity(ARM, i + 1, joint_delta_position_command_mr[i]);
        }

        // Read actual state from the driver
        for (std::size_t i = 0; i < NUMBER_OF_JOINTS; i++)
        {
          joint_position_state_mr[i] = c4g_open.getActualPosition(ARM, i + 1);
        }

        convertMotorRevolutionsToRadiansAndMeters_(joint_position_state_mr, joint_position_state_rad_m);

        // Copy state vector from local to shared variable
        writeSharedVector_(joint_position_state_rad_m, c4g_joint_position_state_);

        if (!c4g_open.send())
          keep_going_ = false;
      }

      if (c4g_open.errorOccurred() || c4g_open.getMode(ARM) == C4G_OPEN_EXIT)
        keep_going_ = false;
    }

    c4g_open.stop();

    ROS_INFO("Communication with C4G interrupted");
  }

  ROS_INFO("Communication thread ended");
}

void C4gHardwareInterface::read()
{
  // Move state data from internal to external interface (handle)
  readSharedVector_(c4g_joint_position_state_, joint_position_state_);
}

void C4gHardwareInterface::write()
{
  // Move command data from external (handle) to internal interface
  writeSharedVector_(joint_position_command_, c4g_joint_position_command_);
}

void C4gHardwareInterface::convertMotorRevolutionsToRadiansAndMeters_(const std::vector<double>& joint_positions_mr,
                                                                      std::vector<double>& joint_positions_rad_m)
{
  for (std::size_t i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    if (i < 4)
      joint_positions_rad_m[i] = (joint_positions_mr[i] - calibration_constants_[i]) * 2 * M_PI / kr_[i];
    else if (i == 4)
      joint_positions_rad_m[i] = (joint_positions_mr[i] + kr_[i] * k54_ * joint_positions_rad_m[i - 1] / (2 * M_PI) -
                                  calibration_constants_[i]) *
                                 2 * M_PI / kr_[i];
    else if (i == 5)
      joint_positions_rad_m[i] =
          (joint_positions_mr[i] + kr_[i] * k65_ * joint_positions_rad_m[i - 1] / (2 * M_PI) -
           kr_[i] * (k65_ * k54_ - k64_) * joint_positions_rad_m[i - 2] / (2 * M_PI) - calibration_constants_[i]) *
          2 * M_PI / kr_[i];
    else if (i == 6)
      joint_positions_rad_m[i] = (joint_positions_mr[i] * kr_[i] - calibration_constants_[i]) / 1000.0;
  }
}

void C4gHardwareInterface::convertRadiansAndMetersToMotorRevolutions_(const std::vector<double>& joint_positions_rad_m,
                                                                      std::vector<double>& joint_positions_mr)
{
  for (std::size_t i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    if (i < 4)
      joint_positions_mr[i] = kr_[i] * joint_positions_rad_m[i] / (2 * M_PI) + calibration_constants_[i];

    else if (i == 4)
      joint_positions_mr[i] = kr_[i] * (joint_positions_rad_m[i] - k54_ * joint_positions_rad_m[i - 1]) / (2 * M_PI) +
                              calibration_constants_[i];
    else if (i == 5)
      joint_positions_mr[i] = kr_[i] *
                                  (joint_positions_rad_m[i] - k65_ * joint_positions_rad_m[i - 1] +
                                   (k65_ * k54_ - k64_) * joint_positions_rad_m[i - 2]) /
                                  (2 * M_PI) +
                              calibration_constants_[i];
    else if (i == 6)
      joint_positions_mr[i] = (joint_positions_rad_m[i] * 1000.0 + calibration_constants_[i]) / kr_[i];
  }
}

void C4gHardwareInterface::readSharedVector_(const std::vector<std::atomic<double>>& shared_vector_in,
                                             std::vector<double>& vector_out) const
{
  for (std::size_t i = 0; i < NUMBER_OF_JOINTS; i++)
    vector_out[i] = shared_vector_in[i];
}

void C4gHardwareInterface::writeSharedVector_(const std::vector<double>& vector_in,
                                              std::vector<std::atomic<double>>& shared_vector_out) const
{
  for (std::size_t i = 0; i < NUMBER_OF_JOINTS; i++)
    shared_vector_out[i] = vector_in[i];
}
