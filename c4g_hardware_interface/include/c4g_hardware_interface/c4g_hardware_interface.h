/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   c4g_hardware_interface.h
 * Author:  Giovanni Longobardi, Fabrizio Di Domenico, 
 *          Giovanni Mignone
 * Org.:    UNISA
 * Date:    Sep 05, 2019
 *
 * This class implements all the methods for constructing a C4G
 * hardware abstraction. It is the software representation of the
 * robot.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_C4G_HARDWARE_INTERFACE_C4G_HARDWARE_INTERFACE_H_
#define INCLUDE_C4G_HARDWARE_INTERFACE_C4G_HARDWARE_INTERFACE_H_

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <c4g_open_driver/C4gOpen.hpp>
#include <thread>

namespace c4g_hardware_interface
{

class C4gHardwareInterface : public hardware_interface::RobotHW
{

public:

    static const unsigned short int ARM = 1;
    static const unsigned short int NUMBER_OF_JOINTS = 7;

    C4gHardwareInterface(ros::NodeHandle& nh);
    ~C4gHardwareInterface();

    void init();
    void read();
    void write();

private:

    /**
     * @brief Keeps communication with the C4G controller by sending references and receiving the current positions and velocities of the joints.
     *        
     * Even though a position interface is considered, this function needs 
     * to set the target velocity since the C4g in Mode 4 waits for both the 
     * target position and the target velocity coming from the PC.
     * Refer to C4g System Software documentation, chapter 14.7 for more information.
     */
    void keepAliveLoop_();

    /**
     * @brief Converts the joint positions received by the C4G controller from motor revolutions to radians,
     *        for joints from 1 to 6, and to meters, for joint 7.
     *        More information about conversion can be found in C4g System Software documentation, chapter 19.
     *
     * @param joint_positions_mr are the joint positions in motor revolutions
     * @param joint_positions_rad_m are the joint positions in radians (except for the last joint, the slide, whose value is in meters)
     */
    void convertMotorRevolutionsToRadiansAndMeters_(const std::vector<double> &joint_positions_mr, std::vector<double> &joint_positions_rad_m);

    /**
     * @brief Converts the joint positions from radians, for joints from 1 to 6, and from meters, for joint 7, to motor revolutions.
     *        More information about conversion can be found in C4g System Software documentation, chapter 19.
     * 
     * @param joint_positions_rad_m are the joint positions in radians (except for the last joint, the slide, whose value is in meters)
     * @param joint_positions_mr are the joint positions in motor revolutions
     */
    void convertRadiansAndMetersToMotorRevolutions_(const std::vector<double> &joint_positions_rad_m, std::vector<double> &joint_positions_mr);

    /**
     * @brief Copy the content of synchronized vector to standard vector
     * 
     * @param shared_vector_in is a vector of atomic double where values are copied from
     * @param vector_out is a vector of double where values are copied to
     */
    void readSharedVector_(const std::vector<std::atomic<double>> & shared_vector_in, std::vector<double> & vector_out) const;

    /**
     * @brief Copy the content of standard vector to synchronized vector
     * 
     * @param vector_in is a vector of double where values are copied from
     * @param shared_vector_out is a vector of atomic double where values are copied to
     */
    void writeSharedVector_(const std::vector<double> & vector_in, std::vector<std::atomic<double>> & shared_vector_out) const;

    // C4gOpen
    bool keep_going_ ;
    std::thread keep_alive_thread_;
    ros::NodeHandle nh_;

    // Kinematic influence coefficients
    float k54_;
    float k65_;
    float k64_;

    // Transmission rates
    std::vector<float> kr_;

    // Calibration constants
    std::vector<float> calibration_constants_;

    // Internal interfaces between main control loop (called from the outside) and keep-alive thread
    std::vector<std::atomic<double>> c4g_joint_position_state_;
    std::vector<std::atomic<double>> c4g_joint_position_command_;

    // External interfaces towards ROS controllers
    // Data are not synchronized because read, update and write are called in sequence
    hardware_interface::JointStateInterface joint_state_interface_;
    std::vector<double> joint_position_state_;
    std::vector<double> joint_velocity_state_;
    std::vector<double> joint_effort_state_;

    hardware_interface::PositionJointInterface joint_position_command_interface_;
    std::vector<double> joint_position_command_;
};

} // namespace c4g_hardware_interface

#endif /* INCLUDE_C4G_HARDWARE_INTERFACE_C4G_HARDWARE_INTERFACE_H_ */
