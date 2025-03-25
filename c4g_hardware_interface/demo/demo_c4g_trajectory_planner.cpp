/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   demo_c4g_trajectory_planner.cpp
 * Author:  Giovanni Longobardi, Giovanni Mignone
 * Org.:    UNISA
 * Date:    Dec 18, 2020
 *
 * This node implements a trajectory planner in order to communicate
 * with the JointGroupPositionController. It generates a sinusoidal
 * movement on all joints centered around the current joint positions
 * read from the joint state controller. The parameters of the
 * sinusoid are
 *
 * Amplitude: 5 degrees for joints from 1 to 6;
 *            5 centimeters for the last joint (the slide);
 * Frequency: 0.25 Hz for all the joints.
 *
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <c4g_hardware_interface/c4g_hardware_interface.h>

const unsigned short int number_of_joints = c4g_hardware_interface::C4gHardwareInterface::NUMBER_OF_JOINTS;

std::vector<double> initial_positions;
std::atomic<bool> robot_state_received(false);

void robotStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  initial_positions.resize(number_of_joints);

  // In this demo it is assumed that the values read by the
  // joint_state_controller are in the order of the kinematic chain.
  // In general this is not always true, the joint_state_controller
  // stores them in alphabetical order.
  for (int32_t i = 0; i < number_of_joints; i++)
    initial_positions[i] = msg->position[i];

  robot_state_received = true;

  ROS_INFO("Robot state received");
}

int32_t main(int32_t argc, char **argv)
{
  // Initializing the node
  ros::init(argc, argv, "demo_c4g_trajectory_planner");
  ros::NodeHandle nodeh;

  std::string node_namespace;
  nodeh.getParam("/namespace", node_namespace);

  // Create a NodeHandle object with the right namespace
  ros::NodeHandle nh(node_namespace);

  // Get the hardware control loop frequency from the parameter server
  double loop_hz;
  nh.getParam("hardware_control_loop/loop_hz", loop_hz);

  float sample_time = 1 / loop_hz;

  // Define the parameters of the sinusoidal movement
  double frequency = 0.25;
  double amplitude = 5;

  // Waiting for the robot initial state
  ros::Subscriber robot_initial_state_subscriber;
  robot_initial_state_subscriber = nh.subscribe("joint_states", 1, robotStateCallback);

  while (robot_state_received == false)
  {
    ros::spinOnce();
    sleep(1);
  }

  robot_initial_state_subscriber.shutdown();

  ros::Publisher c4g_trajectory_planner = nh.advertise<std_msgs::Float64MultiArray>("pos_joint_group_controller/command", 10);

  // Give some time to the subscriber to recognize the publshing of commands
  ros::Duration pub_sub_connection_duration(1.0);
  pub_sub_connection_duration.sleep();

  double rad_amplitude = amplitude * M_PI / 180;
  double m_amplitude = amplitude / 100.0;
  int32_t counter = 0;

  // The trajectory planner starts sending references
  ros::Rate loop_rate(loop_hz);
  std_msgs::Float64MultiArray joint_group_position;
  joint_group_position.data.resize(number_of_joints);

  while (ros::ok())
  {

    double omegat = 2 * M_PI * frequency * ((double)counter * sample_time);

    for (int32_t i = 0; i < number_of_joints; i++)
    {
      if (i == 6)
        joint_group_position.data[i] = initial_positions[i] + (float)(m_amplitude * sin(omegat));
      else
        joint_group_position.data[i] = initial_positions[i] + (float)(rad_amplitude * sin(omegat));
    }

    c4g_trajectory_planner.publish(joint_group_position);

    counter++;

    loop_rate.sleep();
  }

  ros::shutdown();
}