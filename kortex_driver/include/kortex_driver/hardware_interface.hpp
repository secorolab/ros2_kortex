// Copyright 2021, PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author Marq Rasmussen marq.rasmussen@picknik.ai
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-06-15
 *
 */
//----------------------------------------------------------------------
#ifndef KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
#define KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_

#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf/model.h>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <kinova_mediator/mediator.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "kortex_driver/visibility_control.h"

namespace KinovaConstants1 {
  constexpr int ACTUATOR_COUNT = 7;
}  // namespace KinovaConstants1

namespace hardware_interface
{
constexpr char HW_IF_TWIST[] = "twist";
constexpr char HW_IF_FAULT[] = "fault";

}  // namespace hardware_interface

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace k_api = Kinova::Api;

namespace kortex_driver
{
enum class StopStartInterface
{
  NONE,
  STOP_POS_VEL_EFFORT,
  STOP_TWIST,
  STOP_GRIPPER,
  STOP_FAULT_CTRL,
  START_POS_VEL_EFFORT,
  START_TWIST,
  START_GRIPPER,
  START_FAULT_CTRL,
};
class KortexMultiInterfaceHardware : public hardware_interface::SystemInterface
{
public:
  KortexMultiInterfaceHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(KortexMultiInterfaceHardware);

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) final;
  KORTEX_DRIVER_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) final;

  KORTEX_DRIVER_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) final;

private:
  // kinova mediator
  robot_id robot_id_;

  // twist temporary command
  k_api::Base::Twist * k_api_twist_;
  k_api::Base::TwistCommand k_api_twist_command_;

  // To minimize bandwidth we synchronize feedback with the robot only when write() is called
  std::vector<double> arm_commands_positions_;
  std::vector<double> arm_commands_velocities_;
  std::vector<double> arm_commands_efforts_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;

  // twist command interfaces
  std::vector<double> twist_commands_;

  // initialize the mediator
  std::shared_ptr<kinova_mediator> mediator_;

  // Gripper
  k_api::GripperCyclic::MotorCommand * gripper_motor_command_;
  double gripper_command_position_ = 0.0;
  double gripper_command_max_velocity_ = 0.0;
  double gripper_command_max_force_ = 0.0;
  double gripper_position_ = 0.0;
  double gripper_velocity_ = 0.0;
  double gripper_force_command_ = 0.0;
  double gripper_speed_command_ = 0.0;

  rclcpp::Time controller_switch_time_;
  std::atomic<bool> block_write = false;

  // changing active controller on the hardware
  // what controller is running
  bool joint_based_controller_running_;
  bool twist_controller_running_;
  bool gripper_controller_running_;
  bool fault_controller_running_;
  bool joint_position_controller_running_;
  bool joint_velocity_controller_running_;
  bool joint_effort_controller_running_;
  // switching auxiliary vars
  // keeping track of which controller is active so appropriate control mode can be adjusted
  // controller manager sends array of interfaces that should be stopped/started and this is the
  // way to internally collect information on which controller should be stopped and started
  // (different controllers claim different interfaces)
  std::vector<StopStartInterface> stop_modes_;
  std::vector<StopStartInterface> start_modes_;
  // switching auxiliary booleans
  bool stop_joint_based_controller_;
  bool stop_twist_controller_;
  bool stop_gripper_controller_;
  bool stop_fault_controller_;
  bool stop_joint_position_controller_;
  bool stop_joint_velocity_controller_;
  bool stop_joint_effort_controller_;
  bool start_joint_based_controller_;
  bool start_twist_controller_;
  bool start_gripper_controller_;
  bool start_fault_controller_;
  bool start_joint_position_controller_;
  bool start_joint_velocity_controller_;
  bool start_joint_effort_controller_;

  // first pass flag
  bool first_pass_;

  // gripper stuff
  std::string gripper_joint_name_;
  bool use_internal_bus_gripper_comm_;

  // temp variables to use in update loop
  float cmd_degrees_tmp_;
  float cmd_vel_tmp_;
  float cmd_eff_tmp_;
  int num_turns_tmp_ = 0;

  // fault control
  double reset_fault_cmd_;
  double reset_fault_async_success_;
  double in_fault_;
  static constexpr double NO_CMD = std::numeric_limits<double>::quiet_NaN();

  void sendTwistCommand();
  void incrementId();
  void sendJointCommands();
  void prepareCommands();
  void sendGripperCommand(
    k_api::Base::ServoingMode arm_mode, double position, double velocity, double force);

  void readGripperPosition();

  // urdf and KDL
  KDL::Tree kinematic_tree;
  KDL::Chain chain_urdf;
  
  const KDL::Vector gravityVectorLeft;  // [in BL] the negative of this gravity vector is considered as input acceleration for the force controller
  const KDL::Vector gravityVectorRight;  // [in BL] the negative of this gravity vector is considered as input acceleration for the force controller

  // Declarations of the transformation-related variables
  const KDL::Vector BL_x_axis_wrt_GF_left;
  const KDL::Vector BL_y_axis_wrt_GF_left;
  const KDL::Vector BL_z_axis_wrt_GF_left;
  const KDL::Vector BL_position_wrt_GF_left;

  const KDL::Vector BL_x_axis_wrt_GF_right;
  const KDL::Vector BL_y_axis_wrt_GF_right;
  const KDL::Vector BL_z_axis_wrt_GF_right;
  const KDL::Vector BL_position_wrt_GF_right;

  const KDL::Rotation BL_wrt_GF_left; // added as columns
  const KDL::Rotation BL_wrt_GF_right; // added as columns
  
  const KDL::Frame BL_wrt_GF_frame_left;
  const KDL::Frame BL_wrt_GF_frame_right;
  KDL::Frame BL_wrt_GF_frame;

  // end effector Pose
  KDL::Frame measured_endEffPose_BL_arm;
  KDL::Frame measured_endEffPose_GF_arm;
  KDL::FrameVel measured_endEffTwist_BL_arm;
  KDL::FrameVel measured_endEffTwist_GF_arm;

  // Joint variables
  KDL::JntArray jnt_positions;
  KDL::JntArray jnt_velocities;   // has only joint velocities of all joints
  KDL::JntArray jnt_torques_read; // to read from the robot

  KDL::JntArray jnt_torques_cmd;  // to send to the robot
  KDL::JntArrayVel jnt_velocity;  // has both joint position and joint velocity of all joints

  KDL::JntArray jnt_accelerations;
  KDL::JntArray zero_jnt_velocities;
  
  KDL::Wrenches linkWrenches_GF;
  KDL::Wrenches linkWrenches_EE;
  
  
  /* KDL solvers */
  std::shared_ptr<KDL::ChainJntToJacDotSolver> jacobDotSolver;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fkSolverPos;
  std::shared_ptr<KDL::ChainFkSolverVel_recursive> fkSolverVel;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ikSolverAcc;
  std::shared_ptr<KDL::ChainIdSolver_RNE> idSolver;

  // cartesian acceleration
  KDL::Twist xdd;
  KDL::Twist xdd_minus_jd_qd;
  KDL::Twist jd_qd;

  void calculate_joint_torques_RNEA(
    std::shared_ptr<KDL::ChainJntToJacDotSolver> &jacobDotSolver,
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> &ikSolverAcc,
    std::shared_ptr<KDL::ChainIdSolver_RNE> &idSolver,
    KDL::JntArrayVel &jnt_velocity,
    KDL::Twist &jd_qd,
    KDL::Twist &xdd,
    KDL::Twist &xdd_minus_jd_qd,
    KDL::JntArray &jnt_accelerations,
    KDL::JntArray &jnt_positions,
    KDL::JntArray &jnt_velocities,
    KDL::Wrenches &linkWrenches_EE,
    KDL::JntArray &jnt_torques);

  void get_end_effector_pose_and_twist(KDL::JntArrayVel &jnt_velocity,
    const KDL::JntArray &jnt_positions,
    const KDL::JntArray &jnt_velocities,
    KDL::Frame &measured_endEffPose_BL,
    KDL::FrameVel &measured_endEffTwist_BL,
    KDL::Frame &measured_endEffPose_GF,
    KDL::FrameVel &measured_endEffTwist_GF,
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> &fkSolverPos,
    std::shared_ptr<KDL::ChainFkSolverVel_recursive> &fkSolverVel,
    const KDL::Frame &BL_wrt_GF_frame);

  void kinova_feedback(std::shared_ptr<kinova_mediator> kinova_arm_mediator,
    KDL::JntArray &jnt_positions,
    KDL::JntArray &jnt_velocities,
    KDL::JntArray &jnt_torques);

};

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
