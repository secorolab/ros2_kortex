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

#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "kortex_driver/hardware_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexMultiInterfaceHardware");
}

namespace kortex_driver
{
KortexMultiInterfaceHardware::KortexMultiInterfaceHardware()
:
  k_api_twist_(nullptr),
  gripper_motor_command_(nullptr),
  gripper_command_max_velocity_(100.0),
  gripper_command_max_force_(100.0),
  joint_based_controller_running_(false),
  joint_position_controller_running_(false),
  joint_velocity_controller_running_(false),
  joint_effort_controller_running_(false),
  twist_controller_running_(false),
  gripper_controller_running_(false),
  fault_controller_running_(false),
  stop_joint_based_controller_(false),
  stop_twist_controller_(false),
  stop_gripper_controller_(false),
  stop_fault_controller_(false),
  stop_joint_position_controller_(false),
  stop_joint_velocity_controller_(false),
  stop_joint_effort_controller_(false),
  start_joint_position_controller_(false),
  start_joint_velocity_controller_(false),
  start_joint_effort_controller_(false),
  start_joint_based_controller_(false),
  start_twist_controller_(false),
  start_gripper_controller_(false),
  start_fault_controller_(false),
  first_pass_(true),
  gripper_joint_name_(""),
  use_internal_bus_gripper_comm_(false),
  gravityVectorLeft(-9.6, 1.16, 1.3),
  gravityVectorRight(-9.5, -1.0, 1.1),
  BL_x_axis_wrt_GF_left(0.0631, 0.064, 0.9959),
  BL_y_axis_wrt_GF_left(0.7122, -0.702, 0.0),
  BL_z_axis_wrt_GF_left(0.6992, 0.7092, -0.09),
  BL_position_wrt_GF_left(0., 0.08, 0.0),
  BL_x_axis_wrt_GF_right(0.0631, -0.064, 0.9959),
  BL_y_axis_wrt_GF_right(-0.7122, -0.702, 0.0),
  BL_z_axis_wrt_GF_right(0.6992, -0.7092, -0.09),
  BL_position_wrt_GF_right(0.0, -0.08, 0.0),
  BL_wrt_GF_right(BL_x_axis_wrt_GF_right, BL_y_axis_wrt_GF_right, BL_z_axis_wrt_GF_right),
  BL_wrt_GF_left(BL_x_axis_wrt_GF_left, BL_y_axis_wrt_GF_left, BL_z_axis_wrt_GF_left),
  BL_wrt_GF_frame_left(BL_wrt_GF_left, BL_position_wrt_GF_left),
  BL_wrt_GF_frame_right(BL_wrt_GF_right, BL_position_wrt_GF_right),
  jnt_positions(kinova_mediator_constants::ACTUATOR_COUNT),
  jnt_velocities(kinova_mediator_constants::ACTUATOR_COUNT),
  jnt_torques_read(kinova_mediator_constants::ACTUATOR_COUNT),
  jnt_torques_cmd(kinova_mediator_constants::ACTUATOR_COUNT),
  jnt_velocity(kinova_mediator_constants::ACTUATOR_COUNT),
  jnt_accelerations(kinova_mediator_constants::ACTUATOR_COUNT),
  zero_jnt_velocities(kinova_mediator_constants::ACTUATOR_COUNT),
  linkWrenches_GF(kinova_mediator_constants::SEGMENT_COUNT_FULL, KDL::Wrench::Zero()),
  linkWrenches_EE(kinova_mediator_constants::SEGMENT_COUNT_FULL, KDL::Wrench::Zero())
{
  mediator_ = std::make_shared<kinova_mediator>();

  std::string package_path = ament_index_cpp::get_package_share_directory("kortex_driver");
  kdl_parser::treeFromFile(package_path+"/urdf/Kinova_1.urdf", kinematic_tree);
  kinematic_tree.getChain("base_link", "EndEffector_Link", chain_urdf);

  jacobDotSolver = std::make_shared<KDL::ChainJntToJacDotSolver>(chain_urdf);
  fkSolverPos = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_urdf);
  fkSolverVel = std::make_shared<KDL::ChainFkSolverVel_recursive>(chain_urdf);
  ikSolverAcc = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_urdf);

  RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
  auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

CallbackReturn KortexMultiInterfaceHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = info;
  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  if (robot_ip.empty())
  {
    RCLCPP_ERROR(LOGGER, "Robot ip is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Robot ip is '%s'", robot_ip.c_str());
    if (robot_ip == kinova_mediator_constants::IP_ADDRESS_1)
    {
      robot_id_ = robot_id::KINOVA_GEN3_1;
    }
    else if (robot_ip == kinova_mediator_constants::IP_ADDRESS_2)
    {
      robot_id_ = robot_id::KINOVA_GEN3_2;
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Unknown robot ip!");
      return CallbackReturn::ERROR;
    }
  }
  // Username to log into the robot controller
  std::string username = info_.hardware_parameters["username"];
  if (username.empty())
  {
    RCLCPP_ERROR(LOGGER, "Username is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Username is '%s'", username.c_str());
  }
  // Password to log into the robot controller
  std::string password = info_.hardware_parameters["password"];
  if (password.empty())
  {
    RCLCPP_ERROR(LOGGER, "Password is empty!");
    return CallbackReturn::ERROR;
  }
  int port = std::stoi(info_.hardware_parameters["port"]);
  if (port <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Port used '%d'", port);
  }
  int port_realtime = std::stoi(info_.hardware_parameters["port_realtime"]);
  if (port_realtime <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect realtime port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Realtime port used '%d'", port_realtime);
  }

  int session_inactivity_timeout =
    std::stoi(info_.hardware_parameters["session_inactivity_timeout_ms"]);
  if (session_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect session inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Session inactivity timeout is '%d'", session_inactivity_timeout);
  }
  int connection_inactivity_timeout =
    std::stoi(info_.hardware_parameters["connection_inactivity_timeout_ms"]);
  if (connection_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect connection inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Connection inactivity timeout is '%d'", connection_inactivity_timeout);
  }
  // gripper joint name
  gripper_joint_name_ = info_.hardware_parameters["gripper_joint_name"];
  if (gripper_joint_name_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Gripper joint name is empty!");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper joint name is '%s'", gripper_joint_name_.c_str());
  }

  gripper_command_max_velocity_ = std::stod(info_.hardware_parameters["gripper_max_velocity"]);
  gripper_command_max_force_ = std::stod(info_.hardware_parameters["gripper_max_force"]);

  RCLCPP_INFO_STREAM(LOGGER, "Connecting to robot at " << robot_ip);

  // Initialize the mediator
  mediator_->kinova_id = robot_id_;
  mediator_->initialize(kinova_environment::REAL, robot_id_, username, password, 
    session_inactivity_timeout, connection_inactivity_timeout, port, port_realtime);

  RCLCPP_INFO(LOGGER, "Kinova Mediator initialized, setting base and base cyclic");
  auto base_ = mediator_->get_base();
  auto base_cyclic_ = mediator_->get_base_cyclic();
  auto& base_command_ = mediator_->get_base_command();
  auto& base_feedback_ = mediator_->get_base_feedback();
  auto& servoing_mode_ = mediator_->get_servoing_mode();
  auto& arm_servoing_mode_ = mediator_->get_arm_servoing_mode();

  // low level servoing on startup
  servoing_mode_.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
  arm_servoing_mode_ = Kinova::Api::Base::LOW_LEVEL_SERVOING;
  base_->SetServoingMode(servoing_mode_);
  // initialize kortex api twist commandd
  {
    k_api_twist_command_.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    // command.set_duration = execute time (milliseconds) according to the api ->
    // (not implemented yet)
    // see: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/TwistCommand.md
    k_api_twist_command_.set_duration(0);
    k_api_twist_ = k_api_twist_command_.mutable_twist();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  arm_positions_.resize(kinova_mediator_constants::ACTUATOR_COUNT, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(kinova_mediator_constants::ACTUATOR_COUNT, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(kinova_mediator_constants::ACTUATOR_COUNT, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(kinova_mediator_constants::ACTUATOR_COUNT, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(kinova_mediator_constants::ACTUATOR_COUNT, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(kinova_mediator_constants::ACTUATOR_COUNT, std::numeric_limits<double>::quiet_NaN());
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();

  // set size of the twist interface
  twist_commands_.resize(6, 0.0);

  if (robot_ip == kinova_mediator_constants::IP_ADDRESS_1)
  {
    BL_wrt_GF_frame = BL_wrt_GF_frame_left;
    idSolver = std::make_shared<KDL::ChainIdSolver_RNE>(chain_urdf, gravityVectorLeft);  
  }
  else if (robot_ip == kinova_mediator_constants::IP_ADDRESS_2)
  {
    BL_wrt_GF_frame = BL_wrt_GF_frame_right;
    idSolver = std::make_shared<KDL::ChainIdSolver_RNE>(chain_urdf, gravityVectorRight);
  }

  zero_jnt_velocities.data.setZero();

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  if (
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "true") ||
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "True"))
  {
    use_internal_bus_gripper_comm_ = true;
    RCLCPP_INFO(LOGGER, "Using internal bus communication for gripper!");
  }

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, "export_state_interfaces for joint: %s", info_.joints[i].name.c_str());
    if (info_.joints[i].name == gripper_joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {// all position, velocity and effort values are NaN at the beginning
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }

  // state interface which reports if robot is faulted
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("reset_fault", "internal_fault", &in_fault_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_command_position_));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_velocity", &gripper_speed_command_));
      gripper_speed_command_ = gripper_command_max_velocity_;
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_effort", &gripper_force_command_));
      gripper_force_command_ = gripper_command_max_force_;
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }
  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
    }
  }

  // register twist command interfaces
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.x", &twist_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.y", &twist_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.z", &twist_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.x", &twist_commands_[3]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.y", &twist_commands_[4]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.z", &twist_commands_[5]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("reset_fault", "command", &reset_fault_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "reset_fault", "async_success", &reset_fault_async_success_));

  return command_interfaces;
}

return_type KortexMultiInterfaceHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  // reset auxiliary switching booleans
  stop_joint_based_controller_ = stop_twist_controller_ = stop_fault_controller_ = 
  stop_gripper_controller_ = stop_joint_position_controller_ = stop_joint_velocity_controller_ =
  stop_joint_effort_controller_ = false;
  start_joint_based_controller_ = start_twist_controller_ = start_fault_controller_ =
  start_gripper_controller_ = start_joint_position_controller_ = start_joint_velocity_controller_ =
  start_joint_effort_controller_ = false;

  // sleep to ensure all outgoing write commands have finished
  block_write = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  start_modes_.clear();
  stop_modes_.clear();

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto & key : stop_interfaces)
  {
    for (auto & joint : info_.joints)
    {
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_POSITION &&
        joint.name == gripper_joint_name_)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_GRIPPER);
        continue;
      }
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY &&
        joint.name == gripper_joint_name_)
      {
        continue;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS_VEL_EFFORT);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS_VEL_EFFORT);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS_VEL_EFFORT);
      }
    }
    if (
      (key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") ||
      (key == "tcp/twist.linear.z") || (key == "tcp/twist.angular.x") ||
      (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_TWIST);
    }
    if ((key == "reset_fault/command") || (key == "reset_fault/async_success"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_FAULT_CTRL);
    }
  }

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto & key : start_interfaces)
  {
    for (auto & joint : info_.joints)
    {
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_POSITION &&
        joint.name == gripper_joint_name_)
      {
        start_modes_.emplace_back(StopStartInterface::START_GRIPPER);
        continue;
      }
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY &&
        joint.name == gripper_joint_name_)
      {
        continue;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS_VEL_EFFORT);
        start_joint_based_controller_ = true;
        start_joint_position_controller_ = true;
        start_joint_velocity_controller_ = false;
        start_joint_effort_controller_ = false;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS_VEL_EFFORT);
        start_joint_based_controller_ = true;
        start_joint_velocity_controller_ = true;
        start_joint_position_controller_ = false;
        start_joint_effort_controller_ = false;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS_VEL_EFFORT);
        start_joint_based_controller_ = true;
        start_joint_effort_controller_ = true;
        start_joint_position_controller_ = false;
        start_joint_velocity_controller_ = false;
      }
    }
    if (
      (key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") ||
      (key == "tcp/twist.linear.z") || (key == "tcp/twist.angular.x") ||
      (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      start_modes_.emplace_back(StopStartInterface::START_TWIST);
    }
    if ((key == "reset_fault/command") || (key == "reset_fault/async_success"))
    {
      start_modes_.emplace_back(StopStartInterface::START_FAULT_CTRL);
    }
  }

  // prepare flags for performing the switch
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_POS_VEL_EFFORT) !=
      stop_modes_.end())
  {
    stop_joint_based_controller_ = true;
    stop_joint_position_controller_ = true;
    stop_joint_velocity_controller_ = true;
    stop_joint_effort_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_TWIST) !=
      stop_modes_.end())
  {
    stop_twist_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_GRIPPER) !=
      stop_modes_.end())
  {
    stop_gripper_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_FAULT_CTRL) !=
      stop_modes_.end())
  {
    stop_fault_controller_ = true;
  }

  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_POS_VEL_EFFORT) !=
     start_modes_.end()))
  {
    start_joint_based_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_TWIST) !=
      start_modes_.end())
  {
    start_twist_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_GRIPPER) !=
     start_modes_.end()))
  {
    start_gripper_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_FAULT_CTRL) !=
     start_modes_.end()))
  {
    start_fault_controller_ = true;
  }

  // handle exclusiveness between twist and joint based controller
  if (twist_controller_running_ && start_joint_based_controller_ && !stop_twist_controller_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start joint based controller while twist controller is running!");
    return hardware_interface::return_type::ERROR;
  }
  if (joint_based_controller_running_ && start_twist_controller_ && !stop_joint_based_controller_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start twist controller while joint based controller is running!");
    return hardware_interface::return_type::ERROR;
  }

  return ret_val;
}

void KortexMultiInterfaceHardware::calculate_joint_torques_RNEA(
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
  KDL::JntArray &jnt_torques)
{
  // get joint torques
  jacobDotSolver->JntToJacDot(jnt_velocity, jd_qd);
  xdd_minus_jd_qd = xdd - jd_qd;
  ikSolverAcc->CartToJnt(jnt_positions, xdd_minus_jd_qd, jnt_accelerations);
  idSolver->CartToJnt(jnt_positions, jnt_velocities, jnt_accelerations, linkWrenches_EE, jnt_torques);
}

void KortexMultiInterfaceHardware::get_end_effector_pose_and_twist(KDL::JntArrayVel &jnt_velocity,
  const KDL::JntArray &jnt_positions,
  const KDL::JntArray &jnt_velocities,
  KDL::Frame &measured_endEffPose_BL,
  KDL::FrameVel &measured_endEffTwist_BL,
  KDL::Frame &measured_endEffPose_GF,
  KDL::FrameVel &measured_endEffTwist_GF,
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> &fkSolverPos,
  std::shared_ptr<KDL::ChainFkSolverVel_recursive> &fkSolverVel,
  const KDL::Frame &BL_wrt_GF_frame)
{
  // getting the end effector pose and twist
  jnt_velocity.q = jnt_positions;
  jnt_velocity.qdot = jnt_velocities;

  fkSolverPos->JntToCart(jnt_positions, measured_endEffPose_BL);
  fkSolverVel->JntToCart(jnt_velocity, measured_endEffTwist_BL);

  measured_endEffPose_GF = BL_wrt_GF_frame * measured_endEffPose_BL;
  measured_endEffTwist_GF = BL_wrt_GF_frame * measured_endEffTwist_BL;
}

void KortexMultiInterfaceHardware::kinova_feedback(std::shared_ptr<kinova_mediator> kinova_arm_mediator,
  KDL::JntArray &jnt_positions,
  KDL::JntArray &jnt_velocities,
  KDL::JntArray &jnt_torques)
{
  kinova_arm_mediator->get_joint_state(jnt_positions,
                                      jnt_velocities,
                                      jnt_torques);
}

return_type KortexMultiInterfaceHardware::perform_command_mode_switch(
  const vector<std::string> & /*start_interfaces*/, const vector<std::string> & /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  auto base_ = mediator_->get_base();
  auto base_cyclic_ = mediator_->get_base_cyclic();
  auto& servoing_mode_ = mediator_->get_servoing_mode();
  auto& arm_servoing_mode_ = mediator_->get_arm_servoing_mode();
  if (stop_joint_based_controller_)
  {
    joint_based_controller_running_ = false;
    joint_position_controller_running_ = false;
    joint_velocity_controller_running_ = false;
    joint_effort_controller_running_ = false;
    arm_commands_positions_ = arm_positions_;
    arm_commands_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    arm_commands_efforts_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (stop_twist_controller_)
  {
    twist_controller_running_ = false;
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (stop_gripper_controller_)
  {
    gripper_controller_running_ = false;
    gripper_command_position_ = gripper_position_;
  }
  if (stop_fault_controller_)
  {
    fault_controller_running_ = false;
  }

  if (start_joint_based_controller_)
  {
    joint_based_controller_running_ = true;
    servoing_mode_.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_->SetServoingMode(servoing_mode_);
    arm_servoing_mode_ = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
    twist_controller_running_ = false;
    if (start_joint_position_controller_)
    {
      joint_position_controller_running_ = true;
      mediator_->set_control_mode(control_mode::POSITION, nullptr);
    }
    else if (start_joint_velocity_controller_)
    {
      joint_velocity_controller_running_ = true;
      mediator_->set_control_mode(control_mode::VELOCITY, nullptr);
    }
    else if (start_joint_effort_controller_)
    {

      kinova_feedback(mediator_, jnt_positions, jnt_velocities,
        jnt_torques_read);

      get_end_effector_pose_and_twist(
        jnt_velocity, jnt_positions, jnt_velocities,
        measured_endEffPose_BL_arm, measured_endEffTwist_BL_arm,
        measured_endEffPose_GF_arm, measured_endEffTwist_GF_arm,
        fkSolverPos, fkSolverVel, BL_wrt_GF_frame);

      calculate_joint_torques_RNEA(jacobDotSolver, ikSolverAcc, idSolver,
        jnt_velocity, jd_qd, xdd,
        xdd_minus_jd_qd, jnt_accelerations,
        jnt_positions, jnt_velocities,
        linkWrenches_EE, jnt_torques_cmd);

    // convert JntArray to double array
    for (int i = 0; i < kinova_mediator_constants::ACTUATOR_COUNT; i++)
    {
      arm_commands_efforts_[i] = jnt_torques_cmd(i);
    };
    joint_effort_controller_running_ = true;
    mediator_->set_control_mode(control_mode::TORQUE, arm_commands_efforts_.data());
    }
  }
  if (start_twist_controller_)
  {
    servoing_mode_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base_->SetServoingMode(servoing_mode_);
    arm_servoing_mode_ = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
    joint_based_controller_running_ = false;
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    twist_controller_running_ = true;
  }
  if (start_gripper_controller_)
  {
    gripper_command_position_ = gripper_position_;
    gripper_controller_running_ = true;
  }
  if (start_fault_controller_)
  {
    fault_controller_running_ = true;
  }

  // reset auxiliary switching booleans
  stop_joint_based_controller_ = stop_twist_controller_ = stop_fault_controller_ =
    stop_gripper_controller_ = stop_joint_position_controller_ = stop_joint_velocity_controller_ =
    stop_joint_effort_controller_ = false;
  start_joint_based_controller_ = start_twist_controller_ = start_fault_controller_ =
    start_gripper_controller_ = start_joint_position_controller_ = start_joint_velocity_controller_ =
    start_joint_effort_controller_ = false;

  start_modes_.clear();
  stop_modes_.clear();

  block_write = false;

  return ret_val;
}

CallbackReturn KortexMultiInterfaceHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  auto& base_command_ = mediator_->get_base_command();

  RCLCPP_INFO(LOGGER, "Activating KortexMultiInterfaceHardware...");
  auto base_ = mediator_->get_base();
  auto base_cyclic_ = mediator_->get_base_cyclic();

  // first read
  auto feedback_ = base_cyclic_->RefreshFeedback();

  RCLCPP_INFO(LOGGER, "First read done");

  mediator_->get_joint_positions(arm_positions_);
  auto success_ = mediator_->set_joint_positions(arm_positions_);

  if (success_==-1)
  {
    RCLCPP_ERROR(LOGGER, "Failed to set joint positions!");
    return CallbackReturn::ERROR;
  }

  // Initialize gripper
  float gripper_initial_position =
    feedback_.interconnect().gripper_feedback().motor()[0].position();
  RCLCPP_INFO(LOGGER, "Gripper initial position is '%f'.", gripper_initial_position);

  // to radians
  gripper_command_position_ = gripper_initial_position / 100.0 * 0.81;

  // Initialize interconnect command to current gripper position.
  base_command_.mutable_interconnect()->mutable_command_id()->set_identifier(0);
  gripper_motor_command_ =
    base_command_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
  gripper_motor_command_->set_position(gripper_initial_position);  // % position
  gripper_motor_command_->set_velocity(gripper_speed_command_);    // % speed
  gripper_motor_command_->set_force(gripper_force_command_);       // % force

  // Send a first frame
  // RCLCPP_INFO(LOGGER, "Sending first frame");
  // feedback_ = base_cyclic_->Refresh(base_command_, 0);
  // RCLCPP_INFO(LOGGER, "First frame sent");
  // Set some default values
  for (std::size_t i = 0; i < kinova_mediator_constants::ACTUATOR_COUNT; i++)
  {
    if (std::isnan(arm_positions_[i]))
    {
      arm_positions_[i] = arm_positions_[i];  // rad
    }
    if (std::isnan(arm_velocities_[i]))
    {
      arm_velocities_[i] = 0;
    }
    if (std::isnan(arm_efforts_[i]))
    {
      arm_efforts_[i] = 0;
    }
    if (std::isnan(arm_commands_positions_[i]))
    {
      arm_commands_positions_[i] = arm_positions_[i];  // rad
    }
    if (std::isnan(arm_commands_velocities_[i]))
    {
      arm_commands_velocities_[i] = 0;
    }
    if (std::isnan(arm_commands_efforts_[i]))
    {
      arm_commands_efforts_[i] = 0;
    }
  }

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Deactivating KortexMultiInterfaceHardware...");
  auto base_ = mediator_->get_base();
  auto base_cyclic_ = mediator_->get_base_cyclic();
  auto servoing_mode = k_api::Base::ServoingModeInformation();
  // Set back the servoing mode to Single Level Servoing
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_->SetServoingMode(servoing_mode);

  mediator_->deinitialize();

  // memory handling
  delete k_api_twist_;
  delete gripper_motor_command_;

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

return_type KortexMultiInterfaceHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto base_ = mediator_->get_base();
  auto base_cyclic_ = mediator_->get_base_cyclic();
  auto& base_feedback_ = mediator_->get_base_feedback();

  if (first_pass_)
  {
    first_pass_ = false;
    base_feedback_ = base_cyclic_->RefreshFeedback();
  }

  // read if robot is faulted
  in_fault_ = (base_feedback_.base().active_state() == k_api::Common::ArmState::ARMSTATE_IN_FAULT);

  // read gripper state
  readGripperPosition();

  mediator_->get_joint_positions(arm_positions_);
  mediator_->get_joint_velocities(arm_velocities_);
  mediator_->get_joint_torques(arm_efforts_);

  // add all base's faults and warnings into series
  in_fault_ += (base_feedback_.base().fault_bank_a() + base_feedback_.base().fault_bank_b());

  // TODO(livanov93): separate warnings into another variable to expose it via fault controller
  //     + feedback_.base().warning_bank_a() + feedback_.base().warning_bank_b());

  // add mode that can't be easily reached
  in_fault_ += (base_feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_READY);

  return return_type::OK;
}

void KortexMultiInterfaceHardware::readGripperPosition()
{
  auto& base_feedback_ = mediator_->get_base_feedback(); 
  // max joint angle = 0.81 for robotiq_2f_85
  // TODO(anyone) read in as parameter from kortex_controllers.yaml
  if (use_internal_bus_gripper_comm_)
  {
    gripper_position_ =
    base_feedback_.interconnect().gripper_feedback().motor()[0].position() / 100.0 * 0.81;  // rad
  }
}

return_type KortexMultiInterfaceHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto base_ = mediator_->get_base();
  auto base_cyclic_ = mediator_->get_base_cyclic();
  auto& base_feedback_ = mediator_->get_base_feedback();
  auto& servoing_mode_ = mediator_->get_servoing_mode();
  auto& arm_servoing_mode_ = mediator_->get_arm_servoing_mode();

  if (block_write)
  {
    base_feedback_ = base_cyclic_->RefreshFeedback();
    return return_type::OK;
  }

  if (!std::isnan(reset_fault_cmd_) && fault_controller_running_)
  {
    try
    {
      // change servoing mode first
      servoing_mode_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base_->SetServoingMode(servoing_mode_);
      // apply emergency stop - twice to make it sure as calling it once appeared to be unreliable
      // (detected by testing)
      base_->ApplyEmergencyStop(0, {false, 0, 100});
      base_->ApplyEmergencyStop(0, {false, 0, 100});
      // clear faults
      base_->ClearFaults();
      // back to original servoing mode
      if (
        arm_servoing_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING ||
        arm_servoing_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        servoing_mode_.set_servoing_mode(arm_servoing_mode_);
        base_->SetServoingMode(servoing_mode_);
      }
      reset_fault_async_success_ = 1.0;
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
      reset_fault_async_success_ = 0.0;
    }
    catch (...)
    {
      reset_fault_async_success_ = 0.0;
    }
    reset_fault_cmd_ = NO_CMD;
  }

  if (in_fault_ == 0.0)
  {
    if (arm_servoing_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
    {
      // Twist controller active
      if (twist_controller_running_)
      {
        // twist control
        sendTwistCommand();
      }
      else
      {
        // Keep alive mode - no controller active
        RCLCPP_DEBUG(LOGGER, "No controller active in SINGLE_LEVEL_SERVOING mode!");
      }

      // gripper control
      sendGripperCommand(
        arm_servoing_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);
      // read after write in twist mode
      base_feedback_ = base_cyclic_->RefreshFeedback();
    }
    else if (
      (arm_servoing_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) &&
      (base_feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL))
    {
      // Per joint controller active

      // gripper control
      sendGripperCommand(
        arm_servoing_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);

      if (joint_based_controller_running_)
      {
        // send commands to the joints
        sendJointCommands();
      }
      else
      {
        // Keep alive mode - no controller active
        base_feedback_ = base_cyclic_->RefreshFeedback();
        RCLCPP_DEBUG(LOGGER, "No controller active in LOW_LEVEL_SERVOING mode !");
      }
    }
    else
    {
      // Keep alive mode - no controller active
      base_feedback_ = base_cyclic_->RefreshFeedback();
      RCLCPP_DEBUG(
        LOGGER,
        "Fault was not recognized on the robot but combination of Control Mode and Active State "
        "are not supported!");
    }
  }
  else
  {
    // this is needed when the robot was faulted
    // so we can internally conclude it is not faulted anymore
    base_feedback_ = base_cyclic_->RefreshFeedback();
  }

  return return_type::OK;
}

void KortexMultiInterfaceHardware::prepareCommands()
{  // update the command for each joint
  auto& base_command_ = mediator_->get_base_command();
  for (size_t i = 0; i < kinova_mediator_constants::ACTUATOR_COUNT; i++)
  {
    cmd_degrees_tmp_ = static_cast<float>(
      KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(arm_commands_positions_[i])));
    cmd_vel_tmp_ = static_cast<float>(KortexMathUtil::toDeg(arm_commands_velocities_[i]));
    cmd_eff_tmp_ = static_cast<float>(arm_commands_efforts_[i]);

    base_command_.mutable_actuators(static_cast<int>(i))->set_position(cmd_degrees_tmp_);
    base_command_.mutable_actuators(static_cast<int>(i))->set_velocity(cmd_vel_tmp_);
    base_command_.mutable_actuators(static_cast<int>(i))->set_torque_joint(cmd_eff_tmp_);

    // Velocity command interface not implemented properly in the kortex api
    // base_command_.mutable_actuators(i)->set_velocity(cmd_vel_tmp_);
    base_command_.mutable_actuators(static_cast<int>(i))->set_command_id(base_command_.frame_id());
  }
}

void KortexMultiInterfaceHardware::sendJointCommands()
{
  auto& base_command_ = mediator_->get_base_command();
  auto& base_feedback_ = mediator_->get_base_feedback();
  // identifier++
  incrementId();

  prepareCommands();
  auto base_ = mediator_->get_base();
  auto base_cyclic_ = mediator_->get_base_cyclic();  

  // send the command to the robot
  try
  {
    base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
  }
  catch (k_api::KDetailedException & ex)
  {
    base_feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

    RCLCPP_ERROR_STREAM(
      LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error & ex_runtime)
  {
    base_feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  }
  catch (std::future_error & ex_future)
  {
    base_feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  }
  catch (std::exception & ex_std)
  {
    base_feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
}

void KortexMultiInterfaceHardware::incrementId()
{
  auto& base_command_ = mediator_->get_base_command();  
  // Incrementing identifier ensures actuators can reject out of time frames
  base_command_.set_frame_id(base_command_.frame_id() + 1);
  if (base_command_.frame_id() > 65535) base_command_.set_frame_id(0);
}

void KortexMultiInterfaceHardware::sendGripperCommand(
  k_api::Base::ServoingMode arm_mode, double position, double velocity, double force)
{
  auto base_ = mediator_->get_base();
  auto base_cyclic_ = mediator_->get_base_cyclic();
  if (gripper_controller_running_ && !std::isnan(position) && use_internal_bus_gripper_comm_)
  {
    try
    {
      if (arm_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
      {
        k_api::Base::GripperCommand gripper_command;
        gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
        auto finger = gripper_command.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1);
        finger->set_value(
          static_cast<float>(position / 0.81));  // This values needs to be between 0 and 1
        base_->SendGripperCommand(gripper_command);
      }
      else if (arm_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        // % open/closed, this values needs to be between 0 and 100
        gripper_motor_command_->set_position(static_cast<float>(position / 0.81 * 100.0));
        // % gripper speed between 0 and 100 percent
        gripper_motor_command_->set_velocity(static_cast<float>(velocity));
        // % max force threshold, between 0 and 100
        gripper_motor_command_->set_force(static_cast<float>(force));
      }
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR(LOGGER, "Exception caught while sending internal gripper command!");
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }
  }
}

void KortexMultiInterfaceHardware::sendTwistCommand()
{
  auto base_ = mediator_->get_base();
  k_api_twist_->set_linear_x(static_cast<float>(twist_commands_[0]));
  k_api_twist_->set_linear_y(static_cast<float>(twist_commands_[1]));
  k_api_twist_->set_linear_z(static_cast<float>(twist_commands_[2]));
  k_api_twist_->set_angular_x(static_cast<float>(twist_commands_[3]));
  k_api_twist_->set_angular_y(static_cast<float>(twist_commands_[4]));
  k_api_twist_->set_angular_z(static_cast<float>(twist_commands_[5]));
  base_->SendTwistCommand(k_api_twist_command_);
}

}  // namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
