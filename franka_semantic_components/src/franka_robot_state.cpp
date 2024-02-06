// Copyright (c) 2023 Franka Robotics GmbH
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

#include "franka_semantic_components/franka_robot_state.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "translation_utils.hpp"

namespace {

const size_t kBaseLinkIndex = 0;
const size_t kFlangeLinkIndex = 8;
const size_t kLoadLinkIndex = 8;
const std::string kTCPFrameName = "_hand_tcp";

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable_v<From> &&
                     std::is_trivially_copyable_v<To>,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible_v<To>,
                "This implementation additionally requires "
                "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

franka_msgs::msg::Errors errorsToMessage(const franka::Errors& error) {
  franka_msgs::msg::Errors message;
  message.joint_position_limits_violation =
      static_cast<decltype(message.joint_position_limits_violation)>(
          error.joint_position_limits_violation);
  message.cartesian_position_limits_violation =
      static_cast<decltype(message.cartesian_position_limits_violation)>(
          error.cartesian_position_limits_violation);
  message.self_collision_avoidance_violation =
      static_cast<decltype(message.self_collision_avoidance_violation)>(
          error.self_collision_avoidance_violation);
  message.joint_velocity_violation =
      static_cast<decltype(message.joint_velocity_violation)>(error.joint_velocity_violation);
  message.cartesian_velocity_violation =
      static_cast<decltype(message.cartesian_velocity_violation)>(
          error.cartesian_velocity_violation);
  message.force_control_safety_violation =
      static_cast<decltype(message.force_control_safety_violation)>(
          error.force_control_safety_violation);
  message.joint_reflex = static_cast<decltype(message.joint_reflex)>(error.joint_reflex);
  message.cartesian_reflex =
      static_cast<decltype(message.cartesian_reflex)>(error.cartesian_reflex);
  message.max_goal_pose_deviation_violation =
      static_cast<decltype(message.max_goal_pose_deviation_violation)>(
          error.max_goal_pose_deviation_violation);
  message.max_path_pose_deviation_violation =
      static_cast<decltype(message.max_path_pose_deviation_violation)>(
          error.max_path_pose_deviation_violation);
  message.cartesian_velocity_profile_safety_violation =
      static_cast<decltype(message.cartesian_velocity_profile_safety_violation)>(
          error.cartesian_velocity_profile_safety_violation);
  message.joint_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.joint_position_motion_generator_start_pose_invalid)>(
          error.joint_position_motion_generator_start_pose_invalid);
  message.joint_motion_generator_position_limits_violation =
      static_cast<decltype(message.joint_motion_generator_position_limits_violation)>(
          error.joint_motion_generator_position_limits_violation);
  message.joint_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.joint_motion_generator_velocity_limits_violation)>(
          error.joint_motion_generator_velocity_limits_violation);
  message.joint_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.joint_motion_generator_velocity_discontinuity)>(
          error.joint_motion_generator_velocity_discontinuity);
  message.joint_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.joint_motion_generator_acceleration_discontinuity)>(
          error.joint_motion_generator_acceleration_discontinuity);
  message.cartesian_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.cartesian_position_motion_generator_start_pose_invalid)>(
          error.cartesian_position_motion_generator_start_pose_invalid);
  message.cartesian_motion_generator_elbow_limit_violation =
      static_cast<decltype(message.cartesian_motion_generator_elbow_limit_violation)>(
          error.cartesian_motion_generator_elbow_limit_violation);
  message.cartesian_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_velocity_limits_violation)>(
          error.cartesian_motion_generator_velocity_limits_violation);
  message.cartesian_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_velocity_discontinuity)>(
          error.cartesian_motion_generator_velocity_discontinuity);
  message.cartesian_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_acceleration_discontinuity)>(
          error.cartesian_motion_generator_acceleration_discontinuity);
  message.cartesian_motion_generator_elbow_sign_inconsistent =
      static_cast<decltype(message.cartesian_motion_generator_elbow_sign_inconsistent)>(
          error.cartesian_motion_generator_elbow_sign_inconsistent);
  message.cartesian_motion_generator_start_elbow_invalid =
      static_cast<decltype(message.cartesian_motion_generator_start_elbow_invalid)>(
          error.cartesian_motion_generator_start_elbow_invalid);
  message.cartesian_motion_generator_joint_position_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_position_limits_violation)>(
          error.cartesian_motion_generator_joint_position_limits_violation);
  message.cartesian_motion_generator_joint_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_limits_violation)>(
          error.cartesian_motion_generator_joint_velocity_limits_violation);
  message.cartesian_motion_generator_joint_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_discontinuity)>(
          error.cartesian_motion_generator_joint_velocity_discontinuity);
  message.cartesian_motion_generator_joint_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_acceleration_discontinuity)>(
          error.cartesian_motion_generator_joint_acceleration_discontinuity);
  message.cartesian_position_motion_generator_invalid_frame =
      static_cast<decltype(message.cartesian_position_motion_generator_invalid_frame)>(
          error.cartesian_position_motion_generator_invalid_frame);
  message.force_controller_desired_force_tolerance_violation =
      static_cast<decltype(message.force_controller_desired_force_tolerance_violation)>(
          error.force_controller_desired_force_tolerance_violation);
  message.controller_torque_discontinuity =
      static_cast<decltype(message.controller_torque_discontinuity)>(
          error.controller_torque_discontinuity);
  message.start_elbow_sign_inconsistent =
      static_cast<decltype(message.start_elbow_sign_inconsistent)>(
          error.start_elbow_sign_inconsistent);
  message.communication_constraints_violation =
      static_cast<decltype(message.communication_constraints_violation)>(
          error.communication_constraints_violation);
  message.power_limit_violation =
      static_cast<decltype(message.power_limit_violation)>(error.power_limit_violation);
  message.joint_p2p_insufficient_torque_for_planning =
      static_cast<decltype(message.joint_p2p_insufficient_torque_for_planning)>(
          error.joint_p2p_insufficient_torque_for_planning);
  message.tau_j_range_violation =
      static_cast<decltype(message.tau_j_range_violation)>(error.tau_j_range_violation);
  message.instability_detected =
      static_cast<decltype(message.instability_detected)>(error.instability_detected);
  message.joint_move_in_wrong_direction =
      static_cast<decltype(message.joint_move_in_wrong_direction)>(error.joint_move_in_wrong_direction);
  
  // base acceleration tag needs to be added; seems like that was removed for FR3
  return message;
}

}  // anonymous namespace

namespace franka_semantic_components {

FrankaRobotState::FrankaRobotState(const std::string& name, const std::string& robot_description)
    : SemanticComponentInterface(name, 1), model_(std::make_shared<urdf::Model>()) {
  interface_names_.emplace_back(name_);
  robot_description_ = robot_description;
  if (!model_->initString(robot_description_)) {
    throw std::runtime_error("Failed to parse URDF.");
  }

  robot_name_ = get_robot_name_from_urdf();
  gripper_loaded_ = is_gripper_loaded();

  set_links_from_urdf();
  set_joints_from_urdf();

  if (gripper_loaded_) {
    kEndEffectorLinkIndex = get_link_index(robot_name_ + kTCPFrameName);
    kStiffnessLinkIndex = kEndEffectorLinkIndex;
  } else {
    kEndEffectorLinkIndex = kFlangeLinkIndex;
    kStiffnessLinkIndex = kEndEffectorLinkIndex;
  }
}

auto FrankaRobotState::get_link_index(const std::string& link_name) -> size_t {
  auto link_index = std::find(link_names.cbegin(), link_names.cend(), link_name);
  if (link_index != link_names.end()) {
    return std::distance(link_names.cbegin(), link_index);
  } else {
    throw std::runtime_error("Link name not found in URDF.");
  }
}

auto FrankaRobotState::is_gripper_loaded() -> bool {
  const auto& links = model_->links_;
  bool gripper_loaded = links.find(robot_name_ + kTCPFrameName) != links.end();

  return gripper_loaded;
}

auto FrankaRobotState::get_robot_name_from_urdf() -> std::string {
  return model_->name_;
}

auto FrankaRobotState::set_child_links_recursively(const std::shared_ptr<const urdf::Link>& link)
    -> void {
  for (const auto& child_link : link->child_links) {
    link_names.push_back(child_link->name);
    set_child_links_recursively(child_link);
  }
}

auto FrankaRobotState::set_links_from_urdf() -> void {
  auto root_link = model_->getRoot();
  link_names.push_back(root_link->name);
  set_child_links_recursively(root_link);
}

auto FrankaRobotState::set_joints_from_urdf() -> void {
  auto& joints = model_->joints_;
  for (const auto& [name, joint] : joints) {
    if (joint->type == urdf::Joint::REVOLUTE) {
      joint_names.push_back(name);
    }
  }
}

auto FrankaRobotState::initialize_robot_state_msg(franka_msgs::msg::FrankaRobotState& message)
    -> void {
  // The joint state - joint 1 is the first joint while joint 7 is the last revolute joint
  message.measured_joint_state.name =
      std::vector<std::string>(joint_names.cbegin(), joint_names.cend());
  message.desired_joint_state.name =
      std::vector<std::string>(joint_names.cbegin(), joint_names.cend());
  message.measured_joint_motor_state.name =
      std::vector<std::string>(joint_names.cbegin(), joint_names.cend());
  message.tau_ext_hat_filtered.name =
      std::vector<std::string>(joint_names.cbegin(), joint_names.cend());

  message.measured_joint_state.header.frame_id = link_names[kBaseLinkIndex];
  message.desired_joint_state.header.frame_id = link_names[kBaseLinkIndex];
  message.measured_joint_motor_state.header.frame_id = link_names[kBaseLinkIndex];
  message.tau_ext_hat_filtered.header.frame_id = link_names[kBaseLinkIndex];

  // Active wrenches
  message.o_f_ext_hat_k.header.frame_id = link_names[kBaseLinkIndex];
  message.k_f_ext_hat_k.header.frame_id = link_names[kStiffnessLinkIndex];

  // Current EE Pose
  message.o_t_ee.header.frame_id = link_names[kBaseLinkIndex];
  // Desired EE Pose
  message.o_t_ee_d.header.frame_id = link_names[kBaseLinkIndex];
  // Commanded EE Pose
  message.o_t_ee_c.header.frame_id = link_names[kBaseLinkIndex];

  message.f_t_ee.header.frame_id = link_names[kFlangeLinkIndex];
  message.ee_t_k.header.frame_id = link_names[kEndEffectorLinkIndex];

  message.o_dp_ee_d.header.frame_id = link_names[kBaseLinkIndex];
  message.o_dp_ee_c.header.frame_id = link_names[kBaseLinkIndex];
  message.o_ddp_ee_c.header.frame_id = link_names[kBaseLinkIndex];

  // The inertias are with respect to the Center Of Mass.
  // TODO(yazi_ba) frame ids should be referenced to the Center Of Mass
  message.inertia_ee.header.frame_id = link_names[kEndEffectorLinkIndex];
  message.inertia_load.header.frame_id = link_names[kLoadLinkIndex];
  message.inertia_total.header.frame_id = link_names[kEndEffectorLinkIndex];
}

auto FrankaRobotState::get_values_as_message(franka_msgs::msg::FrankaRobotState& message) -> bool {
  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;

  auto franka_state_interface =
      std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                   [&full_interface_name](const auto& interface) {
                     return interface.get().get_name() == full_interface_name;
                   });

  if (franka_state_interface != state_interfaces_.end()) {
    robot_state_ptr = bit_cast<franka::RobotState*>((*franka_state_interface).get().get_value());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("franka_state_semantic_component"),
                 "Franka state interface does not exist! Did you assign the loaned state in the "
                 "controller?");
    return false;
  }

  static_assert(
      sizeof(robot_state_ptr->cartesian_collision) == sizeof(robot_state_ptr->cartesian_contact),
      "Robot state Cartesian members do not have same size");
  static_assert(
      sizeof(robot_state_ptr->cartesian_collision) == sizeof(robot_state_ptr->K_F_ext_hat_K),
      "Robot state Cartesian members do not have same size");
  static_assert(
      sizeof(robot_state_ptr->cartesian_collision) == sizeof(robot_state_ptr->O_F_ext_hat_K),
      "Robot state Cartesian members do not have same size");
  static_assert(sizeof(robot_state_ptr->cartesian_collision) == sizeof(robot_state_ptr->O_dP_EE_d),
                "Robot state Cartesian members do not have same size");
  static_assert(sizeof(robot_state_ptr->cartesian_collision) == sizeof(robot_state_ptr->O_dP_EE_c),
                "Robot state Cartesian members do not have same size");
  static_assert(sizeof(robot_state_ptr->cartesian_collision) == sizeof(robot_state_ptr->O_ddP_EE_c),
                "Robot state Cartesian members do not have same size");
  for (size_t i = 0; i < robot_state_ptr->cartesian_collision.size(); i++) {
    message.cartesian_collision[i] = robot_state_ptr->cartesian_collision[i];
    message.cartesian_contact[i] = robot_state_ptr->cartesian_contact[i];
    message.k_f_ext_hat_k[i] = robot_state_ptr->K_F_ext_hat_K[i];
    message.o_f_ext_hat_k[i] = robot_state_ptr->O_F_ext_hat_K[i];
    message.o_dp_ee_d[i] = robot_state_ptr->O_dP_EE_d[i];
    message.o_dp_ee_c[i] = robot_state_ptr->O_dP_EE_c[i];
    message.o_ddp_ee_c[i] = robot_state_ptr->O_ddP_EE_c[i];
  }

  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->q_d),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->dq),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->dq_d),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->ddq_d),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->tau_J),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->dtau_J),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->tau_J_d),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->theta),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->dtheta),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->joint_collision),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->joint_contact),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr->q) == sizeof(robot_state_ptr->tau_ext_hat_filtered),
                "Robot state joint members do not have same size");
  for (size_t i = 0; i < robot_state_ptr->q.size(); i++) {
    message.q[i] = robot_state_ptr->q[i];
    message.q_d[i] = robot_state_ptr->q_d[i];
    message.dq[i] = robot_state_ptr->dq[i];
    message.dq_d[i] = robot_state_ptr->dq_d[i];
    message.ddq_d[i] = robot_state_ptr->ddq_d[i];
    message.tau_j[i] = robot_state_ptr->tau_J[i];
    message.dtau_j[i] = robot_state_ptr->dtau_J[i];
    message.tau_j_d[i] = robot_state_ptr->tau_J_d[i];
    message.theta[i] = robot_state_ptr->theta[i];
    message.dtheta[i] = robot_state_ptr->dtheta[i];
    message.joint_collision[i] = robot_state_ptr->joint_collision[i];
    message.joint_contact[i] = robot_state_ptr->joint_contact[i];
    message.tau_ext_hat_filtered[i] = robot_state_ptr->tau_ext_hat_filtered[i];
  }

  static_assert(sizeof(robot_state_ptr->elbow) == sizeof(robot_state_ptr->elbow_d),
                "Robot state elbow configuration members do not have same size");
  static_assert(sizeof(robot_state_ptr->elbow) == sizeof(robot_state_ptr->elbow_c),
                "Robot state elbow configuration members do not have same size");
  static_assert(sizeof(robot_state_ptr->elbow) == sizeof(robot_state_ptr->delbow_c),
                "Robot state elbow configuration members do not have same size");
  static_assert(sizeof(robot_state_ptr->elbow) == sizeof(robot_state_ptr->ddelbow_c),
                "Robot state elbow configuration members do not have same size");

  for (size_t i = 0; i < robot_state_ptr->elbow.size(); i++) {
    message.elbow[i] = robot_state_ptr->elbow[i];
    message.elbow_d[i] = robot_state_ptr->elbow_d[i];
    message.elbow_c[i] = robot_state_ptr->elbow_c[i];
    message.delbow_c[i] = robot_state_ptr->delbow_c[i];
    message.ddelbow_c[i] = robot_state_ptr->ddelbow_c[i];
  }

  static_assert(sizeof(robot_state_ptr->O_T_EE) == sizeof(robot_state_ptr->F_T_EE),
                "Robot state transforms do not have same size");
  static_assert(sizeof(robot_state_ptr->O_T_EE) == sizeof(robot_state_ptr->F_T_NE),
                  "Robot state transforms do not have same size");
  static_assert(sizeof(robot_state_ptr->O_T_EE) == sizeof(robot_state_ptr->NE_T_EE),
                  "Robot state transforms do not have same size");
  static_assert(sizeof(robot_state_ptr->O_T_EE) == sizeof(robot_state_ptr->EE_T_K),
                "Robot state transforms do not have same size");
  static_assert(sizeof(robot_state_ptr->O_T_EE) == sizeof(robot_state_ptr->O_T_EE_d),
                "Robot state transforms do not have same size");
  static_assert(sizeof(robot_state_ptr->O_T_EE) == sizeof(robot_state_ptr->O_T_EE_c),
                "Robot state transforms do not have same size");
  for (size_t i = 0; i < robot_state_ptr->O_T_EE.size(); i++) {
    message.o_t_ee[i] = robot_state_ptr->O_T_EE[i];
    message.f_t_ee[i] = robot_state_ptr->F_T_EE[i];
    message.f_t_ne[i] = robot_state_ptr->F_T_NE[i];
    message.ne_t_ee[i] = robot_state_ptr->NE_T_EE[i];
    message.ee_t_k[i] = robot_state_ptr->EE_T_K[i];
    message.o_t_ee_d[i] = robot_state_ptr->O_T_EE_d[i];
    message.o_t_ee_c[i] = robot_state_ptr->O_T_EE_c[i];
  }
  message.m_ee = robot_state_ptr->m_ee;
  message.m_load = robot_state_ptr->m_load;
  message.m_total = robot_state_ptr->m_total;

  for (size_t i = 0; i < robot_state_ptr->I_load.size(); i++) {
    message.i_ee[i] = robot_state_ptr->I_ee[i];
    message.i_load[i] = robot_state_ptr->I_load[i];
    message.i_total[i] = robot_state_ptr->I_total[i];
  }

  for (size_t i = 0; i < robot_state_ptr->F_x_Cload.size(); i++) {
    message.f_x_cee[i] = robot_state_ptr->F_x_Cee[i];
    message.f_x_cload[i] = robot_state_ptr->F_x_Cload[i];
    message.f_x_ctotal[i] = robot_state_ptr->F_x_Ctotal[i];
  }

  message.time = robot_state_ptr->time.toSec();
  message.control_command_success_rate = robot_state_ptr->control_command_success_rate;
  message.current_errors = errorsToMessage(robot_state_ptr->current_errors);
  message.last_motion_errors = errorsToMessage(robot_state_ptr->last_motion_errors);

  switch (robot_state_ptr->robot_mode) {
    case franka::RobotMode::kOther:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_OTHER;
      break;

    case franka::RobotMode::kIdle:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_IDLE;
      break;

    case franka::RobotMode::kMove:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_MOVE;
      break;

    case franka::RobotMode::kGuiding:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_GUIDING;
      break;

    case franka::RobotMode::kReflex:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_REFLEX;
      break;

    case franka::RobotMode::kUserStopped:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_USER_STOPPED;
      break;

    case franka::RobotMode::kAutomaticErrorRecovery:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
      break;
  }
  return true;
}

}  // namespace franka_semantic_components
