// Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

#include "so101_hardware/so101_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so101_hardware
{

SO101SystemHardware::SO101SystemHardware()
: driver_initialized_(false)
{
}

SO101SystemHardware::~SO101SystemHardware()
{
  if (driver_initialized_) {
    servo_driver_.end();
  }
}

hardware_interface::CallbackReturn SO101SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get port parameter (default: /dev/ttyACM0)
  port_ = info_.hardware_parameters["port"];
  if (port_.empty()) {
    port_ = "/dev/ttyACM0";
  }

  // Get calibration file parameter
  calibration_file_ = info_.hardware_parameters["calibration_file"];

  // Initialize motor ID mapping
  motor_ids_["shoulder_pan"] = 1;
  motor_ids_["shoulder_lift"] = 2;
  motor_ids_["elbow_flex"] = 3;
  motor_ids_["wrist_flex"] = 4;
  motor_ids_["wrist_roll"] = 5;
  motor_ids_["gripper"] = 6;

  // Initialize state and command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Load initial positions if specified
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];
    if (joint.parameters.find("initial_position") != joint.parameters.end())
    {
      hw_positions_[i] = std::stod(joint.parameters.at("initial_position"));
      hw_commands_[i] = hw_positions_[i];
      RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"),
                  "Joint '%s' initial position set to: %.3f",
                  joint.name.c_str(), hw_positions_[i]);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO101SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Configuring...please wait...");

  // Load calibration data
  if (!calibration_file_.empty() && !loadCalibration()) {
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"),
                 "Failed to load calibration from: %s", calibration_file_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize servo driver
  if (!servo_driver_.begin(115200, port_.c_str())) {
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"),
                 "Failed to connect to motor bus on port %s. Check connection and permissions.",
                 port_.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"), "TROUBLESHOOTING:");
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"),
                 "1. Check if robot is connected: ls /dev/ttyACM* /dev/ttyUSB*");
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"),
                 "2. If robot is on different port, launch with: port:=/dev/ttyACMX");
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"),
                 "3. Check permissions: sudo usermod -aG dialout $USER (then logout/login)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  driver_initialized_ = true;

  // Initialize joint positions and commands
  for (size_t i = 0; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0.0;
    }
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SO101SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SO101SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SO101SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Activating...please wait...");

  // Enable torque for all motors
  for (const auto & pair : motor_ids_) {
    int motor_id = pair.second;
    if (servo_driver_.EnableTorque(motor_id, 1) == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"),
                   "Failed to enable torque for motor %d", motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Read current positions
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Current motor positions:");
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const std::string& joint_name = info_.joints[i].name;
    int motor_id = motor_ids_[joint_name];

    if (servo_driver_.FeedBack(motor_id) != -1) {
      int raw_pos = servo_driver_.ReadPos(-1);
      bool is_gripper = (joint_name == "gripper");
      hw_positions_[i] = rawToRadians(raw_pos, motor_calibration_[motor_id], is_gripper);

      hw_commands_[i] = hw_positions_[i];

      RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"),
                  "  %s: %.3f rad (motor: %d raw)",
                  joint_name.c_str(), hw_positions_[i], raw_pos);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO101SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Deactivating...please wait...");

  // Disable torque for all motors
  for (const auto & pair : motor_ids_) {
    int motor_id = pair.second;
    if (servo_driver_.EnableTorque(motor_id, 0) == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"),
                   "Failed to disable torque for motor %d", motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SO101SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read positions from all motors
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const std::string& joint_name = info_.joints[i].name;
    int motor_id = motor_ids_[joint_name];

    if (servo_driver_.FeedBack(motor_id) != -1) {
      int raw_pos = servo_driver_.ReadPos(-1);
      bool is_gripper = (joint_name == "gripper");

      hw_positions_[i] = rawToRadians(raw_pos, motor_calibration_[motor_id], is_gripper);
    }

    hw_velocities_[i] = 0.0;  // TODO: Read actual velocity if needed
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SO101SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Prepare arrays for sync write
  std::vector<u8> motor_ids;
  std::vector<s16> positions;
  std::vector<u16> speeds;
  std::vector<u8> accelerations;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (!std::isnan(hw_commands_[i])) {
      const std::string& joint_name = info_.joints[i].name;
      int motor_id = motor_ids_[joint_name];
      bool is_gripper = (joint_name == "gripper");

      int raw_position = radiansToRaw(hw_commands_[i], motor_calibration_[motor_id], is_gripper);

      motor_ids.push_back(motor_id);
      positions.push_back(raw_position);
      speeds.push_back(2400);  // Default speed
      accelerations.push_back(50);  // Default acceleration
    }
  }

  // Write positions to all motors synchronously
  if (!motor_ids.empty()) {
    servo_driver_.SyncWritePosEx(motor_ids.data(), motor_ids.size(),
                                  positions.data(), speeds.data(),
                                  accelerations.data());
  }

  return hardware_interface::return_type::OK;
}

bool SO101SystemHardware::loadCalibration()
{
  if (calibration_file_.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"),
                "No calibration file specified, using default values");
    return true;
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"),
              "Loading calibration from: %s", calibration_file_.c_str());

  std::ifstream file(calibration_file_);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"),
                 "Failed to open calibration file: %s", calibration_file_.c_str());
    return false;
  }

  // Simple YAML parser for calibration file
  std::string line;
  std::string current_motor;
  MotorCalibration current_calib = {0, 0, 0, 0};

  while (std::getline(file, line)) {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#') continue;

    // Check if this is a motor name line (no leading spaces, ends with :)
    if (line.find_first_not_of(" \t") == 0 && line.back() == ':') {
      // Save previous motor if exists
      if (!current_motor.empty() && motor_ids_.count(current_motor) > 0) {
        int motor_id = motor_ids_[current_motor];
        motor_calibration_[motor_id] = current_calib;
      }

      // Start new motor
      current_motor = line.substr(0, line.length() - 1);
      current_calib = {0, 0, 0, 0};
      continue;
    }

    // Parse key-value pairs
    size_t colon_pos = line.find(':');
    if (colon_pos != std::string::npos) {
      std::string key = line.substr(0, colon_pos);
      std::string value = line.substr(colon_pos + 1);

      // Trim whitespace
      key.erase(0, key.find_first_not_of(" \t"));
      key.erase(key.find_last_not_of(" \t") + 1);
      value.erase(0, value.find_first_not_of(" \t"));
      value.erase(value.find_last_not_of(" \t") + 1);

      if (key == "id") current_calib.id = std::stoi(value);
      else if (key == "drive_mode") current_calib.drive_mode = std::stoi(value);
      else if (key == "range_min") current_calib.range_min = std::stoi(value);
      else if (key == "range_max") current_calib.range_max = std::stoi(value);
    }
  }

  // Save last motor
  if (!current_motor.empty() && motor_ids_.count(current_motor) > 0) {
    int motor_id = motor_ids_[current_motor];
    motor_calibration_[motor_id] = current_calib;
  }

  file.close();

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"),
              "Successfully loaded calibration for %zu motors", motor_calibration_.size());

  return true;
}

double SO101SystemHardware::rawToRadians(int raw_position, const MotorCalibration& calib, bool is_gripper)
{
  // Clamp to calibration range (no homing offset applied)
  int clamped = std::max(calib.range_min, std::min(calib.range_max, raw_position));

  // Normalize to [0, 1] based on calibrated range
  double progress = static_cast<double>(clamped - calib.range_min) /
                   static_cast<double>(calib.range_max - calib.range_min);

  // Get URDF limits for each joint
  // TODO: Load these from URDF dynamically in on_configure()
  double urdf_lower, urdf_upper;
  if (calib.id == 1) {  // shoulder_pan
    urdf_lower = -1.91986; urdf_upper = 1.91986;
  } else if (calib.id == 2) {  // shoulder_lift
    urdf_lower = -1.74533; urdf_upper = 1.74533;
  } else if (calib.id == 3) {  // elbow_flex
    urdf_lower = -1.74533; urdf_upper = 1.5708;
  } else if (calib.id == 4) {  // wrist_flex
    urdf_lower = -1.65806; urdf_upper = 1.65806;
  } else if (calib.id == 5) {  // wrist_roll
    urdf_lower = -2.79253; urdf_upper = 2.79253;
  } else if (calib.id == 6) {  // gripper - use radians like other joints!
    urdf_lower = -0.1745; urdf_upper = 1.4483;
  } else {
    urdf_lower = -M_PI; urdf_upper = M_PI;
  }

  // Scale to URDF limits (radians for ALL joints, including gripper)
  return progress * (urdf_upper - urdf_lower) + urdf_lower;
}

int SO101SystemHardware::radiansToRaw(double radians, const MotorCalibration& calib, bool is_gripper)
{
  // Get URDF limits for each joint
  // TODO: Load these from URDF dynamically in on_configure()
  double urdf_lower, urdf_upper;
  if (calib.id == 1) {  // shoulder_pan
    urdf_lower = -1.91986; urdf_upper = 1.91986;
  } else if (calib.id == 2) {  // shoulder_lift
    urdf_lower = -1.74533; urdf_upper = 1.74533;
  } else if (calib.id == 3) {  // elbow_flex
    urdf_lower = -1.74533; urdf_upper = 1.5708;
  } else if (calib.id == 4) {  // wrist_flex
    urdf_lower = -1.65806; urdf_upper = 1.65806;
  } else if (calib.id == 5) {  // wrist_roll
    urdf_lower = -2.79253; urdf_upper = 2.79253;
  } else if (calib.id == 6) {  // gripper - use radians like other joints!
    urdf_lower = -0.1745; urdf_upper = 1.4483;
  } else {
    urdf_lower = -M_PI; urdf_upper = M_PI;
  }

  // Clamp to URDF limits
  double clamped_radians = std::min(urdf_upper, std::max(urdf_lower, radians));

  // Normalize to [0, 1]
  double progress = (clamped_radians - urdf_lower) / (urdf_upper - urdf_lower);
  
  // Scale to motor range
  int raw_position = static_cast<int>(progress * (calib.range_max - calib.range_min) + calib.range_min);

  // Return raw position directly (no homing offset)
  return raw_position;
}

}  // namespace so101_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  so101_hardware::SO101SystemHardware, hardware_interface::SystemInterface)
