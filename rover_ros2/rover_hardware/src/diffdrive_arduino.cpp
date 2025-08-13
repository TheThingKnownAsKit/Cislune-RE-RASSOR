// Copyright 2021 ros2_control Development Team
// Modifications for 2-channel Teensy protocol (c) 2025 Cislune / Ella Moody
//
// Licensed under the Apache License, Version 2.0

#include "diffdrive_arduino/diffdrive_arduino.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino {

// ---------------------------------------------------------------------------------------
// ----------------------------- SETUP FUNCTIONS -----------------------------------------
// ---------------------------------------------------------------------------------------

/*
--------------------------- on_init -----------------------------------------
Read parameters, check joint interfaces, and initialize wheel models.
*/
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Required parameters
  cfg_.left_front_wheel_name  = info_.hardware_parameters["left_front_wheel_name"];
  cfg_.left_rear_wheel_name   = info_.hardware_parameters["left_rear_wheel_name"];
  cfg_.right_front_wheel_name = info_.hardware_parameters["right_front_wheel_name"];
  cfg_.right_rear_wheel_name  = info_.hardware_parameters["right_rear_wheel_name"];

  cfg_.loop_rate          = std::stof(info_.hardware_parameters["loop_rate"]);        // Hz
  cfg_.device             = info_.hardware_parameters["device"];                      // /dev/ttyACM*
  cfg_.baud_rate          = std::stoi(info_.hardware_parameters["baud_rate"]);        // 115200
  cfg_.timeout_ms         = std::stoi(info_.hardware_parameters["timeout_ms"]);       // read timeout
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Optional PID parameters (will be forwarded to Teensy)
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
                "PID values not supplied, using defaults.");
  }

  // Wheel objects: store names & encoder scaling
  wheel_lf_.setup(cfg_.left_front_wheel_name,  cfg_.enc_counts_per_rev);
  wheel_lr_.setup(cfg_.left_rear_wheel_name,   cfg_.enc_counts_per_rev);
  wheel_rf_.setup(cfg_.right_front_wheel_name, cfg_.enc_counts_per_rev);
  wheel_rr_.setup(cfg_.right_rear_wheel_name,  cfg_.enc_counts_per_rev);

  // Validate command/state interfaces for each joint
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // one command interface (velocity)
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' must have exactly 1 command interface: '%s'",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // two state interfaces (position, velocity)
    if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' must have 2 state interfaces: position, velocity", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



/*
-------------------- export_state_interfaces --------------------------------
Positions & velocities for all four joints.
*/
std::vector<hardware_interface::StateInterface>
DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(wheel_lf_.name, hardware_interface::HW_IF_POSITION, &wheel_lf_.pos);
  state_interfaces.emplace_back(wheel_lf_.name, hardware_interface::HW_IF_VELOCITY, &wheel_lf_.vel);

  state_interfaces.emplace_back(wheel_lr_.name, hardware_interface::HW_IF_POSITION, &wheel_lr_.pos);
  state_interfaces.emplace_back(wheel_lr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_lr_.vel);

  state_interfaces.emplace_back(wheel_rf_.name, hardware_interface::HW_IF_POSITION, &wheel_rf_.pos);
  state_interfaces.emplace_back(wheel_rf_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rf_.vel);

  state_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_POSITION, &wheel_rr_.pos);
  state_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.vel);

  return state_interfaces;
}



/*
------------------- export_command_interfaces -------------------------------
Velocity commands (rad/s) for all four joints.
*/
std::vector<hardware_interface::CommandInterface>
DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(wheel_lf_.name, hardware_interface::HW_IF_VELOCITY, &wheel_lf_.cmd);
  command_interfaces.emplace_back(wheel_lr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_lr_.cmd);
  command_interfaces.emplace_back(wheel_rf_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rf_.cmd);
  command_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.cmd);

  return command_interfaces;
}


// ---------------------------------------------------------------------------------------
// ----------------------------- LIFECYCLE FUNCTIONS -------------------------------------
// ---------------------------------------------------------------------------------------

/*
------------------- on_configure -------------------------------
Reconfigure the cfg_ parameters.
*/
hardware_interface::CallbackReturn
DiffDriveArduinoHardware::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring...");

  if (comms_.connected()) { comms_.disconnect(); }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}



/*
------------------- on_cleanup -------------------------------
Disconnect comms_.
*/
hardware_interface::CallbackReturn
DiffDriveArduinoHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up...");

  if (comms_.connected()) { comms_.disconnect(); }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}



/*
------------------- on_activate -------------------------------
Activate by sending the pid values to Teensy.
*/
hardware_interface::CallbackReturn
DiffDriveArduinoHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating...");

  if (!comms_.connected()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (cfg_.pid_p > 0) {
    // Forward PID to Teensy. The comms layer may down-map to "p <Kp> <Ki>" as needed.
    comms_.set_pid_values(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}



/*
------------------- on_deactivate -------------------------------
Doesn't do anything as of right now.
*/
hardware_interface::CallbackReturn
DiffDriveArduinoHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}



/*
-------------------------------- read ---------------------------------------
Poll two-channel encoder deltas (Left, Right), accumulate into per-wheel
cumulative encoder counts, then update joint positions (rad) & velocities.
*/
hardware_interface::return_type
DiffDriveArduinoHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Teensy returns change in counts since last E, averaged per side
  int dLeft_counts  = 0;
  int dRight_counts = 0;
  comms_.read_encoder_deltas_lr(dLeft_counts, dRight_counts);

  // Accumulate into per-wheel cumulative counts (assign the side delta to each wheel)
  wheel_lf_.enc += dLeft_counts;
  wheel_lr_.enc += dLeft_counts;
  wheel_rf_.enc += dRight_counts;
  wheel_rr_.enc += dRight_counts;

  const double dt = period.seconds();

  // Update positions (rad) and velocities (rad/s)
  // calc_enc_angle() converts cumulative encoder counts -> wheel angle (rad)
  double pos_prev = wheel_lf_.pos;
  wheel_lf_.pos = wheel_lf_.calc_enc_angle();
  wheel_lf_.vel = (wheel_lf_.pos - pos_prev) / dt;

  pos_prev = wheel_lr_.pos;
  wheel_lr_.pos = wheel_lr_.calc_enc_angle();
  wheel_lr_.vel = (wheel_lr_.pos - pos_prev) / dt;

  pos_prev = wheel_rf_.pos;
  wheel_rf_.pos = wheel_rf_.calc_enc_angle();
  wheel_rf_.vel = (wheel_rf_.pos - pos_prev) / dt;

  pos_prev = wheel_rr_.pos;
  wheel_rr_.pos = wheel_rr_.calc_enc_angle();
  wheel_rr_.vel = (wheel_rr_.pos - pos_prev) / dt;

  return hardware_interface::return_type::OK;
}



/*
-------------------------------- write --------------------------------------
Convert each joint's desired wheel speed (rad/s) into encoder counts/loop,
then aggregate per-side (average) and send a 2-channel command "M L R".
*/
hardware_interface::return_type
DiffDriveArduinoHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Convert each wheel command (rad/s) into counts per loop
  // rads_per_count = (2pi / enc_counts_per_rev); so:
  // counts_per_second = wheel.cmd / rads_per_count
  // counts_per_loop   = counts_per_second / loop_rate
  auto to_counts_per_loop = [&](const Wheel & w) -> int {
    // Avoid division by zero if rads_per_count not set
    if (w.rads_per_count <= std::numeric_limits<double>::epsilon()) return 0;
    const double cps = w.cmd / w.rads_per_count;         // counts/s
    const double cpl = cps / cfg_.loop_rate;             // counts/loop
    return static_cast<int>(std::lround(cpl));           // Teensy expects integers
  };

  const int lf_cpl = to_counts_per_loop(wheel_lf_);
  const int lr_cpl = to_counts_per_loop(wheel_lr_);
  const int rf_cpl = to_counts_per_loop(wheel_rf_);
  const int rr_cpl = to_counts_per_loop(wheel_rr_);

  // Aggregate per side. We use AVERAGE so the side scaling remains identical
  // to a single wheel's ticks_per_rev.
  const int left_cpl  = static_cast<int>(std::lround((lf_cpl + lr_cpl) / 2.0));
  const int right_cpl = static_cast<int>(std::lround((rf_cpl + rr_cpl) / 2.0));

  // Send 2-channel command to Teensy: M <left> <right>
  comms_.set_motor_values_lr(left_cpl, right_cpl);

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
