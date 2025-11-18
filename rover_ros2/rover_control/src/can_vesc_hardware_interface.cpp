#include "rover_control/can_vesc_hardware_interface.hpp"
#include <cstring>

namespace rover_control
{

hardware_interface::CallbackReturn CanVescHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_interface_ = info.hardware_parameters.at("can_interface");
  wheel_radius_  = std::stod(info.hardware_parameters.at("wheel_radius"));
  gear_ratio_    = std::stod(info.hardware_parameters.at("gear_ratio"));
  motor_pole_pairs_ = 7;
  if (info.hardware_parameters.find("motor_pole_pairs") != info.hardware_parameters.end()) {
    motor_pole_pairs_ = std::stoi(info.hardware_parameters.at("motor_pole_pairs"));
  }

  if (info.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL(rclcpp::get_logger("CanVescHardwareInterface"),
                "Expected %d joints, but got %zu in URDF", NUM_JOINTS, info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  vesc_ids_ = {1, 2, 3, 4};

  cmd_velocity_.assign(NUM_JOINTS, 0.0);
  state_velocity_.assign(NUM_JOINTS, 0.0);

  for (const auto & joint : info.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
        joint.state_interfaces.size() != 1   || joint.state_interfaces[0].name   != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("CanVescHardwareInterface"),
                  "Joint '%s' interfaces are not configured correctly. Expected exactly one velocity command and state interface.",
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanVescHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  struct ifreq ifr;
  struct sockaddr_can addr;
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("CanVescHardwareInterface"), "Failed to create CAN socket");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("CanVescHardwareInterface"), "CAN interface %s not found", can_interface_.c_str());
    ::close(can_socket_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("CanVescHardwareInterface"), "Failed to bind to CAN interface %s", can_interface_.c_str());
    ::close(can_socket_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  fcntl(can_socket_, F_SETFL, O_NONBLOCK);
  RCLCPP_INFO(rclcpp::get_logger("CanVescHardwareInterface"), "CAN interface %s configured and socket bound", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanVescHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  struct can_frame frame;
  frame.can_id  = 0;
  frame.can_dlc = 4;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    uint32_t can_id = (vesc_ids_[i] & 0xFF) | ((uint32_t)3 << 8);
    frame.can_id = can_id | CAN_EFF_FLAG;
    std::memset(frame.data, 0, 4);
    write(can_socket_, &frame, sizeof(frame));
  }
  std::fill(state_velocity_.begin(), state_velocity_.end(), 0.0);
  std::fill(cmd_velocity_.begin(), cmd_velocity_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanVescHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (can_socket_ >= 0) {
    struct can_frame frame;
    frame.can_dlc = 4;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      uint32_t can_id = (vesc_ids_[i] & 0xFF) | ((uint32_t)3 << 8);
      frame.can_id = can_id | CAN_EFF_FLAG;
      std::memset(frame.data, 0, 4);
      write(can_socket_, &frame, sizeof(frame));
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanVescHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanVescHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanVescHardwareInterface::on_error(const rclcpp_lifecycle::State &)
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CanVescHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(NUM_JOINTS);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CanVescHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmd_interfaces;
  cmd_interfaces.reserve(NUM_JOINTS);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    cmd_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]));
  }
  return cmd_interfaces;
}

hardware_interface::return_type CanVescHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  struct can_frame frame;
  while (true) {
    ssize_t bytes_read = ::read(can_socket_, &frame, sizeof(frame));
    if (bytes_read < 0) {
      break;
    }
    if (bytes_read < (ssize_t)sizeof(struct can_frame)) {
      continue;
    }
    if (!(frame.can_id & CAN_EFF_FLAG)) {
      continue;
    }
    uint32_t id = frame.can_id & CAN_EFF_MASK;
    uint8_t can_cmd = (id >> 8) & 0xFF;
    uint8_t controller_id = id & 0xFF;
    if (can_cmd == 9) {
      for (size_t i = 0; i < NUM_JOINTS; ++i) {
        if (controller_id == vesc_ids_[i]) {
          int32_t rpm_raw = (frame.data[0] << 24) | (frame.data[1] << 16) |
                             (frame.data[2] << 8)  | (frame.data[3]);
          double erpm = static_cast<double>(rpm_raw);
          double wheel_radps = (erpm / motor_pole_pairs_ / gear_ratio_) * (2 * M_PI / 60.0);
          state_velocity_[i] = wheel_radps;
          break;
        }
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanVescHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  struct can_frame frame;
  frame.can_dlc = 4;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    double wheel_rps = cmd_velocity_[i] / (2 * M_PI);
    double wheel_rpm = wheel_rps * 60.0;
    double motor_rpm = wheel_rpm * gear_ratio_;
    double erpm     = motor_rpm * motor_pole_pairs_;
    int32_t erpm_int = static_cast<int32_t>(erpm);

    uint32_t can_id = (vesc_ids_[i] & 0xFF) | ((uint32_t)3 << 8);
    frame.can_id = can_id | CAN_EFF_FLAG;

    frame.data[0] = (uint8_t)((erpm_int >> 24) & 0xFF);
    frame.data[1] = (uint8_t)((erpm_int >> 16) & 0xFF);
    frame.data[2] = (uint8_t)((erpm_int >> 8) & 0xFF);
    frame.data[3] = (uint8_t)(erpm_int & 0xFF);

    if (::write(can_socket_, &frame, sizeof(frame)) < 0) {
      RCLCPP_ERROR_ONCE(rclcpp::get_logger("CanVescHardwareInterface"),
                        "Failed to send CAN frame to VESC ID %d", vesc_ids_[i]);
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace rover_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover_control::CanVescHardwareInterface, hardware_interface::SystemInterface)
