#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>

namespace rover_control
{

class RERASSORHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;

    // Fetch the needed information from the Xacros file passed as info
    front_left_joint_name_ = info_.joints[0].name;
    front_right_joint_name_ = info_.joints[1].name;
    back_left_joint_name_ = info_.joints[2].name;
    back_right_joint_name_ = info_.joints[3].name;
    device_ = info_.hardware_parameters["device"];
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

    // Create variables to use later
    fl_pos_ = fr_pos_ = bl_pos_ = br_pos_ = 0.0;
    fl_vel_ = fr_vel_ = bl_vel_ = br_vel_ = 0.0;
    fl_cmd_ = fr_cmd_ = bl_cmd_ = br_cmd_ = 0.0;

    // Open the serial port
    serial_fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("RERASSORHardwareInterface"), "Failed to open serial port: %s", device_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Create a terminal
    struct termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) return hardware_interface::CallbackReturn::ERROR; // Read current serial port settings (error out if failed)
    cfsetospeed(&tty, baud_rate_); // Set input and output speeds
    cfsetispeed(&tty, baud_rate_);
    tty.c_cflag |= (CLOCAL | CREAD | CS8); // Configure hardware pin flags
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tcsetattr(serial_fd_, TCSANOW, &tty);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    return {
      hardware_interface::StateInterface(front_left_joint_name_, hardware_interface::HW_IF_POSITION, &fl_pos_),
      hardware_interface::StateInterface(front_left_joint_name_, hardware_interface::HW_IF_VELOCITY, &fl_vel_),
      hardware_interface::StateInterface(front_right_joint_name_, hardware_interface::HW_IF_POSITION, &fr_pos_),
      hardware_interface::StateInterface(front_right_joint_name_, hardware_interface::HW_IF_VELOCITY, &fr_vel_),
      hardware_interface::StateInterface(back_left_joint_name_, hardware_interface::HW_IF_POSITION, &bl_pos_),
      hardware_interface::StateInterface(back_left_joint_name_, hardware_interface::HW_IF_VELOCITY, &bl_vel_),
      hardware_interface::StateInterface(back_right_joint_name_, hardware_interface::HW_IF_POSITION, &br_pos_),
      hardware_interface::StateInterface(back_right_joint_name_, hardware_interface::HW_IF_VELOCITY, &br_vel_)
    };
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    return {
      hardware_interface::CommandInterface(front_left_joint_name_, hardware_interface::HW_IF_VELOCITY, &fl_cmd_),
      hardware_interface::CommandInterface(front_right_joint_name_, hardware_interface::HW_IF_VELOCITY, &fr_cmd_),
      hardware_interface::CommandInterface(back_left_joint_name_, hardware_interface::HW_IF_VELOCITY, &bl_cmd_),
      hardware_interface::CommandInterface(back_right_joint_name_, hardware_interface::HW_IF_VELOCITY, &br_cmd_)
    };
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // Encoder feedback could go here
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::string cmd = "M " + std::to_string(left_command_) + " " + std::to_string(right_command_) + "\n";
    write(serial_fd_, cmd.c_str(), cmd.size());
    return hardware_interface::return_type::OK;
  }

private:
  std::string front_left_joint_name_, front_right_joint_name_, back_left_joint_name_, back_right_joint_name_;
  double fl_pos_, fl_vel_, fl_cmd_;
  double fr_pos_, fr_vel_, fr_cmd_;
  double bl_pos_, bl_vel_, bl_cmd_;
  double br_pos_, br_vel_, br_cmd_;
  std::string device_;
  int baud_rate_, serial_fd_;
};

}  // namespace rover_control

PLUGINLIB_EXPORT_CLASS(rover_control::RERASSORHardwareInterface, hardware_interface::SystemInterface)
