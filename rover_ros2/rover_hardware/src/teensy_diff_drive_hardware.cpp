#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
// (Include any serial port library headers if used, e.g., <serial/serial.h> for the wjwwood serial library)

// Namespace and class definition
namespace rover_hardware
{

class TeensyDiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TeensyDiffDriveHardware)

  // Lifecycle-related methods (on_init, on_configure, on_activate, etc.)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
  {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Parse parameters from the URDF <ros2_control> tag
    device_ = info.hardware_parameters.at("device");
    baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
    timeout_ms_ = std::stoi(info.hardware_parameters.at("timeout_ms"));
    enc_counts_per_rev_ = std::stoi(info.hardware_parameters.at("enc_counts_per_rev"));
    // Joint name parameters (to identify which joints are left/right)
    left_front_joint_ = info.hardware_parameters.at("left_front_wheel_name");
    left_rear_joint_  = info.hardware_parameters.at("left_rear_wheel_name");
    right_front_joint_ = info.hardware_parameters.at("right_front_wheel_name");
    right_rear_joint_  = info.hardware_parameters.at("right_rear_wheel_name");

    // Initialize storage for 4 joints: front_left, rear_left, front_right, rear_right
    pos_.assign(4, 0.0);
    vel_.assign(4, 0.0);
    cmd_.assign(4, 0.0);
    last_pos_.assign(2, 0.0);  // [left_side, right_side] last positions (radians)
    // Offsets for encoders to zero them at startup
    enc_offset_.assign(2, 0);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous*/) override
  {
    // Open the serial connection to the Teensy
    RCLCPP_INFO(rclcpp::get_logger("TeensyHardware"), "Connecting to Teensy on %s...", device_.c_str());
    if (!openSerialPort(device_.c_str(), baud_rate_, timeout_ms_)) {  // openSerialPort: user-defined
      RCLCPP_ERROR(rclcpp::get_logger("TeensyHardware"), "Failed to open serial port");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("TeensyHardware"), "Serial port opened.");
    // Optionally, flush or synchronize initial encoder readings:
    flushSerialInput();
    // Read an initial encoder line to establish baseline (non-blocking)
    int init_left = 0, init_right = 0;
    if (readEncoderCounts(init_left, init_right)) {
      enc_offset_[0] = init_left;
      enc_offset_[1] = init_right;
    } else {
      enc_offset_[0] = enc_offset_[1] = 0;
    }
    last_pos_[0] = last_pos_[1] = 0.0;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous*/) override
  {
    // On activation, ensure motors are stopped initially
    for (double &c : cmd_) { c = 0.0; }
    write(rclcpp::Time(0,0), rclcpp::Duration(0,0));  // send zero speeds
    RCLCPP_INFO(rclcpp::get_logger("TeensyHardware"), "Hardware interface activated.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous*/) override
  {
    // On deactivate, maybe stop the motors and close serial
    for (double &c : cmd_) { c = 0.0; }
    write(rclcpp::Time(0,0), rclcpp::Duration(0,0));  // try to send stop
    closeSerialPort();
    RCLCPP_INFO(rclcpp::get_logger("TeensyHardware"), "Hardware interface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Export state interfaces (position and velocity for each wheel joint)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // We assume joint order: [FL, RL, FR, RR]
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_front_joint_,  hardware_interface::HW_IF_POSITION, &pos_[INDEX_FL]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_front_joint_,  hardware_interface::HW_IF_VELOCITY, &vel_[INDEX_FL]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_rear_joint_,   hardware_interface::HW_IF_POSITION, &pos_[INDEX_RL]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_rear_joint_,   hardware_interface::HW_IF_VELOCITY, &vel_[INDEX_RL]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_front_joint_, hardware_interface::HW_IF_POSITION, &pos_[INDEX_FR]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_front_joint_, hardware_interface::HW_IF_VELOCITY, &vel_[INDEX_FR]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_rear_joint_,  hardware_interface::HW_IF_POSITION, &pos_[INDEX_RR]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_rear_joint_,  hardware_interface::HW_IF_VELOCITY, &vel_[INDEX_RR]));
    return state_interfaces;
  }

  // Export command interfaces (velocity command for each wheel joint)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> cmd_interfaces;
    cmd_interfaces.emplace_back(hardware_interface::CommandInterface(left_front_joint_,  hardware_interface::HW_IF_VELOCITY, &cmd_[INDEX_FL]));
    cmd_interfaces.emplace_back(hardware_interface::CommandInterface(left_rear_joint_,   hardware_interface::HW_IF_VELOCITY, &cmd_[INDEX_RL]));
    cmd_interfaces.emplace_back(hardware_interface::CommandInterface(right_front_joint_, hardware_interface::HW_IF_VELOCITY, &cmd_[INDEX_FR]));
    cmd_interfaces.emplace_back(hardware_interface::CommandInterface(right_rear_joint_,  hardware_interface::HW_IF_VELOCITY, &cmd_[INDEX_RR]));
    return cmd_interfaces;
  }

  // Read from hardware (called periodically by controller manager)
  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) override
  {
    // Ensure serial connection is alive
    if (!serialPortIsOpen()) {
      RCLCPP_ERROR(rclcpp::get_logger("TeensyHardware"), "Serial connection lost!");
      return hardware_interface::return_type::ERROR;
    }
    // Try to read encoder counts from Teensy
    int counts_left = 0, counts_right = 0;
    bool got_data = readEncoderCounts(counts_left, counts_right);
    if (!got_data) {
      // If no new data was available this cycle, we could choose to reuse last values or return OK.
      // For robust design, don't treat it as an error unless persistent.
      // (We assume Teensy sends at a steady rate; minor misses are okay.)
    }

    // Convert counts to wheel rotation (radians) using encoder resolution
    double left_pos_rad = (counts_left - enc_offset_[0]) * (2 * M_PI / enc_counts_per_rev_);
    double right_pos_rad = (counts_right - enc_offset_[1]) * (2 * M_PI / enc_counts_per_rev_);
    // Compute velocities (rad/s) based on position change in this period
    double dt = period.seconds();
    if (dt > 0) {
      double left_vel_rad = (left_pos_rad - last_pos_[0]) / dt;
      double right_vel_rad = (right_pos_rad - last_pos_[1]) / dt;
      // Update last position for next velocity calculation
      last_pos_[0] = left_pos_rad;
      last_pos_[1] = right_pos_rad;
      // Set joint state values for both wheels on each side
      pos_[INDEX_FL] = pos_[INDEX_RL] = left_pos_rad;
      vel_[INDEX_FL] = vel_[INDEX_RL] = left_vel_rad;
      pos_[INDEX_FR] = pos_[INDEX_RR] = right_pos_rad;
      vel_[INDEX_FR] = vel_[INDEX_RR] = right_vel_rad;
    }

    return hardware_interface::return_type::OK;
  }

  // Write commands to hardware (called periodically by controller manager)
  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (!serialPortIsOpen()) {
      return hardware_interface::return_type::ERROR;
    }
    // The diff_drive_controller will set the same velocity on both wheels of each side:contentReference[oaicite:8]{index=8}.
    // We combine the left and right commands (here we just average front and rear, in case of tiny differences).
    double left_cmd = 0.5 * (cmd_[INDEX_FL] + cmd_[INDEX_RL]);
    double right_cmd = 0.5 * (cmd_[INDEX_FR] + cmd_[INDEX_RR]);
    // Send the two commands to Teensy as text
    char out[32];
    snprintf(out, sizeof(out), "%.3f %.3f\n", left_cmd, right_cmd);
    size_t bytes_written = serialWriteBytes(out, strlen(out));
    if (bytes_written <= 0) {
      RCLCPP_WARN(rclcpp::get_logger("TeensyHardware"), "Failed to write to serial");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

private:
  // Indices for convenience
  static constexpr size_t INDEX_FL = 0;
  static constexpr size_t INDEX_RL = 1;
  static constexpr size_t INDEX_FR = 2;
  static constexpr size_t INDEX_RR = 3;

  // Joint names (from parameters)
  std::string left_front_joint_, left_rear_joint_;
  std::string right_front_joint_, right_rear_joint_;

  // Hardware parameters
  std::string device_;
  int baud_rate_ = 115200;
  int timeout_ms_ = 1000;
  int enc_counts_per_rev_ = 0;

  // State and command storage
  std::vector<double> pos_;   // positions [FL, RL, FR, RR]
  std::vector<double> vel_;   // velocities [FL, RL, FR, RR]
  std::vector<double> cmd_;   // commands  [FL, RL, FR, RR]

  // Last position (for velocity calc) and encoder offset for left/right drives
  std::vector<double> last_pos_;    // [left_side, right_side] last positions (rad)
  std::vector<long> enc_offset_;   // [left_side, right_side] initial encoder counts offset

  // Serial port handle and helper methods (pseudo-code placeholders)
  // In practice, use an actual serial library or system calls.
  int serial_fd_ = -1;
  bool openSerialPort(const char* device, int baud, int timeout_ms) {
    // TODO: open serial_fd_ with termios or serial::Serial, set baud and timeout.
    // Return true if success.
    serial_fd_ = ::open(device, O_RDWR | O_NOCTTY);
    // ... configure baud with termios (or use a library) ...
    return serial_fd_ != -1;
  }
  void closeSerialPort() {
    if (serial_fd_ != -1) { ::close(serial_fd_); serial_fd_ = -1; }
  }
  bool serialPortIsOpen() const { return serial_fd_ != -1; }
  size_t serialWriteBytes(const char* data, size_t len) {
    if (serial_fd_ == -1) return 0;
    return ::write(serial_fd_, data, len);
  }
  bool readEncoderCounts(int &left_count, int &right_count) {
    // Non-blocking read of a line "L R\n" from serial into left_count and right_count.
    // You can use read() and buffer until newline, then sscanf to parse.
    char buf[64];
    ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
    if (n <= 0) {
      return false;
    }
    buf[n] = '\0';
    // Find newline
    char *newline = strchr(buf, '\n');
    if (!newline) {
      // If no newline yet, you might need to accumulate across calls.
      // For brevity, assume we got a full line in one read.
      return false;
    }
    *newline = '\0';
    // Parse two integers
    long l = 0, r = 0;
    int parsed = sscanf(buf, "%ld %ld", &l, &r);
    if (parsed == 2) {
      left_count = static_cast<int>(l);
      right_count = static_cast<int>(r);
      return true;
    }
    return false;
  }
  void flushSerialInput() {
    // Clear any buffered data (e.g. using tcflush or reading until empty)
    if (serial_fd_ != -1) {
      tcflush(serial_fd_, TCIFLUSH);
    }
  }
};  // class

}  // namespace teensy_diff_drive

// Export the plugin
PLUGINLIB_EXPORT_CLASS(teensy_diff_drive::TeensyDiffDriveHardware, hardware_interface::SystemInterface)
