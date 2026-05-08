#include "rover_hardware/rover_can_interface.hpp"

// --- SYSTEM HEADERS (Fixes fcntl, read, write errors) ---
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

// --- ROS 2 HEADERS (Fixes HW_IF_... errors) ---
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace rover_hardware
{

hardware_interface::CallbackReturn RoverCANInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Expecting 4 joints (FL, RL, FR, RR)
  // Initialize with NaN to ensure safety before we get real data
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Read parameters from URDF
  for (const auto & hardware_param : info_.hardware_parameters) {
      if (hardware_param.first == "can_interface") {
          can_interface_name_ = hardware_param.second;
      }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverCANInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Setup SocketCAN
  struct sockaddr_can addr;
  struct ifreq ifr;

  if ((socket_can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("RoverCANInterface"), "Socket creation failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Copy interface name (e.g., "can0") to ifreq structure
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_name_.c_str(), IFNAMSIZ - 1);
  
  if (ioctl(socket_can_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("RoverCANInterface"), "Interface setup failed: %s. Is the interface up?", can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_can_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("RoverCANInterface"), "Bind failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set non-blocking mode (fixes the fcntl/O_NONBLOCK error)
  int flags = fcntl(socket_can_fd_, F_GETFL, 0);
  fcntl(socket_can_fd_, F_SETFL, flags | O_NONBLOCK);

  RCLCPP_INFO(rclcpp::get_logger("RoverCANInterface"), "CAN Interface Configured on %s", can_interface_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverCANInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (socket_can_fd_ >= 0) {
    close(socket_can_fd_);
    socket_can_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverCANInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands/states to safe values upon activation
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverCANInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverCANInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // Fixes HW_IF_POSITION/VELOCITY error
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverCANInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // Fixes HW_IF_VELOCITY error
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type RoverCANInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  struct can_frame frame;
  
  // Non-blocking read loop
  while (::read(socket_can_fd_, &frame, sizeof(struct can_frame)) > 0)
  {
    // Payload unpacking (Assumes ESP32 sent [float pos | float vel])
    float pos_f = 0.0f;
    float vel_f = 0.0f;
    
    if (frame.can_dlc >= 8) {
      std::memcpy(&pos_f, &frame.data[0], 4);
      std::memcpy(&vel_f, &frame.data[4], 4);
    }

    // Cast IDs to unsigned (canid_t) to fix signed comparison warnings
    if (frame.can_id == static_cast<canid_t>(can_id_left_fb_)) {
      // LEFT SIDE: Indices 0 (Front) and 1 (Rear)
      hw_positions_[0] = static_cast<double>(pos_f);
      hw_velocities_[0] = static_cast<double>(vel_f);
      
      hw_positions_[1] = static_cast<double>(pos_f);
      hw_velocities_[1] = static_cast<double>(vel_f);
    }
    else if (frame.can_id == static_cast<canid_t>(can_id_right_fb_)) {
      // RIGHT SIDE: Indices 2 (Front) and 3 (Rear)
      hw_positions_[2] = static_cast<double>(pos_f);
      hw_velocities_[2] = static_cast<double>(vel_f);
      
      hw_positions_[3] = static_cast<double>(pos_f);
      hw_velocities_[3] = static_cast<double>(vel_f);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverCANInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  struct can_frame frame_left, frame_right;
  
  // COMMAND LEFT (Use Front Left command)
  float left_cmd_f = static_cast<float>(hw_commands_[0]);
  
  frame_left.can_id = can_id_left_cmd_;
  frame_left.can_dlc = 4; // Only sending 1 float (Velocity)
  std::memset(frame_left.data, 0, 8);
  std::memcpy(&frame_left.data[0], &left_cmd_f, 4);

  // COMMAND RIGHT (Use Front Right command)
  float right_cmd_f = static_cast<float>(hw_commands_[2]);

  frame_right.can_id = can_id_right_cmd_;
  frame_right.can_dlc = 4;
  std::memset(frame_right.data, 0, 8);
  std::memcpy(&frame_right.data[0], &right_cmd_f, 4);

  // Send to CAN bus
  ::write(socket_can_fd_, &frame_left, sizeof(struct can_frame));
  ::write(socket_can_fd_, &frame_right, sizeof(struct can_frame));

  return hardware_interface::return_type::OK;
}

} // namespace rover_hardware