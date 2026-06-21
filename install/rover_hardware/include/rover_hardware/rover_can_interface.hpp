#ifndef ROVER_HARDWARE__ROVER_CAN_INTERFACE_HPP_
#define ROVER_HARDWARE__ROVER_CAN_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"

// Linux SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace rover_hardware
{

class RoverCANInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverCANInterface)

  // Lifecycle Node Interface
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Hardware Interface
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- Parameters ---
  std::string can_interface_name_ = "can0";
  int can_id_left_cmd_ = 0x10;   // Send Command to Left
  int can_id_right_cmd_ = 0x11;  // Send Command to Right
  
  // ESP32 sends FEEDBACK on these IDs
  int can_id_left_fb_ = 0x20;    
  int can_id_right_fb_ = 0x21;

  // --- Communication ---
  int socket_can_fd_ = -1;
  
  // --- Data Storage ---
  // Order for Differential Drive Controller usually: FL, RL, FR, RR
  // Note: We mirror FL->RL and FR->RR because we only have 2 real controllers
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace rover_hardware

#endif  // ROVER_HARDWARE__ROVER_CAN_INTERFACE_HPP_