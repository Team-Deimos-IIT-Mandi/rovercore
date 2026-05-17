#include "rover_hardware/rover_can_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <cmath>

namespace rover_hardware
{

// --- 4-WHEEL BINARY PROTOCOL ---

// 18 Bytes
struct CommandPacket {
    uint8_t header = 0xA5;
    float fl_vel;
    float rl_vel;
    float fr_vel;
    float rr_vel;
    uint8_t terminator = 0x5A;
} __attribute__((packed));

// 34 Bytes
struct FeedbackPacket {
    uint8_t header = 0xA5;
    float fl_pos;
    float fl_vel;
    float rl_pos;
    float rl_vel;
    float fr_pos;
    float fr_vel;
    float rr_pos;
    float rr_vel;
    uint8_t terminator = 0x5A;
} __attribute__((packed));

class RoverSerialInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        // Reserve memory for 4 joints
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        for (const auto & hardware_param : info_.hardware_parameters) {
            if (hardware_param.first == "serial_port") {
                serial_port_name_ = hardware_param.second;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(rclcpp::get_logger("RoverSerial"), "Failed to open %s: %s", serial_port_name_.c_str(), strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return hardware_interface::CallbackReturn::ERROR;

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; 
        tty.c_iflag &= ~IGNBRK;                     
        tty.c_lflag = 0;                            
        tty.c_oflag = 0;                            
        tty.c_cc[VMIN]  = 0;                        
        tty.c_cc[VTIME] = 1;                        

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);     
        tty.c_cflag |= (CLOCAL | CREAD);            
        tty.c_cflag &= ~(PARENB | PARODD);          
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return hardware_interface::CallbackReturn::ERROR;

        RCLCPP_INFO(rclcpp::get_logger("RoverSerial"), "4-Wheel Serial Interface Configured on %s", serial_port_name_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        if (serial_fd_ >= 0) close(serial_fd_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        uint8_t buffer[128]; // Increased buffer size
        int n = ::read(serial_fd_, buffer, sizeof(buffer));
        
        if (n >= (int)sizeof(FeedbackPacket)) {
            // Scan buffer for valid packet
            for (int i = 0; i <= n - (int)sizeof(FeedbackPacket); i++) {
                if (buffer[i] == 0xA5 && buffer[i + sizeof(FeedbackPacket) - 1] == 0x5A) {
                    FeedbackPacket fb;
                    std::memcpy(&fb, &buffer[i], sizeof(fb));

                    // ESP32 fl/rl slots wire to physical RIGHT motors, fr/rr to physical LEFT.
                    // Joint order: LF(0), LB(1), RF(2), RB(3).

                    // LF joint (physical left) ← fr feedback
                    hw_positions_[0] = (double)fb.fr_pos;
                    hw_velocities_[0] = (double)fb.fr_vel;

                    // LB joint (physical left) ← rr feedback
                    hw_positions_[1] = (double)fb.rr_pos;
                    hw_velocities_[1] = (double)fb.rr_vel;

                    // RF joint (physical right) ← fl feedback
                    hw_positions_[2] = (double)fb.fl_pos;
                    hw_velocities_[2] = (double)fb.fl_vel;

                    // RB joint (physical right) ← rl feedback
                    hw_positions_[3] = (double)fb.rl_pos;
                    hw_velocities_[3] = (double)fb.rl_vel;
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        CommandPacket cmd;
        cmd.header = 0xA5;
        
        // ESP32 fl/rl slots wire to physical RIGHT motors, fr/rr to physical LEFT.
        // LF(0)/LB(1) are left controller joints → send to fr/rr (physical left).
        // RF(2)/RB(3) are right controller joints → send to fl/rl (physical right).
        cmd.fr_vel = -(float)hw_commands_[0]; // LF → physical left front
        cmd.rr_vel = -(float)hw_commands_[1]; // LB → physical left rear
        cmd.fl_vel = -(float)hw_commands_[2]; // RF → physical right front
        cmd.rl_vel = -(float)hw_commands_[3]; // RB → physical right rear
        
        cmd.terminator = 0x5A;

        ::write(serial_fd_, &cmd, sizeof(cmd));
        return hardware_interface::return_type::OK;
    }

private:
    std::string serial_port_name_ = "/dev/ttyUSB0";
    int serial_fd_ = -1;
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
};

} // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(
  rover_hardware::RoverSerialInterface,
  hardware_interface::SystemInterface
)