#include "bearmax_hardware/bearmax_arduino.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bearmax_hardware
{
    // Used for array indexes! Don't change numbers!
    enum Joint {
        CHASSIS = 0,
        L_ARM_SHOULDER,
        L_ARM_ROTATOR,
        L_ARM_ELBOW,
        L_ARM_GRIP,
        L_ARM_THUMB,
        L_ARM_PAW,
        R_ARM_SHOULDER,
        R_ARM_ROTATOR,
        R_ARM_ELBOW,
        R_ARM_GRIP,
        R_ARM_THUMB,
        R_ARM_PAW,
        L_HEAD,
        R_HEAD,
        HEAD_YAW,
        NUMBER_OF_JOINTS /* not a valid index! */
    };

#define CMD_INTERFACE(X,Y) command_interfaces.emplace_back( hardware_interface::CommandInterface(X, hardware_interface::HW_IF_POSITION, &hw_servo_cmds_[Y]))
#define STATE_INTERFACE(X,Y) state_interfaces.emplace_back( hardware_interface::StateInterface(X, hardware_interface::HW_IF_POSITION, &hw_servo_states_[Y]))


hardware_interface::CallbackReturn BearmaxArduino::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Initializing...");

    // Get parameters
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);

    hw_servo_states_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    hw_servo_cmds_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Setup Arduino
    arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

    if (arduino_.connected()) {
        RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Already connected!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Not yet connected!");
    }

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Finished Initialization");

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BearmaxArduino::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

/*
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_servo_states_[i]
            )
        );
    }
    */

    STATE_INTERFACE("chassis_joint", Joint::CHASSIS);
    STATE_INTERFACE("left_shoulder_joint", Joint::L_ARM_SHOULDER);
    STATE_INTERFACE("left_rotator_joint", Joint::L_ARM_ROTATOR);
    STATE_INTERFACE("left_elbow_joint", Joint::L_ARM_ELBOW);
    STATE_INTERFACE("left_grip_joint", Joint::L_ARM_GRIP);
    STATE_INTERFACE("left_thumb_joint", Joint::L_ARM_THUMB);
    STATE_INTERFACE("left_paw_joint", Joint::L_ARM_PAW);
    STATE_INTERFACE("right_shoulder_joint", Joint::R_ARM_SHOULDER);
    STATE_INTERFACE("right_rotator_joint", Joint::R_ARM_ROTATOR);
    STATE_INTERFACE("right_elbow_joint", Joint::R_ARM_ELBOW);
    STATE_INTERFACE("right_grip_joint", Joint::R_ARM_GRIP);
    STATE_INTERFACE("right_thumb_joint", Joint::R_ARM_THUMB);
    STATE_INTERFACE("right_paw_joint", Joint::R_ARM_PAW);
    STATE_INTERFACE("left_neck_joint", Joint::L_HEAD);
    STATE_INTERFACE("right_neck_joint", Joint::R_HEAD);
    STATE_INTERFACE("neck_yaw_joint", Joint::HEAD_YAW);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BearmaxArduino::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    /*
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_servo_cmds_[i]
            )
        );
    }
    */

    CMD_INTERFACE("chassis_joint", Joint::CHASSIS);
    CMD_INTERFACE("left_shoulder_joint", Joint::L_ARM_SHOULDER);
    CMD_INTERFACE("left_rotator_joint", Joint::L_ARM_ROTATOR);
    CMD_INTERFACE("left_elbow_joint", Joint::L_ARM_ELBOW);
    CMD_INTERFACE("left_grip_joint", Joint::L_ARM_GRIP);
    CMD_INTERFACE("left_thumb_joint", Joint::L_ARM_THUMB);
    CMD_INTERFACE("left_paw_joint", Joint::L_ARM_PAW);
    CMD_INTERFACE("right_shoulder_joint", Joint::R_ARM_SHOULDER);
    CMD_INTERFACE("right_rotator_joint", Joint::R_ARM_ROTATOR);
    CMD_INTERFACE("right_elbow_joint", Joint::R_ARM_ELBOW);
    CMD_INTERFACE("right_grip_joint", Joint::R_ARM_GRIP);
    CMD_INTERFACE("right_thumb_joint", Joint::R_ARM_THUMB);
    CMD_INTERFACE("right_paw_joint", Joint::R_ARM_PAW);
    CMD_INTERFACE("left_neck_joint", Joint::L_HEAD);
    CMD_INTERFACE("right_neck_joint", Joint::R_HEAD);
    CMD_INTERFACE("neck_yaw_joint", Joint::HEAD_YAW);

    return command_interfaces;
}

hardware_interface::CallbackReturn BearmaxArduino::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!arduino_.connected()) {
        arduino_.connect();
    }

    /*using namespace std::this_thread;
    using namespace std::chrono;

    sleep_for(nanoseconds(5*(1000000000)));*/

    // TODO: Maybe add an 'activate' command to arduino to power on the servos?

    // Set default values.
    for (auto i = 0u; i < hw_servo_states_.size(); i++) {
        if (std::isnan(hw_servo_states_[i])) {
            hw_servo_states_[i] = 0;
            hw_servo_cmds_[i] = 0;
        }
    }
    /*
    std::vector<uint8_t> res;

    std::stringstream ss;

    ss << res.size() << " ";

    size_t n = arduino_.testRead(res);

    ss << n << " " << res.size();

    std::string sService = ss.str();

    std::string res_str(res.begin(), res.end());

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Reading Servo Values!");
    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), sService.c_str());
    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), res_str.c_str());
    */
    

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearmaxArduino::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (arduino_.connected()) {
        arduino_.disconnect();
    }

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Deactivated!");

    // TODO: Maybe add an 'deactivate' command to arduino to power off the servos?

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BearmaxArduino::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!arduino_.connected()) {
        return hardware_interface::return_type::ERROR;
    }

    //RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Reading Servo Values!");

    //arduino_.getServoValues(hw_servo_states_);

    for (uint i = 0; i < hw_servo_states_.size(); i++) {
        hw_servo_states_[i] = hw_servo_cmds_[i];

    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type BearmaxArduino::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!arduino_.connected()) {
        return hardware_interface::return_type::ERROR;
    }

    //RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Writing Servo Values!");

    std::string res = arduino_.setServoValues(hw_servo_cmds_);

    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bearmax_hardware::BearmaxArduino,
    hardware_interface::SystemInterface)

