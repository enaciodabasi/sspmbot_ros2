/**
 * @file sspmbot_system.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sspmbot_hardware/sspmbot_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace sspmbot
{
    namespace hardware
    {
        hardware_interface::CallbackReturn sspmbot::hardware::SSPmBotHardware::on_init(const hardware_interface::HardwareInfo &info)
        {
            if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            {
                return hardware_interface::CallbackReturn::ERROR;
            }

            m_HardwarePositionStates.resize(
                info_.joints.size(),
                std::numeric_limits<double>::quiet_NaN()
            );

            m_HardwareVelocityStates.resize(
                info_.joints.size(),
                std::numeric_limits<double>::quiet_NaN()
            );

            m_HardwareCommands.resize(
                info_.joints.size(),
                std::numeric_limits<double>::quiet_NaN()
            );

            for(hardware_interface::ComponentInfo& joint : info_.joints)
            {
                if(joint.command_interfaces.size() != 1)
                {
                    RCLCPP_FATAL(rclcpp::get_logger("SSPmBotHardware"), "Invalid number of command interfaces (expected 1), got %ld", joint.command_interfaces.size());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                if(joint.state_interfaces.size() != 2)
                {
                    RCLCPP_FATAL(rclcpp::get_logger("SSPmBotHardware"), "Invalid number of joint interfaces (expected 2), got %ld", joint.state_interfaces.size());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                if(joint.command_interfaces.at(0).name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(rclcpp::get_logger("SSPmBotHardware"), "Invalid joint command interface type (expected velocity), got %s", joint.command_interfaces.at(0).name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) 
                {
                    RCLCPP_FATAL(rclcpp::get_logger("SSPmBotHardware"), "Invalid joint state interface 0 type (expected: position), got %s", joint.state_interfaces.at(0).name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(rclcpp::get_logger("SSPmBotHardware"), "Invalid joint state interface 1 type (expected: velocity), got %s", joint.state_interfaces.at(1).name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Successfully initalized hardware interface.");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        std::vector<hardware_interface::StateInterface> SSPmBotHardware::export_state_interfaces()
        {
            std::vector<hardware_interface::StateInterface> stateInterfaces;
            RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Exporting state interfaces.");
            for(std::size_t i = 0; i < info_.joints.size(); i++)
            {
                RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Adding position and velocity state interfaces: %s", info_.joints[i].name.c_str());
                stateInterfaces.emplace_back(
                    hardware_interface::StateInterface(
                        info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION,
                        &m_HardwarePositionStates[i]
                    )
                );
                stateInterfaces.emplace_back(
                    hardware_interface::StateInterface(
                        info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY,
                        &m_HardwareVelocityStates[i]
                    )
                );
            }
            RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Successfully exported state interfaces.");
            return stateInterfaces;
        }

        std::vector<hardware_interface::CommandInterface> SSPmBotHardware::export_command_interfaces()
        {
            RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Exporting command interfaces.");
            
            std::vector<hardware_interface::CommandInterface> commandInterfaces;
            for(std::size_t i = 0; i < info_.joints.size(); i++)
            {
                RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Adding velocity command interfaces: %s", info_.joints[i].name.c_str());
                commandInterfaces.emplace_back(
                    hardware_interface::CommandInterface(
                        info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY,
                        &m_HardwareCommands[i]
                    )
                );
            }
            RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Successfully exported command interfaces.");
            return commandInterfaces;
        }

        hardware_interface::CallbackReturn SSPmBotHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
        {
            RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Activating hardware interface...");

            for(std::size_t i = 0; i < info_.joints.size(); i++)
            {
                if(std::isnan(m_HardwarePositionStates[i]))
                {
                    m_HardwarePositionStates[i] = 0.0;
                }

                if(std::isnan(m_HardwareVelocityStates[i]))
                {
                    m_HardwareVelocityStates[i] = 0.0;
                }

                if(std::isnan(m_HardwareCommands[i]))
                {
                    m_HardwareCommands[i] = 0.0;
                }
            }

            // Create hardware communicator

            RCLCPP_INFO(rclcpp::get_logger("SSPmBotHardware"), "Hardware interface started.");
            
            return hardware_interface::CallbackReturn::SUCCESS;

        }

        hardware_interface::CallbackReturn SSPmBotHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
        {
            
        }

        hardware_interface::return_type SSPmBotHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
        {
            
        }

        hardware_interface::return_type SSPmBotHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
        {
            
        }
    }
}


PLUGINLIB_EXPORT_CLASS(
    sspmbot::hardware::SSPmBotHardware,
    hardware_interface::SystemInterface
)