/**
 * @file mecanum_drive_controller.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "mecanum_drive_controller/mecanum_drive_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace mecanum_drive_controller
{


    controller_interface::InterfaceConfiguration MecanumDriveController::command_interface_configuration() const
    {
        
    }
    controller_interface::InterfaceConfiguration MecanumDriveController::state_interface_configuration() const
    {
        
    }
    controller_interface::return_type MecanumDriveController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        
    }
    controller_interface::CallbackReturn MecanumDriveController::on_init()
    {
        
    }
    controller_interface::CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        
    }
    controller_interface::CallbackReturn MecanumDriveController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        
    }
    controller_interface::CallbackReturn MecanumDriveController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        
    }
    controller_interface::CallbackReturn MecanumDriveController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        
    }
    controller_interface::CallbackReturn MecanumDriveController::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        
    }
    controller_interface::CallbackReturn MecanumDriveController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        
    }

    std::unique_ptr<WheelHandle> MecanumDriveController::getWheelHandle(const std::string& joint_name)
    {
        const auto positionState = std::find_if(
            state_interfaces_.cbegin(),
            state_interfaces_.cend(),
            [&joint_name](const hardware_interface::LoanedStateInterface& loaned_state_interface){
                return loaned_state_interface.get_name() == joint_name && loaned_state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
            }
        );
        
        if(positionState == state_interfaces_.cend())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), 
                "Position state interface not found for joint: %s", 
                joint_name.c_str()
            );
            return nullptr;
        }

        const auto velocityState = std::find_if(
            state_interfaces_.cbegin(),
            state_interfaces_.cend(),
            [&joint_name](const hardware_interface::LoanedStateInterface& loaned_state_interface){
                return loaned_state_interface.get_name() == joint_name && loaned_state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
            }
        );

        if(velocityState == state_interfaces_.cend())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), 
                "Velocity state interface not found for joint: %s", 
                joint_name.c_str()
            );
            return nullptr;
        }
        
        const auto velCommandInterface = std::find_if(
            command_interfaces_.begin(),
            command_interfaces_.end(),
            [&joint_name](hardware_interface::LoanedCommandInterface& loaned_command_interface){
                return loaned_command_interface.get_name() == joint_name && loaned_command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
            }
        );

        if(velCommandInterface == command_interfaces_.end())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), 
                "Velocity command interface not found for joint: %s", 
                joint_name.c_str()
            );
            
            return nullptr;
        }

        return std::make_unique<WheelHandle>(
            std::ref(*positionState),
            std::ref(*velocityState),
            std::ref(*velCommandInterface)
        );
    }
}