/**
 * @file sspmbot_system.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SSPMBOT_HARDWARE__SSPMBOT_SYSTEM_HPP
#define SSPMBOT_HARDWARE__SSPMBOT_SYSTEM_HPP

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "visibility_control.h"
#include "sspmbot_hardware_communicator.hpp"
#include "amr_kinematics/mecanum_kinematics.hpp"

namespace sspmbot
{

    namespace hardware
    {
        class SSPmBotHardware : public hardware_interface::SystemInterface
        {
            public:

            RCLCPP_SHARED_PTR_DEFINITIONS(SSPmBotHardware);

            SSPMBOT_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

            SSPMBOT_HARDWARE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            SSPMBOT_HARDWARE_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            SSPMBOT_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

            SSPMBOT_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            SSPMBOT_HARDWARE_PUBLIC
            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

            SSPMBOT_HARDWARE_PUBLIC
            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

            private:

            std::vector<double> m_HardwareCommands;
            std::vector<double> m_JointPositionStates;
            std::vector<double> m_JointVelocityStates;

            rclcpp::executors::MultiThreadedExecutor m_Executor;

            std::unique_ptr<std::thread> m_ExecutorThread;

            std::shared_ptr<Communicator> m_HardwareCommunicator;


        };
    }
    
}

#endif // SSPMBOT_HARDWARE__SSPMBOT_SYSTEM_HPP