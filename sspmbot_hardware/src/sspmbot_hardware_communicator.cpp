/**
 * @file sspmbot_hardware_communicator.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sspmbot_hardware/sspmbot_hardware_communicator.hpp"

namespace sspmbot
{
    namespace hardware
    {
        Communicator::Communicator(const std::string& node_name)
            : rclcpp::Node(node_name)
        {
                using namespace std::placeholders;

                m_WheelVelsSub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                    "hw/amr/hw_wheel_vels",
                    rclcpp::SystemDefaultsQoS(),
                    std::bind(
                        &Communicator::hw_wheel_vels_callback,
                        this,
                        _1
                    )
                );

                m_ImuSub = this->create_subscription<sensor_msgs::msg::Imu>(
                    "hw/amr/imu_data",
                    rclcpp::SensorDataQoS(),
                    std::bind(
                        &Communicator::hw_imu_callback,
                        this,
                        _1
                    )
                );

                m_BatteryStateSub = this->create_subscription<sensor_msgs::msg::BatteryState>(
                    "hw/amr/battery_state",
                    rclcpp::SystemDefaultsQoS(),
                    std::bind(
                        &Communicator::hw_battery_state_callback,
                        this,
                        _1
                    )
                );

                m_WheelVelsPub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "hw/amr/target_wheel_vels",
                    rclcpp::SystemDefaultsQoS()
                );
                m_RtWheelVelsPub = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(m_WheelVelsPub);
                auto& rtWheelVelPubMsg = m_RtWheelVelsPub->msg_;
                rtWheelVelPubMsg.data.resize(4);                
                
        }

        void Communicator::hw_wheel_vels_callback(const std_msgs::msg::Float64MultiArray::SharedPtr hw_wheel_vels_msg)
        {
            
        }

        void Communicator::hw_imu_callback(const sensor_msgs::msg::Imu::SharedPtr hw_imu_msg)
        {

        }

        void Communicator::hw_battery_state_callback(const sensor_msgs::msg::BatteryState::SharedPtr hw_battery_state_msg)
        {

        }
    }
}