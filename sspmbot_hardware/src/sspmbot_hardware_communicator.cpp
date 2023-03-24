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
                
                m_ReceivedImuMsgPtr.set(std::make_shared<sensor_msgs::msg::Imu>());

                m_ImuPub = this->create_publisher<sensor_msgs::msg::Imu>(
                    "/amr/imu_data",
                    rclcpp::SensorDataQoS()
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

                m_ReceivedBatteryStatePtr.set(std::make_shared<sensor_msgs::msg::BatteryState>());

                m_WheelVelsPub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "hw/amr/target_wheel_vels",
                    rclcpp::SystemDefaultsQoS()
                );
                m_RtWheelVelsPub = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(m_WheelVelsPub);
                auto& rtWheelVelPubMsg = m_RtWheelVelsPub->msg_;
                rtWheelVelPubMsg.data.resize(4);                
                
        }

        kinematics::WheelVelocities Communicator::getWheelVelsFromHW()
        {
            std::shared_ptr<std_msgs::msg::Float64MultiArray> hwVels;
            m_ReceivedWheelVelsPtr.get(hwVels);
            kinematics::WheelVelocities wheelVels;
            if(hwVels->data.size() != 4)
            {
                wheelVels.state = kinematics::WheelVelocityState::ERROR;
                return wheelVels;   
            }
            
            wheelVels.frontLeft = hwVels->data[0];
            wheelVels.frontRight = hwVels->data[1];
            wheelVels.rearLeft = hwVels->data[2];
            wheelVels.rearRight = hwVels->data[3];
            wheelVels.state = kinematics::WheelVelocityState::OK;
            
            return wheelVels;
        }

        void Communicator::sendVelocityCommandToHW(const kinematics::WheelVelocities& wheel_vel_commands)
        {
            std_msgs::msg::Float64MultiArray wheelCmds;
            wheelCmds.data.reserve(4);
            wheelCmds.data[0] = wheel_vel_commands.frontLeft;
            wheelCmds.data[1] = wheel_vel_commands.frontRight;
            wheelCmds.data[2] = wheel_vel_commands.rearLeft;
            wheelCmds.data[3] = wheel_vel_commands.rearRight;
            if(m_RtWheelVelsPub->trylock())
            {
                auto& rtMsg = m_RtWheelVelsPub->msg_;
                rtMsg = wheelCmds;
                m_RtWheelVelsPub->unlockAndPublish();
            }
        }

        void Communicator::hw_wheel_vels_callback(const std_msgs::msg::Float64MultiArray::SharedPtr hw_wheel_vels_msg)
        {
            m_ReceivedWheelVelsPtr.set(std::move(hw_wheel_vels_msg));
        }

        void Communicator::hw_imu_callback(const sensor_msgs::msg::Imu::SharedPtr hw_imu_msg)
        {
            m_ReceivedImuMsgPtr.set(std::move(hw_imu_msg));
        }

        void Communicator::hw_battery_state_callback(const sensor_msgs::msg::BatteryState::SharedPtr hw_battery_state_msg)
        {
            m_ReceivedBatteryStatePtr.set(std::move(hw_battery_state_msg));
        }
    }
}