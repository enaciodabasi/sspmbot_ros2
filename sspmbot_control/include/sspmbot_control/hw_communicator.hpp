/**
 * @file hardware_communicator.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HARDWARE_COMMUNICATOR_HPP
#define HARDWARE_COMMUNICATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include "amr_kinematics/mecanum_kinematics.hpp"

#include <string>
#include <functional>
#include <memory>

namespace sspmbot
{
    namespace hardware
    {

        class Communicator  : public rclcpp::Node
        {
            public:

            Communicator(const std::string& node_name);
            ~Communicator(){};

            /**
             * @brief Get the latest wheel velocities coming from the hardware.
             * 
             * @return  kinematics::WheelVelocities struct.
             *  
             */
            kinematics::WheelVelocities getWheelVelsFromHW();

            void sendVelocityCommandToHW(const kinematics::WheelVelocities& wheel_vel_commands);
            
            private:

            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_ImuSub = nullptr;
            realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::Imu>> m_ReceivedImuMsgPtr{nullptr};
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_ImuPub = nullptr;

            rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_BatteryStateSub = nullptr;
            realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::BatteryState>> m_ReceivedBatteryStatePtr{nullptr};

            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_WheelVelsSub = nullptr;
            realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64MultiArray>> m_ReceivedWheelVelsPtr{nullptr};

            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_WheelVelsPub = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> m_RtWheelVelsPub;

            /**
             * @brief 
             * 
             * @param hw_wheel_vels_msg: An array of size 4 that contains the wheel velocities, sent from the hardware, in rad/s.
             * Array's first element: Front left wheel velocity
             * Array's second element: Front right wheel velocity
             * Array's third element: Rear left wheel velocity
             * Array's fourth element: Rear right wheel velocity
             */
            void hw_wheel_vels_callback(const std_msgs::msg::Float64MultiArray::SharedPtr hw_wheel_vels_msg);

            void hw_imu_callback(const sensor_msgs::msg::Imu::SharedPtr hw_imu_msg);

            void hw_battery_state_callback(const sensor_msgs::msg::BatteryState::SharedPtr hw_battery_state_msg);

        };

    }
}

#endif // HARDWARE_COMMUNICATOR_HPP