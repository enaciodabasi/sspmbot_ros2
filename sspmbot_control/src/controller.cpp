/**
 * @file controller.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sspmbot_control/controller.hpp"

using namespace std::chrono_literals;

namespace sspmbot
{
    namespace controller
    {
        Controller::Controller(std::shared_ptr<hardware::Communicator>& communicator_node)
            : Node("sspmbot_amr_controller_node"), m_HardwareCommunicator{communicator_node}
        {
            using namespace std::placeholders;

            RCLCPP_INFO(this->get_logger(), "Creating the SSPMBot controller node.");

            m_Odom = std::make_unique<odometry::Odometry>(m_WheelInfo);

            m_TwistCmdSub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                "/cmd_vel",
                rclcpp::SystemDefaultsQoS(),
                std::bind(
                    &Controller::cmdVel_callback,
                    this,
                    _1
                )
            );
            m_TwistCmdMsgPtr.set(std::make_shared<geometry_msgs::msg::TwistStamped>());

            m_OdomPub = this->create_publisher<nav_msgs::msg::Odometry>(
                "/odom",
                rclcpp::SystemDefaultsQoS()
            );
            m_RtOdomPub = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
                m_OdomPub
            );

            m_OdomTransformPub = this->create_publisher<tf2_msgs::msg::TFMessage>(
                "/tf",
                rclcpp::SystemDefaultsQoS()
            );
            m_RtOdomTransformPub = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
                m_OdomTransformPub
            );

            /* m_UpdateLoop = this->create_wall_timer(
                20ms,
                std::bind(
                    &Controller::update,
                    this
                )
            ); */
        }

        Controller::~Controller()
        {

        }

        void Controller::update()
        {

            rclcpp::Time currentTime = this->get_clock()->now();

            std::shared_ptr<geometry_msgs::msg::TwistStamped> lastVelCmd;
            m_TwistCmdMsgPtr.get(lastVelCmd);
            if(lastVelCmd == nullptr)
            {
                return;
            }


            const auto lastCmdTime = currentTime - lastVelCmd->header.stamp;
            if(lastCmdTime > m_VelCommandTimeout)
            {
                lastVelCmd->twist.linear.x = 0.0;
                lastVelCmd->twist.linear.y = 0.0;
                lastVelCmd->twist.angular.z = 0.0;  
            }

            const double linear_x = lastVelCmd->twist.linear.x;
            const double linear_y = lastVelCmd->twist.linear.y;
            const double angular_z = lastVelCmd->twist.angular.z;

            // Update Odometry.

            // Send transform.

            const kinematics::WheelVelocities wheelVelCmds = kinematics::mecanum::calculate_wheel_velocities(
                linear_x,
                linear_y,
                angular_z,
                m_WheelInfo
            );

            m_HardwareCommunicator->sendVelocityCommandToHW(
              wheelVelCmds 
            );

        }


        void Controller::cmdVel_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> cmd_vel_msg)
        {
            if(cmd_vel_msg->header.stamp.sec == 0 && cmd_vel_msg->header.stamp.nanosec == 0)
            {
                cmd_vel_msg->header.stamp = this->get_clock()->now();
            }

            m_TwistCmdMsgPtr.set(std::move(cmd_vel_msg));
        }

    }
}