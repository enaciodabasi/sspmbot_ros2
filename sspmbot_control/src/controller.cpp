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
            auto& odomMsg = m_RtOdomPub->msg_;
            odomMsg.pose.pose.position.z = 0.0;
            odomMsg.twist.twist.linear.z = 0.0;
            odomMsg.twist.twist.angular.x = 0.0;
            odomMsg.twist.twist.angular.y = 0.0;

            m_OdomTransformPub = this->create_publisher<tf2_msgs::msg::TFMessage>(
                "/tf",
                rclcpp::SystemDefaultsQoS()
            );
            m_RtOdomTransformPub = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
                m_OdomTransformPub
            );
            auto& odomTransformMsgs = m_RtOdomTransformPub->msg_;
            odomTransformMsgs.transforms.resize(1);
            auto& odomTransformMsg = odomTransformMsgs.transforms.front();
            odomTransformMsg.transform.translation.z = 0.0;

            m_UpdateLoop = this->create_wall_timer(
                20ms,
                std::bind(
                    &Controller::update,
                    this
                )
            );
        }

        Controller::~Controller()
        {

        }

        void Controller::update()
        {

            rclcpp::Time currentTime = this->get_clock()->now();

            auto currentVels = m_HardwareCommunicator->getWheelVelsFromHW();
            auto currentRobotVel = kinematics::mecanum::calculate_body_velocities(currentVels, m_WheelInfo);

            // Update Odometry and tf between odom and base_link frames.
            auto odomInfo = m_Odom->getOdometry(
                {currentRobotVel.linearX, currentRobotVel.linearY, currentRobotVel.angularZ},
                currentTime
            );

            if(odomInfo != std::nullopt)
            {
                bool odomTfPublished = false;

                auto odometryInfo = std::move(*odomInfo);
                if(m_RtOdomTransformPub->trylock())
                {
                    auto odomTransform = odometryInfo.second;
                    // We only send one transformation, so use the front transform in the vector.
                    auto& odomTransformMsg = m_RtOdomTransformPub->msg_.transforms.front();

                    odomTransformMsg.header.frame_id = odomTransform.header.frame_id;
                    odomTransformMsg.header.stamp = odomTransform.header.stamp;
                    odomTransformMsg.child_frame_id = odomTransform.child_frame_id;
                    odomTransformMsg.transform.translation.x = odomTransform.transform.translation.x;
                    odomTransformMsg.transform.translation.y = odomTransform.transform.translation.y;
                    odomTransformMsg.transform.rotation.x = odomTransform.transform.rotation.x;
                    odomTransformMsg.transform.rotation.y = odomTransform.transform.rotation.y;
                    odomTransformMsg.transform.rotation.z = odomTransform.transform.rotation.z;
                    odomTransformMsg.transform.rotation.w = odomTransform.transform.rotation.w;

                    m_RtOdomTransformPub->unlockAndPublish();

                    odomTfPublished = true;
                }

                if(odomTfPublished && m_RtOdomPub->trylock())
                {
                    auto odom = odometryInfo.first;

                    auto& odomMsg = m_RtOdomPub->msg_;
                    odomMsg.header.frame_id = odom.header.frame_id;
                    odomMsg.header.stamp = odom.header.stamp;
                    
                    odomMsg.child_frame_id = odom.child_frame_id;
                    
                    odomMsg.pose.pose.position.x = odom.pose.pose.position.x;
                    odomMsg.pose.pose.position.y = odom.pose.pose.position.y;

                    odomMsg.pose.pose.orientation.x = odom.pose.pose.orientation.x;
                    odomMsg.pose.pose.orientation.y = odom.pose.pose.orientation.y;
                    odomMsg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
                    odomMsg.pose.pose.orientation.w = odom.pose.pose.orientation.w;

                    odomMsg.twist.twist.linear.x = odom.twist.twist.linear.x;
                    odomMsg.twist.twist.linear.y = odom.twist.twist.linear.y;
                    odomMsg.twist.twist.angular.z = odom.twist.twist.angular.z;

                    m_RtOdomPub->unlockAndPublish();
                }

                
            }

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