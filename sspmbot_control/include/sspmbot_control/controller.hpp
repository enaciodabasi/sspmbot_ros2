/**
 * @file controller.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_box.h>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "amr_kinematics/mecanum_kinematics.hpp"
#include "amr_kinematics/odom.hpp"
#include "hw_communicator.hpp"

namespace sspmbot
{
    namespace controller
    {
        class Controller : public rclcpp::Node
        {
            public:

            Controller(std::shared_ptr<hardware::Communicator>& communicator_node);
            ~Controller();

            void update();

            private:

            std::shared_ptr<hardware::Communicator> m_HardwareCommunicator;

            rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistCmdSub;
            realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> m_TwistCmdMsgPtr{nullptr};

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_OdomPub;
            std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> m_RtOdomPub = nullptr;

            rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_OdomTransformPub;
            std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> m_RtOdomTransformPub;

            std::unique_ptr<odometry::Odometry> m_Odom;

            rclcpp::TimerBase::SharedPtr m_UpdateLoop;

            std::chrono::milliseconds m_VelCommandTimeout{500};

            kinematics::WheelInformation m_WheelInfo;


            void cmdVel_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> cmd_vel_msg);

        };
    }
}

#endif // CONTROLLER_HPP