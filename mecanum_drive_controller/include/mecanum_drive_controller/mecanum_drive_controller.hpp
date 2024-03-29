/**
 * @file mecanum_drive_controller.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef MECANUM_DRIVE_CONTROLLER_HPP
#define MECANUM_DRIVE_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <memory>
#include <functional>
#include <queue>
#include <chrono>

#include "visibility_control.h"
#include "wheel_handle.hpp"
#include "mecanum_drive_controller_parameters.hpp"
#include "odom.hpp"

namespace mecanum_drive_controller
{
    class MecanumDriveController : public controller_interface::ControllerInterface
    {
        public:

        MECANUM_DRIVE_CONTROLLER_PUBLIC
        MecanumDriveController();
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
        
        MECANUM_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

        private: // Joint related variables and functions:

        std::unique_ptr<WheelHandle> m_FrontLeftWheelHandle;
        std::unique_ptr<WheelHandle> m_FrontRightWheelHandle;
        std::unique_ptr<WheelHandle> m_RearLeftWheelHandle;
        std::unique_ptr<WheelHandle> m_RearRightWheelHandle;

        struct
        {
            std::string frontLeftWheel;
            std::string frontRightWheel;
            std::string rearLeftWheel;
            std::string rearRightWheel;
        } m_JointNames;

        std::unique_ptr<WheelHandle> getWheelHandle(const std::string& joint_name);

        kinematics::WheelInformation m_WheelInfo;

        private:

        mecanum::odometry::Odometry m_Odom;

        std::chrono::milliseconds m_VelocityCommandTimeout{500};

        rclcpp::Time m_PreviousUpdateTimeStamp;

        rclcpp::Time m_PreviousPublishTimestamp;

        double m_PublishFrequency = 100.0; // [Hz];

        rclcpp::Duration m_PublishPeriod = rclcpp::Duration::from_nanoseconds(0);

        bool m_IsHalted = false;

        private: // ROS 2 related variables and functions:

        std::shared_ptr<ParamListener> m_ParamListener;
        Params m_Params;

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistCmdSub;
        realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> m_TwistCmdMsgPtr{nullptr};
        std::queue<geometry_msgs::msg::TwistStamped> m_PreviousCommands;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_OdomPub;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> m_RtOdomPub = nullptr;

        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_OdomTransformPub;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> m_RtOdomTransformPub;

        void cmdVel_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> cmdVel_msg);

        /**
         * @brief Sets the realtime publisher messages for Odom and Transform publishers.
         * 
         */
        void configure_realtime_publisher_msgs();

        bool resetController();

        void halt();

    };
}

#endif 