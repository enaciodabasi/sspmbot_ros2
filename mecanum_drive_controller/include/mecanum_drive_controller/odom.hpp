/**
 * @file odometry.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <iostream>
#include <optional>

#include "mecanum_kinematics.hpp"

namespace mecanum
{
    namespace odometry
    {   
        typedef std::pair<nav_msgs::msg::Odometry, geometry_msgs::msg::TransformStamped> OdomInfo;
        
        class Odometry
        {
            
            public:
            
            Odometry();
            Odometry(const kinematics::WheelInformation& wheel_info);
            ~Odometry();

            inline void setWheelParams(const kinematics::WheelInformation& wheel_info)
            {
                m_WheelInformation = wheel_info;
            }

            /**
             * @brief Get the Odometry based on the body velocities and the wheel params.
             * 
             * @param body_vels: Struct containing the linear X and Y velocities and angular Z velocity. 
             * @param curr_time: rclcpp::Time, time when this function is called. 
             * @return Odom std::nullopt if the time interval is too small, otherwise nav_msgs::msg::Odom. 
             */
            std::optional<OdomInfo> getOdometry(
                const kinematics::BodyVelocities& body_vels,
                rclcpp::Time curr_time
            );

            private:

            double m_X;
            double m_Y;
            double m_Th;

            rclcpp::Time m_Timestamp;

            nav_msgs::msg::Odometry m_Odometry;

            std::string m_OdomTransformFrameID = "odom";
            std::string m_OdomTransformChildFrameID = "base_link";

            kinematics::WheelInformation m_WheelInformation;

            void updatePositions(
                const double linX,
                const double linY,
                const double angZ
            );
            

        };
    }
}

#endif