/**
 * @file odometry.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "odom.hpp"

namespace sspmbot
{
    namespace odometry
    {
        Odometry::Odometry(const kinematics::WheelInformation& wheel_info)
            : m_X(0.0), m_Y(0.0), m_Th(0.0)
        {
            m_Timestamp = rclcpp::Time(0.0);
        }

        std::optional<OdomInfo> Odometry::getOdometry(
            const kinematics::BodyVelocities& body_vels,
            rclcpp::Time curr_time
        )
        {
            
            const double dt = curr_time.seconds() - m_Timestamp.seconds();
            if(dt < 0.0001) // If the time interval is too small to update the odometry healthily, return std::nullopt.
            {
                return std::nullopt;
            }   
            
            updatePositions(
                body_vels.linearX,
                body_vels.linearY,
                body_vels.angularZ
            );

            m_Timestamp = curr_time; // Update time.
            
            tf2::Quaternion yawQuat;
            yawQuat.setRPY(0, 0, m_Th); // Create Quaternion from the yaw of the robot (m_Th).
            
            geometry_msgs::msg::Quaternion odomQuat;
            odomQuat = tf2::toMsg<tf2::Quaternion, geometry_msgs::msg::Quaternion>(yawQuat); 

            geometry_msgs::msg::TransformStamped odomTransform;
            odomTransform.header.stamp = curr_time;
            odomTransform.header.frame_id = m_OdomTransformFrameID;
            odomTransform.child_frame_id = m_OdomTransformChildFrameID;
            odomTransform.transform.translation.x = m_X;
            odomTransform.transform.translation.y = m_Y;
            odomTransform.transform.translation.z = 0.0;
            odomTransform.transform.rotation = odomQuat;

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = curr_time;
            odom.header.frame_id = m_OdomTransformFrameID;
            odom.pose.pose.position.x = m_X;
            odom.pose.pose.position.y = m_Y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odomQuat;
            odom.child_frame_id = m_OdomTransformFrameID;
            odom.twist.twist.linear.x = body_vels.linearX;
            odom.twist.twist.linear.y = body_vels.linearY;
            odom.twist.twist.angular.z = body_vels.angularZ;

            m_Odometry = odom;

            return std::make_pair(odom, odomTransform);

        }

        void Odometry::updatePositions(
            const double linX,
            const double linY,
            const double angZ
        )
        {
            m_X += linX * std::cos(angZ) - linY * std::sin(angZ);
            m_Y += linX * std::sin(angZ) + linY * std::cos(angZ);
            m_Th += angZ;
            m_Th = fmod(m_Th, 2*M_PI); // take the remaining of mod(360, m_Th)
        }
    }
}