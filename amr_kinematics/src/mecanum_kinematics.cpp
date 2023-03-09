/**
 * @file mecanum_kinematics.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "mecanum_kinematics.hpp"

namespace kinematics
{
    namespace mecanum
    {
        WheelVelocities calculate_wheel_velocities(
            const double& linear_x,
            const double& linear_y,
            const double& angular_z,
            const WheelInformation& wheel_info)
        {
            WheelVelocities vels;
            double wheelConstant = wheel_info.wheel_separation_width + wheel_info.wheel_separation_length;
            vels.frontLeft = (1.0 / wheel_info.wheel_radius) * (linear_x - linear_y - wheelConstant * angular_z);
            vels.frontRight = (1.0 / wheel_info.wheel_radius) * (linear_x + linear_y + wheelConstant * angular_z);
            vels.rearLeft = (1.0 / wheel_info.wheel_radius) * (linear_x + linear_y - wheelConstant * angular_z);
            vels.rearRight = (1.0 / wheel_info.wheel_radius) * (linear_x - linear_y + wheelConstant * angular_z);
        
            return vels;
        }

        BodyVelocities calculate_body_velocities(
            const WheelVelocities& wheel_velocities,
            const WheelInformation& wheel_info)
        {
            BodyVelocities bVels;
            bVels.linearX = (
                wheel_velocities.frontLeft  + 
                wheel_velocities.frontRight + 
                wheel_velocities.rearLeft   + 
                wheel_velocities.rearRight
                ) * (wheel_info.wheel_radius / 4.0);
            
            bVels.linearY = (
                -wheel_velocities.frontLeft +
                wheel_velocities.frontRight +
                wheel_velocities.rearLeft   -
                wheel_velocities.rearRight
            ) * (wheel_info.wheel_radius / 4.0);

            bVels.angularZ = (
                -wheel_velocities.frontLeft +
                wheel_velocities.frontRight -
                wheel_velocities.rearLeft   +
                wheel_velocities.rearRight
            ) * (wheel_info.wheel_radius / (4.0 * (wheel_info.wheel_separation_width + wheel_info.wheel_separation_length)));

            return bVels;
        }
    }
}