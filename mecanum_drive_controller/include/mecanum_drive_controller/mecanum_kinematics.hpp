/**
 * @file mecanum_kinematics.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MECANUM_KINEMATICS
#define MECANUM_KINEMATICS

#include <string>

namespace kinematics
{

    struct WheelInformation
        {
            double wheel_radius{0.0};

            double wheel_separation_width{0.0};

            double wheel_separation_length{0.0};
        };

        enum class WheelVelocityState
        {
            OK,
            ERROR
        };

        /**
         * @brief Contains individual wheel velocities in [m/s]
         * 
         */
        struct WheelVelocities
        {
            
            WheelVelocityState state;

            double frontLeft{0.0};

            double frontRight{0.0};
            
            double rearLeft{0.0};

            double rearRight{0.0};
        };

        struct BodyVelocities
        {
            double linearX{0.0};

            double linearY{0.0};

            double angularZ{0.0};
        };

    namespace mecanum
    {

        /**
         * @brief Calculates individual wheel velocities
         * using the kinematic equation of a 4WD Mecanum robot.
         * @param linear_x: twist.linear.x 
         * @param linear_y: twist.linear.y
         * @param angular_z: twist.angular.z
         * @param wheel_info: Struct that holds the required information about the wheel.
         * @return A WheelVelocities struct. 
         */
        WheelVelocities calculate_wheel_velocities(
            const double& linear_x,
            const double& linear_y,
            const double& angular_z,
            const WheelInformation& wheel_info
        );
        /**
         * @brief 
         * 
         * @param wheel_velocities 
         * @param wheel_info 
         * @return BodyVelocities 
         */
        BodyVelocities calculate_body_velocities(
            const WheelVelocities& wheel_velocities,
            const WheelInformation& wheel_info
        );
    }
}

#endif // MECANUM_KINEMATICS