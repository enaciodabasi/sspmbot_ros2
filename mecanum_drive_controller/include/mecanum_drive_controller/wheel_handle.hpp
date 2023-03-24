/**
 * @file wheel_state.hpp
 * @author Eren Naci Odabaşı
 * @brief 
 * @version 0.1
 * @date 2023-03-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef WHEEL_STATE_HPP
#define WHEEL_STATE_HPP

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace mecanum_drive_controller
{
    class WheelHandle
    {
        public:

        WheelHandle(
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> loaned_position_state,
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> loaned_velocity_state,
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> loaned_velocity_command
        )   : m_LoanedPositionState{loaned_position_state}, m_LoanedVelocityState{loaned_velocity_state}, m_LoanedVelocityCommand{loaned_velocity_command} {}

        inline void set_velocity_command(double vel)
        {
            m_LoanedVelocityCommand.get().set_value(vel);
        }
        inline double get_position_state() const
        {
            return m_LoanedPositionState.get().get_value();
        }
        inline double get_velocity_state() const
        {
            return m_LoanedVelocityState.get().get_value();
        }

        private:

        std::reference_wrapper<const hardware_interface::LoanedStateInterface> m_LoanedPositionState;

        std::reference_wrapper<const hardware_interface::LoanedStateInterface> m_LoanedVelocityState;

        std::reference_wrapper<hardware_interface::LoanedCommandInterface> m_LoanedVelocityCommand;

    };
}

#endif // WHEEL_STATE_HPP