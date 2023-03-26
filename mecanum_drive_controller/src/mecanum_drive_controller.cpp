/**
 * @file mecanum_drive_controller.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "mecanum_drive_controller/mecanum_drive_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace mecanum_drive_controller
{

    namespace
    {
        auto constexpr DEFAULT_VEL_CMD_TOPIC = "/cmd_vel";
        auto constexpr DEFAULT_ODOM_TOPIC = "/wheel_odom";
        auto constexpr DEFAULT_TF_TOPIC = "/tf";
    }

    MecanumDriveController::MecanumDriveController()
        : controller_interface::ControllerInterface()
    {

    };

    controller_interface::InterfaceConfiguration MecanumDriveController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmdInterfaceConfig;
        cmdInterfaceConfig.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for(const auto& jointName : m_Params.left_wheel_names)
        {
            cmdInterfaceConfig.names.push_back(jointName + "/" + hardware_interface::HW_IF_VELOCITY);
        }

        for(const auto& jointName : m_Params.right_wheel_names)
        {
            cmdInterfaceConfig.names.push_back(jointName + "/" + hardware_interface::HW_IF_VELOCITY);
        }

        return cmdInterfaceConfig;
    }

    controller_interface::InterfaceConfiguration MecanumDriveController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration stateInterfaceConfig;
        stateInterfaceConfig.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for(const auto& jointName : m_Params.left_wheel_names)
        {
            stateInterfaceConfig.names.push_back(jointName + "/" + hardware_interface::HW_IF_VELOCITY);
            //stateInterfaceConfig.names.push_back(jointName + "/" + hardware_interface::HW_IF_POSITION);
        }

        for(const auto& jointName : m_Params.right_wheel_names)
        {
            stateInterfaceConfig.names.push_back(jointName + "/" + hardware_interface::HW_IF_VELOCITY);
            //stateInterfaceConfig.names.push_back(jointName + "/" + hardware_interface::HW_IF_POSITION);
        }

        return stateInterfaceConfig;
    }

    controller_interface::return_type MecanumDriveController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if(get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            if(!m_IsHalted)
            {   
                this->halt();
                m_IsHalted = true;
            }

            return controller_interface::return_type::OK;
        }
        
        // Get the current joint velocities which are calculated from the incoming encoder data.
        kinematics::WheelVelocities currentWheelVels = {
            kinematics::WheelVelocityState::OK,
            m_FrontLeftWheelHandle->get_velocity_state(),
            m_FrontRightWheelHandle->get_velocity_state(),
            m_RearLeftWheelHandle->get_velocity_state(),
            m_RearRightWheelHandle->get_velocity_state()
        };

        auto currentBodyVel = kinematics::mecanum::calculate_body_velocities(
            currentWheelVels,
            m_WheelInfo
        );

        // Update odometry with the encoder data.
        auto updatedOdomInfo = m_Odom.update(currentBodyVel, time);

        if(updatedOdomInfo != std::nullopt && (m_PreviousPublishTimestamp + m_PublishPeriod < time))
        {
            m_PreviousPublishTimestamp += m_PublishPeriod;

            auto odomInfo = std::move(*updatedOdomInfo);
            
            if(m_RtOdomPub->trylock())
            {
                auto odom = odomInfo.first;
                auto& odomMsg = m_RtOdomPub->msg_;

                odomMsg.header.stamp = time;
                odomMsg.pose = odom.pose;

                odomMsg.twist.twist.linear.x = odom.twist.twist.linear.x;
                odomMsg.twist.twist.linear.y = odom.twist.twist.linear.y;
                odomMsg.twist.twist.angular.z = odom.twist.twist.angular.z;

                m_RtOdomPub->unlockAndPublish();
            }

            if(m_RtOdomTransformPub->trylock())
            {
                auto transform = odomInfo.second;

                auto& tfMsg = m_RtOdomTransformPub->msg_.transforms.front();
                
                tfMsg.header.stamp = time;
                tfMsg.transform = transform.transform;

                m_RtOdomTransformPub->unlockAndPublish();
            }


        }

        std::shared_ptr<geometry_msgs::msg::TwistStamped> lastTwistCommand;
        m_TwistCmdMsgPtr.get(lastTwistCommand);
        if(lastTwistCommand == nullptr)
        {
            RCLCPP_WARN(this->get_node()->get_logger(), "Received empty twist command.");
            return controller_interface::return_type::ERROR;
        }

        if(time - lastTwistCommand->header.stamp > m_VelocityCommandTimeout)
        {
            lastTwistCommand->twist.linear.x = 0.0;
            lastTwistCommand->twist.linear.y = 0.0;
            lastTwistCommand->twist.angular.z = 0.0;
        }

        m_PreviousUpdateTimeStamp = time;

        const kinematics::WheelVelocities wheelVelCmds = kinematics::mecanum::calculate_wheel_velocities(
            lastTwistCommand->twist.linear.x,
            lastTwistCommand->twist.linear.y,
            lastTwistCommand->twist.angular.z,
            m_WheelInfo            
        );

        m_FrontLeftWheelHandle->set_velocity_command(wheelVelCmds.frontLeft);
        m_FrontRightWheelHandle->set_velocity_command(wheelVelCmds.frontRight);
        m_RearLeftWheelHandle->set_velocity_command(wheelVelCmds.rearLeft);
        m_RearRightWheelHandle->set_velocity_command(wheelVelCmds.rearRight);
        
        return controller_interface::return_type::OK;

    }

    controller_interface::CallbackReturn MecanumDriveController::on_init()
    {
        try
        {
            m_ParamListener = std::make_shared<ParamListener>(this->get_node());
            m_Params = m_ParamListener->get_params();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Error while loading parameters.");
            return controller_interface::CallbackReturn::ERROR;
        }

        std::vector<std::string> tempJointNames;
        for(const auto& jointName : m_Params.left_wheel_names)
        {
            tempJointNames.push_back(jointName);
        }
        for(const auto& jointName : m_Params.right_wheel_names)
        {
            tempJointNames.push_back(jointName);
        }

        if(tempJointNames.size() != 4)
        {   
            RCLCPP_ERROR(
                this->get_node()->get_logger(),
                "Not enough wheel names provided.");
            return controller_interface::CallbackReturn::ERROR;
        }

        m_JointNames.frontLeftWheel = tempJointNames[0];
        m_JointNames.rearLeftWheel = tempJointNames[1];
        m_JointNames.frontRightWheel = tempJointNames[2];
        m_JointNames.rearRightWheel = tempJointNames[3];

        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        m_Odom.setWheelParams(
            {m_Params.wheel_radius, m_Params.wheel_separation_width, m_Params.wheel_separation_length}
        );

        m_VelocityCommandTimeout = std::chrono::milliseconds(static_cast<int>(m_Params.velocity_command_timeout * 1000.0));

        const geometry_msgs::msg::TwistStamped tempTwist;

        m_TwistCmdMsgPtr.set(std::make_shared<geometry_msgs::msg::TwistStamped>(tempTwist));
        m_PreviousCommands.emplace(tempTwist);
        m_PreviousCommands.emplace(tempTwist);

        m_TwistCmdSub = this->get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            DEFAULT_VEL_CMD_TOPIC,
            rclcpp::SystemDefaultsQoS(),
            std::bind(
                &MecanumDriveController::cmdVel_callback,
                this,
                std::placeholders::_1
            )
        );

        m_OdomPub = this->get_node()->create_publisher<nav_msgs::msg::Odometry>(
            DEFAULT_ODOM_TOPIC,
            rclcpp::SystemDefaultsQoS()
        );
        m_RtOdomPub = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
            m_OdomPub
        );

        m_OdomTransformPub = this->get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
            DEFAULT_TF_TOPIC,
            rclcpp::SystemDefaultsQoS()
        );
        m_RtOdomTransformPub = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
            m_OdomTransformPub
        );

        this->configure_realtime_publisher_msgs();

        m_PublishFrequency = m_Params.publish_rate;

        m_PublishPeriod = rclcpp::Duration::from_seconds(1.0 / m_PublishFrequency);

        m_PreviousPublishTimestamp = this->get_node()->now();

        m_PreviousUpdateTimeStamp = this->get_node()->now();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn MecanumDriveController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        m_FrontLeftWheelHandle = getWheelHandle(m_JointNames.frontLeftWheel);
        m_FrontRightWheelHandle = getWheelHandle(m_JointNames.frontRightWheel);
        m_RearLeftWheelHandle = getWheelHandle(m_JointNames.rearLeftWheel);
        m_RearRightWheelHandle = getWheelHandle(m_JointNames.rearRightWheel);

        if(!m_FrontLeftWheelHandle || !m_FrontRightWheelHandle || !m_RearLeftWheelHandle || !m_RearRightWheelHandle)
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(),
                "Failed during wheel interface initialization."
            );

            return controller_interface::CallbackReturn::ERROR;
        }

        m_IsHalted = false;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn MecanumDriveController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn MecanumDriveController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        if(!resetController())
            return controller_interface::CallbackReturn::ERROR;

        m_TwistCmdMsgPtr.set(std::make_shared<geometry_msgs::msg::TwistStamped>());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn MecanumDriveController::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        if(!resetController())
            return controller_interface::CallbackReturn::ERROR;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn MecanumDriveController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::unique_ptr<WheelHandle> MecanumDriveController::getWheelHandle(const std::string& joint_name)
    {
        const auto positionState = std::find_if(
            state_interfaces_.cbegin(),
            state_interfaces_.cend(),
            [&joint_name](const hardware_interface::LoanedStateInterface& loaned_state_interface){
                return loaned_state_interface.get_name() == joint_name && loaned_state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
            }
        );
        
        if(positionState == state_interfaces_.cend())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), 
                "Position state interface not found for joint: %s", 
                joint_name.c_str()
            );
            return nullptr;
        }

        const auto velocityState = std::find_if(
            state_interfaces_.cbegin(),
            state_interfaces_.cend(),
            [&joint_name](const hardware_interface::LoanedStateInterface& loaned_state_interface){
                return loaned_state_interface.get_name() == joint_name && loaned_state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
            }
        );

        if(velocityState == state_interfaces_.cend())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), 
                "Velocity state interface not found for joint: %s", 
                joint_name.c_str()
            );
            return nullptr;
        }
        
        const auto velCommandInterface = std::find_if(
            command_interfaces_.begin(),
            command_interfaces_.end(),
            [&joint_name](hardware_interface::LoanedCommandInterface& loaned_command_interface){
                return loaned_command_interface.get_name() == joint_name && loaned_command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
            }
        );

        if(velCommandInterface == command_interfaces_.end())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), 
                "Velocity command interface not found for joint: %s", 
                joint_name.c_str()
            );
            
            return nullptr;
        }

        return std::make_unique<WheelHandle>(
            std::ref(*positionState),
            std::ref(*velocityState),
            std::ref(*velCommandInterface)
        );
    }

    void MecanumDriveController::cmdVel_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> cmdVel_msg)
    {
        if((cmdVel_msg->header.stamp.sec == 0) && (cmdVel_msg->header.stamp.nanosec == 0))
        {
            cmdVel_msg->header.stamp = this->get_node()->get_clock()->now();
        }

        m_TwistCmdMsgPtr.set(std::move(cmdVel_msg));
    }

    void MecanumDriveController::configure_realtime_publisher_msgs()
    {
        std::string controller_ns = std::string(this->get_node()->get_namespace());
        if(controller_ns == "/")
        {
            controller_ns = "";
        }
        else
        {
            controller_ns = controller_ns + "/";
        }

        auto& odomMsg = m_RtOdomPub->msg_;
        odomMsg.header.frame_id = controller_ns + m_Params.odom_frame_id;
        odomMsg.child_frame_id = controller_ns + m_Params.base_frame_id;

        auto& odomTfMsg = m_RtOdomTransformPub->msg_;
        odomTfMsg.transforms.resize(1); // Resize transforms to 1 because we only publish transforms between the base_frame and odometry.
        odomTfMsg.transforms.front().header.frame_id = controller_ns + m_Params.odom_frame_id;
        odomTfMsg.transforms.front().child_frame_id = controller_ns + m_Params.base_frame_id;

    }

    bool MecanumDriveController::resetController()
    {
        std::queue<geometry_msgs::msg::TwistStamped> emptyTwistQ;
        std::swap(m_PreviousCommands, emptyTwistQ);

        m_TwistCmdSub.reset();

        m_FrontLeftWheelHandle.reset();
        m_FrontRightWheelHandle.reset();
        m_RearLeftWheelHandle.reset();
        m_RearRightWheelHandle.reset();

        m_TwistCmdMsgPtr.set(nullptr);

        m_IsHalted = false;

        return true;
    }

    void MecanumDriveController::halt()
    {
        m_FrontLeftWheelHandle->set_velocity_command(0.0);
        m_FrontRightWheelHandle->set_velocity_command(0.0);
        m_RearLeftWheelHandle->set_velocity_command(0.0);
        m_RearRightWheelHandle->set_velocity_command(0.0);
    }
}

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(
    mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerInterface
)