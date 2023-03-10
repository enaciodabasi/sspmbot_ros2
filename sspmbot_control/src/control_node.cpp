#include "sspmbot_control/controller.hpp"
#include "sspmbot_control/hw_communicator.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<sspmbot::hardware::Communicator> hcn = std::make_shared<sspmbot::hardware::Communicator>("amr_hardware_communicator_node");

    std::shared_ptr<sspmbot::controller::Controller> cn = std::make_shared<sspmbot::controller::Controller>(hcn);

    rclcpp::executors::MultiThreadedExecutor mte;

    mte.add_node(hcn);
    mte.add_node(cn);
   
    mte.spin();
    
    rclcpp::shutdown();
    
    return 0;

}   
