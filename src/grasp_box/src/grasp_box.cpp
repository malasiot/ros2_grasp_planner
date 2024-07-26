#include <rclcpp/rclcpp.hpp>
#include "grasp_box_service.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<GraspBoxService> service(new GraspBoxService()) ;
    rclcpp::executors::MultiThreadedExecutor executor ;
    executor.add_node(service) ;
    executor.spin() ;

    rclcpp::shutdown();
}