#include <rclcpp/rclcpp.hpp>
#include "motion_planning_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<MotionPlanningNode> node(new MotionPlanningNode()) ;
    node->setup() ;

    rclcpp::executors::SingleThreadedExecutor executor ;
    executor.add_node(node) ;
    executor.spin() ;

    rclcpp::shutdown();
}