#include <rclcpp/rclcpp.hpp>
#include "grasp_candidates_filter_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<GraspCandidatesFilterNode> node(new GraspCandidatesFilterNode()) ;
    node->setup() ;

    rclcpp::executors::SingleThreadedExecutor executor ;
    executor.add_node(node) ;

    executor.spin() ;
    

    rclcpp::shutdown();
}