#include <rclcpp/rclcpp.hpp>
#include "grasp_candidates_node.hpp"
#include "masked_point_cloud.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<MaskedPointCloud> pcl(new MaskedPointCloud()) ;
    pcl->setup() ;

    std::shared_ptr<GraspCandidatesNode> node(new GraspCandidatesNode()) ;
    node->setup(pcl) ;

    rclcpp::executors::MultiThreadedExecutor executor ;
    executor.add_node(pcl) ;
    executor.add_node(node) ;

    executor.spin() ;
    

    rclcpp::shutdown();
}