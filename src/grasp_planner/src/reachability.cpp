#include <rclcpp/rclcpp.hpp>
#include "reachability_node.hpp"
#include "grasp_planner_interfaces/srv/reachability.hpp"

class ReachabilityService:  public rclcpp::Node {
public:
    ReachabilityService(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): rclcpp::Node("reachability_service", options) {
        service_ = create_service<ReachabilitySrv>("reachability_service", std::bind(&ReachabilityService::run, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);
    }

    void setup(const std::shared_ptr<ReachabilityNode> &rn) {
        reachability_node_ = rn ;
    }

private:
   
    using ReachabilitySrv = grasp_planner_interfaces::srv::Reachability ;

    rclcpp::Service<ReachabilitySrv>::SharedPtr service_ ;
        
    void run(const std::shared_ptr<ReachabilitySrv::Request> request, std::shared_ptr<ReachabilitySrv::Response> response) {
        reachability_node_->doTest() ;
    }
   
    std::shared_ptr<ReachabilityNode> reachability_node_ ;
  
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

   
    rclcpp::NodeOptions options ;
    options.automatically_declare_parameters_from_overrides(false) ;

    std::shared_ptr<ReachabilityNode> reach_node(new ReachabilityNode(options)) ;

    reach_node->setup() ;

    std::shared_ptr<ReachabilityService> srv_node(new ReachabilityService()) ;

    srv_node->setup(reach_node) ;

    rclcpp::executors::MultiThreadedExecutor executor ;

    executor.add_node(reach_node) ;
    executor.add_node(srv_node) ;

    executor.spin() ;

    rclcpp::shutdown();
}