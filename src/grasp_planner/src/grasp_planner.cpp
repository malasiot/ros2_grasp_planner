#include <rclcpp/rclcpp.hpp>
#include "move_group_interface.hpp"
#include "grasp_planner_service.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options ;
    options.automatically_declare_parameters_from_overrides(true) ;
    std::shared_ptr<MoveGroupInterfaceNode> mgi(new MoveGroupInterfaceNode(options)) ;
    mgi->setup() ;

    std::shared_ptr<GraspPlannerService> service(new GraspPlannerService()) ;
    service->setup(mgi) ;

    rclcpp::executors::MultiThreadedExecutor executor ;
    executor.add_node(mgi) ;
    executor.add_node(service) ;

    executor.spin() ;
    

    rclcpp::shutdown();
}