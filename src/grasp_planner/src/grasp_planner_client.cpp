#include <rclcpp/rclcpp.hpp>
#include <grasp_planner_interfaces/srv/grasp_net.hpp>

#include <memory>

void callback(const std::shared_ptr<grasp_planner_interfaces::srv::GraspNet::Request> request,
          std::shared_ptr<grasp_planner_interfaces::srv::GraspNet::Response>      response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<grasp_planner_interfaces::srv::GraspNet>::SharedPtr service =
    node->create_service<grasp_planner_interfaces::srv::GraspNet>("graspnet", &callback);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}