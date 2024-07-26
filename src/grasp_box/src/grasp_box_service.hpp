#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "grasp_planner_interfaces/srv/grasp_box.hpp"
#include "grasp_planner_interfaces/msg/grasp.hpp"

struct Box ;

class GraspBoxService:  public rclcpp::Node {
public:
    GraspBoxService(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    
    using GraspBoxSrv = grasp_planner_interfaces::srv::GraspBox;

    rclcpp::Service<GraspBoxSrv>::SharedPtr service_ ;
   
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> boxes_rviz_pub_ ;

    void visualizeBoxes(const std::vector<Box> &boxes);

    void callback(const std::shared_ptr<GraspBoxSrv::Request> request, std::shared_ptr<GraspBoxSrv::Response> response);
};