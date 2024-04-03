#pragma once

#include <rclcpp/node.hpp>
#include "moveit_ik.hpp"
#include <grasp_planner_interfaces/msg/grasp.hpp>

class MoveGroupInterfaceNode: public rclcpp::Node {
public:
    MoveGroupInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) ;

    bool setup() ;

    void filterGrasps(const std::vector<grasp_planner_interfaces::msg::Grasp> &candidates, std::vector<grasp_planner_interfaces::msg::Grasp> &filtered) ;
private:
    moveit::core::RobotModelConstPtr model_ ;
    planning_scene::PlanningScenePtr scene_ ;
    std::shared_ptr<MoveItIKSolver> solver_left_arm_, solver_right_arm_ ;
};