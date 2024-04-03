#pragma once

#include <rclcpp/node.hpp>
#include "moveit_ik.hpp"

class MoveGroupInterfaceNode: public rclcpp::Node {
public:
    MoveGroupInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) ;

    bool setup() ;
private:
    moveit::core::RobotModelConstPtr model_ ;
    planning_scene::PlanningScenePtr scene_ ;
    std::shared_ptr<MoveItIKSolver> solver_left_arm_, solver_right_arm_ ;
};