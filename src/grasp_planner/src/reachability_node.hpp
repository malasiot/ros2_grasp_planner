#pragma once

#include <rclcpp/node.hpp>
#include "moveit_ik.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

class ReachabilityNode: public rclcpp::Node {
public:
    ReachabilityNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) ;

    void setup() ;
    void doTest() ;

    private:

    planning_scene::PlanningScenePtr scene_ ;
    std::shared_ptr<MoveItIKSolver> solver_left_arm_, solver_right_arm_ ;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    double dist_thresh_ = 0.01 ;
};