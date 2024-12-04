#pragma once

#include <rclcpp/node.hpp>
#include "moveit_ik.hpp"
#include <grasp_planner_msgs/msg/grasp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <grasp_planner_msgs/srv/motion_planning.hpp>


class MotionPlanningNode: public rclcpp::Node {
public:
    MotionPlanningNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) ;

    void setup() ;

private:

    using MotionPlanningSrv = grasp_planner_msgs::srv::MotionPlanning ;


    void plan(const std::shared_ptr<MotionPlanningSrv::Request> request, std::shared_ptr<MotionPlanningSrv::Response> response) ;

private:

    void computeMotionPlan(const std::vector<double> &lp, const std::vector<double> &rp, MotionPlanningSrv::Response &response) ;
    bool jointStateToRobotState(const sensor_msgs::msg::JointState &joint_state, moveit::core::RobotState &state);

    rclcpp::Service<MotionPlanningSrv>::SharedPtr service_ ;
    
    moveit::core::RobotModelConstPtr model_ ;
    planning_scene::PlanningScenePtr scene_ ;
    std::shared_ptr<MoveItIKSolver> solver_left_arm_, solver_right_arm_ ;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>> robot_state_publisher_ ;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_ ;
};