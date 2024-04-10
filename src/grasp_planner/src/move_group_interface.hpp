#pragma once

#include <rclcpp/node.hpp>
#include "moveit_ik.hpp"
#include <grasp_planner_interfaces/msg/grasp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

struct GraspCandidate {
    GraspCandidate(float cost, const std::vector<double> &jl, const Eigen::Vector3d &lc, const std::vector<double> &jr, const Eigen::Vector3d &rc, const Eigen::Quaterniond &r):
        cost_(cost), dir_(r), lpos_(lc), rpos_(rc), jl_(jl), jr_(jr) {}
        
    Eigen::Quaterniond dir_ ;
    Eigen::Vector3d lpos_, rpos_ ;
    std::vector<double> jl_, jr_ ;
    float cost_ ;
    
    moveit_msgs::msg::RobotState start_state_;
    moveit_msgs::msg::RobotTrajectory trajectory_;

};

class MoveGroupInterfaceNode: public rclcpp::Node {
public:
    MoveGroupInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) ;

    bool setup() ;

    void filterGrasps(const std::vector<grasp_planner_interfaces::msg::Grasp> &candidates, 
        float offset, float finger_width,
        std::vector<grasp_planner_interfaces::msg::Grasp> &filtered,
        std::vector<GraspCandidate> &res) ;

    void computeMotionPlans(std::vector<GraspCandidate> &candidates, uint max_plans) ;
private:

    moveit::core::RobotModelConstPtr model_ ;
    planning_scene::PlanningScenePtr scene_ ;
    std::shared_ptr<MoveItIKSolver> solver_left_arm_, solver_right_arm_ ;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>> robot_state_publisher_ ;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_ ;
};