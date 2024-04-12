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
    GraspCandidate(float cost, const std::vector<double> &jl, double lm, 
    const Eigen::Vector3d &lc, const std::vector<double> &jr, double rm,
     Eigen::Vector3d &rc, const Eigen::Quaterniond &r):
        cost_(cost), dir_(r), lpos_(lc), rpos_(rc), jl_(jl), jr_(jr), lm_(lm), rm_(rm) {}
        
    Eigen::Quaterniond dir_ ;
    Eigen::Vector3d lpos_, rpos_ ;
    std::vector<double> jl_, jr_ ;
    double lm_, rm_ ;
    float cost_ ;
    
    moveit_msgs::msg::RobotState start_state_;
    moveit_msgs::msg::RobotTrajectory trajectory_;

};

class MoveGroupInterfaceNode: public rclcpp::Node {
public:
    MoveGroupInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) ;

    bool setup() ;

    void filterGrasps(const std::vector<grasp_planner_interfaces::msg::Grasp> &candidates, 
        float offset, float finger_width, float clearance_thresh, 
        std::vector<grasp_planner_interfaces::msg::Grasp> &filtered,
        std::vector<GraspCandidate> &res) ;

    void computeMotionPlans(std::vector<GraspCandidate> &candidates, const sensor_msgs::msg::JointState &start_state, uint max_plans) ;
private:

    bool jointStateToRobotState(const sensor_msgs::msg::JointState &joint_state, moveit::core::RobotState &state);

    moveit::core::RobotModelConstPtr model_ ;
    planning_scene::PlanningScenePtr scene_ ;
    std::shared_ptr<MoveItIKSolver> solver_left_arm_, solver_right_arm_ ;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>> robot_state_publisher_ ;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_ ;
};