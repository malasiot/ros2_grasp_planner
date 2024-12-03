#pragma once

#include <rclcpp/node.hpp>
#include "moveit_ik.hpp"
#include <grasp_planner_msgs/msg/grasp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <grasp_planner_msgs/srv/grasp_candidates_filter.hpp>

struct GraspCandidate {
    GraspCandidate(uint c, float cost, const std::vector<double> &jl, double lm, 
    const Eigen::Vector3d &lc, const Eigen::Quaterniond &lq, const std::vector<double> &jr, double rm,
     Eigen::Vector3d &rc, const Eigen::Quaterniond &rq): candidate_(c),
        ldir_(lq), rdir_(rq), lpos_(lc), rpos_(rc), jl_(jl), jr_(jr), lm_(lm), rm_(rm), cost_(cost)  {}
        
    uint candidate_ ;
    Eigen::Quaterniond ldir_, rdir_ ;
    Eigen::Vector3d lpos_, rpos_ ;
    std::vector<double> jl_, jr_ ;
    double lm_, rm_ ;
    float cost_ ;
};

struct GraspCandidateFilterParams {
    double offset_, finger_width_, clearence_thresh_ ;
    std::vector<double> tol_6dof_, tol_7dof_ ;
    uint n_attempts_6dof_, n_attempts_7dof_ ;
};

class GraspCandidatesFilterNode: public rclcpp::Node {
public:
    GraspCandidatesFilterNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) ;

    void setup() ;

private:

    using GraspCandidatesFilterSrv = grasp_planner_msgs::srv::GraspCandidatesFilter ;


    void filter(const std::shared_ptr<GraspCandidatesFilterSrv::Request> request, std::shared_ptr<GraspCandidatesFilterSrv::Response> response) ;

    void filterGrasps(const std::vector<grasp_planner_msgs::msg::Grasp> &candidates, const GraspCandidateFilterParams &params,
        bool left_tactile, bool right_tactile,
        std::vector<grasp_planner_msgs::msg::Grasp> &filtered,
        std::vector<GraspCandidate> &res) ;
  
    void visualizeResult(const GraspCandidate &r);
private:

    bool jointStateToRobotState(const sensor_msgs::msg::JointState &joint_state, moveit::core::RobotState &state);

    
    rclcpp::Service<GraspCandidatesFilterSrv>::SharedPtr service_ ;

     std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> grasps_rviz_pub_ ;

    moveit::core::RobotModelConstPtr model_ ;
    planning_scene::PlanningScenePtr scene_ ;
    std::shared_ptr<MoveItIKSolver> solver_left_arm_, solver_right_arm_ ;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>> robot_state_publisher_ ;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_ ;
};