#pragma once

#include <rclcpp/node.hpp>
#include "moveit_ik.hpp"
#include <grasp_planner_interfaces/msg/grasp.hpp>
#include <octomap_msgs/msg/octomap.hpp>

struct GraspCandidate {
    GraspCandidate(const Eigen::Vector3d &lc, const Eigen::Vector3d &rc, const Eigen::Quaterniond &r):
        dir_(r), lpos_(lc), rpos_(rc) {}
        
    Eigen::Quaterniond dir_ ;
    Eigen::Vector3d lpos_, rpos_ ;
};

class MoveGroupInterfaceNode: public rclcpp::Node {
public:
    MoveGroupInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) ;

    bool setup() ;

    void filterGrasps(const std::vector<grasp_planner_interfaces::msg::Grasp> &candidates, 
        std::vector<grasp_planner_interfaces::msg::Grasp> &filtered,
        std::vector<GraspCandidate> &res) ;
private:

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr octomap) const ;
    
    moveit::core::RobotModelConstPtr model_ ;
    planning_scene::PlanningScenePtr scene_ ;
    std::shared_ptr<MoveItIKSolver> solver_left_arm_, solver_right_arm_ ;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr octomap_pub_;
};