#include "move_group_interface.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/planning_scene.hpp>

MoveGroupInterfaceNode::MoveGroupInterfaceNode(const rclcpp::NodeOptions &options): rclcpp::Node("move_group_interface", options) {

} ;

bool MoveGroupInterfaceNode::setup() {
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    model_ = robot_model_loader.getModel();
    scene_.reset(new planning_scene::PlanningScene(model_)) ;
    solver_left_arm_.reset(new MoveItIKSolver(model_, scene_, "l_iiwa_arm")) ;
    solver_right_arm_.reset(new MoveItIKSolver(model_, scene_, "r_iiwa_arm")) ;
}