#include "move_group_interface.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/planning_scene.hpp>

using namespace std ;
using namespace Eigen ;

MoveGroupInterfaceNode::MoveGroupInterfaceNode(const rclcpp::NodeOptions &options): rclcpp::Node("move_group_interface", options) {

} ;

bool MoveGroupInterfaceNode::setup() {
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    model_ = robot_model_loader.getModel();
    scene_.reset(new planning_scene::PlanningScene(model_)) ;
    solver_left_arm_.reset(new MoveItIKSolver(model_, scene_, "l_iiwa_arm")) ;
    solver_right_arm_.reset(new MoveItIKSolver(model_, scene_, "r_iiwa_arm")) ;
}

void MoveGroupInterfaceNode::filterGrasps(const std::vector<grasp_planner_interfaces::msg::Grasp> &candidates, std::vector<grasp_planner_interfaces::msg::Grasp> &filtered) {
    const double finger_width = 0.01 ;

    for( const auto &grasp: candidates ) {
         const auto &trv = grasp.translation ;
      const auto &rotv = grasp.rotation ;

      Eigen::Vector3d c(trv[0], trv[1], trv[2]) ;
      Eigen::Matrix3d rot ;
      rot << rotv[0], rotv[1], rotv[2], 
            rotv[3], rotv[4], rotv[5],
            rotv[6], rotv[7], rotv[8] ;

      static const double base_depth = 0.02 ;
   
      auto approach = rot.col(0) ; 
      auto binormal = -rot.col(1) ;
      double hw = 0.5 * grasp.width;
      double hand_depth = grasp.depth ;

      Vector3d left_bottom =  c - (hw + finger_width) * binormal - approach * (base_depth + finger_width/2);
      Vector3d right_bottom = c + (hw + finger_width) * binormal - approach * (base_depth + finger_width/2);
      Vector3d left_center = c - (hw + finger_width/2) * binormal + approach * ( hand_depth - base_depth )/2.0 ;
      Vector3d right_center = c + (hw + finger_width/2) * binormal + approach * ( hand_depth - base_depth )/2.0 ;
      

    }
}