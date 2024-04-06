#include "move_group_interface.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/planning_scene.hpp>

using namespace std;
using namespace Eigen;

MoveGroupInterfaceNode::MoveGroupInterfaceNode(const rclcpp::NodeOptions &options) : rclcpp::Node("move_group_interface", options){

                                                                                     };

bool MoveGroupInterfaceNode::setup()
{
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    model_ = robot_model_loader.getModel();
    scene_.reset(new planning_scene::PlanningScene(model_));
    solver_left_arm_.reset(new MoveItIKSolver(model_, scene_, "l_iiwa_arm"));
    solver_right_arm_.reset(new MoveItIKSolver(model_, scene_, "r_iiwa_arm"));

    octomap_sub_ =  create_subscription<octomap_msgs::msg::Octomap>(
      "octomap_binary", 10, std::bind(&MoveGroupInterfaceNode::octomapCallback, this, std::placeholders::_1));

      octomap_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1) ;
}

void MoveGroupInterfaceNode::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr octomap) const {
    
    moveit_msgs::msg::PlanningScene planning_scene ;
    scene_->getPlanningSceneMsg(planning_scene) ;
    planning_scene.world.octomap.octomap = *octomap ;
    auto acm = planning_scene.allowed_collision_matrix ;
    
   
     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str())
     octomap_pub_->publish(planning_scene);
      
}

static Eigen::Isometry3d poseFromEigen(const Vector3d &c, const Quaterniond &r)
{
    Isometry3d t;
    t.setIdentity();
    t.linear() = r.toRotationMatrix();
    t.translation() = c;
    return t;
}
void MoveGroupInterfaceNode::filterGrasps(const std::vector<grasp_planner_interfaces::msg::Grasp> &candidates,
                                          std::vector<grasp_planner_interfaces::msg::Grasp> &filtered,
                                          std::vector<GraspCandidate> &result)
{
    const double finger_width = 0.01;

    uint count = 0;
    for (const auto &grasp : candidates)
    {
        const auto &trv = grasp.translation;
        const auto &rotv = grasp.rotation;
        count++;
        Eigen::Vector3d c(trv.x, trv.y, trv.z);
        Eigen::Quaterniond rot(rotv.w, rotv.x, rotv.y, rotv.z);
        Eigen::Matrix3d m = rot.toRotationMatrix();

        static const double base_depth = 0.02;

        auto approach = m.col(0);
        auto binormal = m.col(1);
        double hw = 0.5 * grasp.width;
        double hand_depth = grasp.depth;

        Vector3d left_bottom = c - (hw + finger_width) * binormal - approach * (base_depth + finger_width / 2);
        Vector3d right_bottom = c + (hw + finger_width) * binormal - approach * (base_depth + finger_width / 2);
        Vector3d left_center = c + (hw + finger_width / 2) * binormal + approach * (hand_depth - base_depth) / 2.0;
        Vector3d right_center = c - (hw + finger_width / 2) * binormal + approach * (hand_depth - base_depth) / 2.0;

        // we need to align x axis with z axis
        rot.y() = -rot.y();
        rot.z() = -rot.z();

        auto ls = solver_left_arm_->solveIK(poseFromEigen(left_center, rot));
        auto rs = solver_right_arm_->solveIK(poseFromEigen(right_center, rot));

        if (!ls.empty() && !rs.empty())
        {
            filtered.emplace_back(grasp) ;
            result.emplace_back(left_center, right_center, rot);
        }
    }
}