#include "move_group_interface.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/planning_scene.hpp>

using namespace std;
using namespace Eigen;

MoveGroupInterfaceNode::MoveGroupInterfaceNode(const rclcpp::NodeOptions &options) : rclcpp::Node("move_group_interface", options){
}

bool MoveGroupInterfaceNode::setup() {
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");

  //  planning_scene_monitor_->startWorldGeometryMonitor() ;
  planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
  robot_state_publisher_ = create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 10) ;
  
}

static Eigen::Isometry3d poseFromEigen(const Vector3d &c, const Quaterniond &r)
{
    Isometry3d t;
    t.setIdentity();
    t.linear() = r.toRotationMatrix();
    t.translation() = c;
    return t;
}

struct ReachTest {
    Vector3d l_, r_ ;
    int candidate_ ;
    Quaterniond rot_ ;
    bool valid_ = false ;
    std::vector<double> ls_, rs_ ;
};

void MoveGroupInterfaceNode::filterGrasps(const std::vector<grasp_planner_interfaces::msg::Grasp> &candidates,
                                            float depth_offset, float finger_width, 
                                          std::vector<grasp_planner_interfaces::msg::Grasp> &filtered,
                                          std::vector<GraspCandidate> &result)
{
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_) ;

    MoveItIKSolver solver_left_arm(planning_scene, "l_iiwa_arm", "l_ee");
    MoveItIKSolver solver_right_arm(planning_scene, "r_iiwa_arm", "r_ee");

    std::vector<ReachTest> tests ;

    uint count = 0;
    for (const auto &grasp : candidates)
    {
        const auto &trv = grasp.translation;
        const auto &rotv = grasp.rotation;
        count++;
        Eigen::Vector3d c(trv.x, trv.y, trv.z);
        Eigen::Quaterniond rot(rotv.w, rotv.x, rotv.y, rotv.z);
        Eigen::Matrix3d m = rot.toRotationMatrix();

       
        auto approach = m.col(0);
        auto binormal = m.col(1);

        if ( binormal.y() < 0 ) binormal = -binormal ;
        double hw = 0.5 * grasp.width;
        double hand_depth = grasp.depth;

        Vector3d left_tip = c + (hw + finger_width / 2) * binormal + approach * (hand_depth - depth_offset);
        Vector3d right_tip = c - (hw + finger_width / 2) * binormal + approach * (hand_depth - depth_offset);

        Matrix3d trot ;
        trot.col(1) = binormal ;
        trot.col(2) = approach ;
        trot.col(0) = binormal.cross(approach) ;

        Quaterniond qrot(trot) ;

        ReachTest test ;
        test.l_ = left_tip ;
        test.r_ = right_tip ; 
        test.rot_ = qrot ;
        test.candidate_ = count++ ;

        tests.emplace_back(test) ;
    }

    // left hand reachability

    moveit::core::RobotState state(planning_scene->getRobotModel()) ;
    auto jml = state.getJointModelGroup("l_iiwa_arm") ;
    auto rml = state.getJointModelGroup("r_iiwa_arm") ;
    state.setToDefaultValues("r_iiwa_arm", "upright") ;

    for( auto &test: tests )  {
        auto ls = solver_left_arm.solveIK(poseFromEigen(test.l_, test.rot_));

        if ( !ls.empty() ) {
            test.ls_ = ls ;
        }
    }

    // right hand reachability

    state.setToDefaultValues("l_iiwa_arm", "upright") ;
    for( auto &test: tests )  {
        if ( test.ls_.empty() ) continue ;
        auto rs = solver_right_arm.solveIK(poseFromEigen(test.r_, test.rot_));

        if ( !rs.empty() ) {
            test.rs_ = rs ;
        }
    }

    // test if found solutions are in self collision

    std::set<int> filtered_set ;

    for( auto &test: tests ) {
        if ( test.ls_.empty() || test.rs_.empty() ) continue ;
    
        state.setJointGroupPositions(jml, test.ls_) ;
        state.setJointGroupPositions(rml, test.rs_) ;
        state.update() ;

        if ( !planning_scene->isStateColliding(state) ) {
            filtered_set.insert(test.candidate_) ;
            const auto &grasp = candidates[test.candidate_] ;
            result.emplace_back(grasp.score, test.ls_, test.l_, test.rs_, test.r_, test.rot_);
        }
    }



    for( int idx: filtered_set ) {
        filtered.emplace_back(candidates[idx]) ;
    }

    std::sort(result.begin(), result.end(), [](const GraspCandidate &a, const GraspCandidate &b)
    { 
        return a.cost_ > b.cost_; 
    });

    if ( !result.empty() ) {
        const auto &r = result[0] ;

        moveit::core::RobotState state(planning_scene->getRobotModel()) ;

        auto jml = state.getJointModelGroup("l_iiwa_arm") ;
        auto rml = state.getJointModelGroup("r_iiwa_arm") ;

        state.setJointGroupPositions(jml, r.jl_) ;
        state.setJointGroupPositions(rml, r.jr_) ;
        state.update() ;

        moveit_msgs::msg::DisplayRobotState drs ;

        moveit::core::robotStateToRobotStateMsg(state, drs.state) ;

        robot_state_publisher_->publish(drs) ;
    }
}