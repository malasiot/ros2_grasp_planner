#include "move_group_interface.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

using namespace std;
using namespace Eigen;

MoveGroupInterfaceNode::MoveGroupInterfaceNode(const rclcpp::NodeOptions &options) : rclcpp::Node("move_group_interface", options){
}

bool MoveGroupInterfaceNode::setup() {
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    move_group_interface_.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), "dual_arm"));
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
    double lc_, rc_, lm_, rm_ ;
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

#pragma omp parallel for
    for( size_t i = 0 ; i<tests.size() ; i++ )  {
        auto &test = tests[i] ;
        auto [ls, clearance, manip] = solver_left_arm.solveIK(poseFromEigen(test.l_, test.rot_));

        if ( !ls.empty() ) {
            test.ls_ = ls ;
            test.lc_ = clearance ;
            test.lm_ = manip ;
        }
    }

    // right hand reachability

    state.setToDefaultValues("l_iiwa_arm", "upright") ;
#pragma omp parallel for
    for( size_t i = 0 ; i<tests.size() ; i++ )  {
        auto &test = tests[i] ;
        if ( test.ls_.empty() ) continue ;
        auto [rs, clearance, manip] = solver_right_arm.solveIK(poseFromEigen(test.r_, test.rot_));

        if ( !rs.empty() ) {
            test.rs_ = rs ;
            test.rc_ = clearance ;
            test.rm_ = manip ;
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
            RCLCPP_INFO(get_logger(), "Left hand metrics: Clearance: %f, Manipualibility: %f", test.lc_, test.lm_ );
            RCLCPP_INFO(get_logger(), "Right hand metrics: Clearance: %f, Manipualibility: %f", test.rc_, test.rm_ );
            filtered_set.insert(test.candidate_) ;
            const auto &grasp = candidates[test.candidate_] ;
            result.emplace_back(grasp.score, test.ls_, test.lc_, test.lm_, test.l_, 
                                             test.rs_, test.rc_, test.rm_, test.r_, test.rot_);
        }
    }



    for( int idx: filtered_set ) {
        filtered.emplace_back(candidates[idx]) ;
    }

    std::sort(result.begin(), result.end(), [](const GraspCandidate &a, const GraspCandidate &b)
    { 
        double ca = a.lc_ + a.rc_, cb =  b.lc_ + b.rc_  ;
        double ma = a.lm_ + a.rm_, mb = b.lm_ + b.rm_ ;

        return ( ca > cb ) && (ma > mb ) ; // sort based on largest clearing and manipulability
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

void MoveGroupInterfaceNode::computeMotionPlans(std::vector<GraspCandidate> &candidates, uint max_results) {
    
    uint count = 0 ;
    for( auto &grasp: candidates ) {
        moveit::planning_interface::MoveGroupInterface move_group_interface(shared_from_this(), "dual_arm");
        move_group_interface.setPlannerId("RRTConnectkConfigDefault"); //default planener
        move_group_interface.setPlanningTime(5);
        move_group_interface.setGoalTolerance(0.01);
        move_group_interface.setMaxAccelerationScalingFactor(1);
        move_group_interface.setMaxVelocityScalingFactor(1);
        move_group_interface.setNumPlanningAttempts(10);
        
        moveit::core::RobotState state(move_group_interface.getRobotModel()) ;

        auto jml = state.getJointModelGroup("l_iiwa_arm") ;
        auto rml = state.getJointModelGroup("r_iiwa_arm") ;

        state.setJointGroupPositions(jml, grasp.jl_) ;
        state.setJointGroupPositions(rml, grasp.jr_) ;
        state.update() ;

        move_group_interface.setJointValueTarget(state) ;
                
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if ( static_cast<bool>(move_group_interface.plan(plan)) ) {
            grasp.start_state_ = plan.start_state_ ;
            grasp.trajectory_ = plan.trajectory_ ;

            ++count ;
            if ( count == max_results ) break ;
            //move_group_interface.execute(plan) ;
        }

    }
    
}