#include "reachability_node.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

using namespace std;
using namespace Eigen;

ReachabilityNode::ReachabilityNode(const rclcpp::NodeOptions &options) : rclcpp::Node("reachability_node", options)
{
     declare_parameter("volume", std::vector<double>{});
     declare_parameter("cell_size", 0.02);
}

void ReachabilityNode::setup()
{
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
}  

static Eigen::Isometry3d poseFromEigen(const Vector3d &c, const Quaterniond &r)
{
    Isometry3d t;
    t.setIdentity();
    t.linear() = r.toRotationMatrix();
    t.translation() = c;
    return t;
}

void ReachabilityNode::doTest()
{
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

    MoveItIKSolver solver_left_arm(planning_scene, "iiwa_left_arm", "iiwa_left_ee", dist_thresh_);
    MoveItIKSolver solver_right_arm(planning_scene, "iiwa_right_arm", "iiwa_right_ee", dist_thresh_);

    vector<double> cutoff = get_parameter("volume").as_double_array() ;
    double cell_size = get_parameter("cell_size").as_double() ;
    
    float minx = cutoff[0] ;
    float maxx = cutoff[1] ;
    float miny = cutoff[2] ;
    float maxy = cutoff[3] ;
    float minz = cutoff[4] ;
    float maxz = cutoff[5] ;

    moveit::core::RobotState state(planning_scene->getRobotModel());
    auto jml = state.getJointModelGroup("iiwa_left_arm");
    auto rml = state.getJointModelGroup("iiwa_right_arm");
    state.setToDefaultValues("iiwa_right_arm", "upright");

    for( double x = minx ; x <= maxx ; x += cell_size ) {
        for( double y = miny ; x <= maxy ; y += cell_size ) {
            for( double z = minz ; z <= maxz ; z += cell_size ) {
                Vector3d c{x, y, z} ;

                Eigen::Matrix3d m ;
                 m << 0, 0, -1, 
                      0, -1, 0,
                      -1, 0, 0 ;
                Quaterniond qrot(m);
                auto [ls, manip] = solver_left_arm.solveIK(poseFromEigen(c, qrot));

                cout << c.adjoint() << endl ;
                if (!ls.empty())
                {       
                    for( int i=0 ; i<ls.size() ; i++ )
                        cout << ls[i] * 180/M_PI << ' ' ;
                    cout << endl ;
                }
            }
        }
    }
/*
    std::vector<ReachTest> tests;

    uint count = 0;
    for (const auto &grasp : candidates)
    {
        const auto &trv = grasp.translation;
        const auto &rotv = grasp.rotation;

        Eigen::Vector3d c(trv.x, trv.y, trv.z);
        Eigen::Quaterniond rot(rotv.w, rotv.x, rotv.y, rotv.z);
        Eigen::Matrix3d m = rot.toRotationMatrix();
       
        auto approach = m.col(0).eval();
        auto binormal = m.col(1).eval();
        auto xc = m.col(2).eval() ;

        if (binormal.y() < 0) {
            binormal = -binormal;
            xc = -xc ;
        }
        double hw = 0.5 * grasp.width;
        double hand_depth = grasp.depth;

        Vector3d left_tip = c - (hw + finger_width / 2) * binormal + approach * (hand_depth - depth_offset);
        Vector3d right_tip = c + (hw + finger_width / 2) * binormal + approach * (hand_depth - depth_offset);

        Matrix3d trot;
        trot.col(0) = approach;
        trot.col(1) = binormal;
        trot.col(2) = xc ;

        Quaterniond qrot(trot);

        ReachTest test;
        test.l_ = left_tip;
        test.r_ = right_tip;
        test.rot_ = trot;
        test.candidate_ = count++;

        tests.emplace_back(test);
    }

    // left hand reachability

    moveit::core::RobotState state(planning_scene->getRobotModel());
    auto jml = state.getJointModelGroup("iiwa_left_arm");
    auto rml = state.getJointModelGroup("iiwa_right_arm");
    state.setToDefaultValues("iiwa_right_arm", "upright");

#pragma omp parallel for
    for (size_t i = 0; i < tests.size(); i++)
    {
        auto &test = tests[i];
        auto [ls, manip] = solver_left_arm.solveIK(poseFromEigen(test.l_, test.rot_));

        if (!ls.empty())
        {
            test.ls_ = ls;
            test.lm_ = manip;
        }
    }

    // right hand reachability

    state.setToDefaultValues("iiwa_left_arm", "upright");
#pragma omp parallel for
    for (size_t i = 0; i < tests.size(); i++)
    {
        auto &test = tests[i];
        if (test.ls_.empty())
            continue;
        auto [rs, manip] = solver_right_arm.solveIK(poseFromEigen(test.r_, test.rot_));

        if (!rs.empty())  {
            test.rs_ = rs;
            test.rm_ = manip;
        }
    }

    // test if found solutions are in self collision

    std::set<int> filtered_set;

    for (auto &test : tests)  {
        if (test.ls_.empty() || test.rs_.empty())
            continue;

        state.setJointGroupPositions(jml, test.ls_);
        state.setJointGroupPositions(rml, test.rs_);
        state.update();

        if (!planning_scene->isStateColliding(state))
        {
            RCLCPP_DEBUG(get_logger(), "Left hand metrics: Manipualibility: %f", test.lm_);
            RCLCPP_DEBUG(get_logger(), "Right hand metrics: Manipualibility: %f", test.rm_);
            filtered_set.insert(test.candidate_);
            const auto &grasp = candidates[test.candidate_];
            result.emplace_back(grasp.score, test.ls_, test.lm_, test.l_,
                                test.rs_, test.rm_, test.r_, test.rot_);
        }
    }

    for (int idx : filtered_set)
    {
        filtered.emplace_back(candidates[idx]);
    }

    std::sort(result.begin(), result.end(), [](const GraspCandidate &a, const GraspCandidate &b)
              {
                  double ma = a.lm_ + a.rm_, mb = b.lm_ + b.rm_;

                  return ma > mb; // sort based on manipulability
              });

    if (!result.empty())
    {
        const auto &r = result[0];

        moveit::core::RobotState state(planning_scene->getRobotModel());

        auto jml = state.getJointModelGroup("iiwa_left_arm");
        auto rml = state.getJointModelGroup("iiwa_right_arm");

        state.setJointGroupPositions(jml, r.jl_);
        state.setJointGroupPositions(rml, r.jr_);
        state.update();

        moveit_msgs::msg::DisplayRobotState drs;

        moveit::core::robotStateToRobotStateMsg(state, drs.state);

        robot_state_publisher_->publish(drs);
    }*/
}
