#include "move_group_interface.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

#include <cvx/math/rng.hpp>

using namespace std;
using namespace Eigen;
using namespace cvx ;

RNG g_rng ;

static void randomizePose(Vector3d &t, Quaterniond &q, const std::vector<double> &tol) {
    Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2) ;

    Vector3d xyz_tol{tol[0], tol[1], tol[2]} ;
    Vector3d rpy_tol{tol[3], tol[4], tol[5]} ;

    Vector3d tmin = t - xyz_tol ;
    Vector3d tmax = t + xyz_tol ;

    Vector3d rmin, rmax ;
    rmin.x() = euler.x() - rpy_tol.x() ;
    rmax.x() = euler.x() + rpy_tol.x() ;

    rmin.y() = euler.y() - rpy_tol.y() ;
    rmax.y() = euler.y() + rpy_tol.y() ;

    rmin.z() = euler.z() - rpy_tol.z() ;
    rmax.z() = euler.z() + rpy_tol.z() ;

    double lower_bounds[6], upper_bounds[6] ;

    lower_bounds[0] = tmin.x() ; upper_bounds[0] = tmax.x() ;
    lower_bounds[1] = tmin.y() ; upper_bounds[1] = tmax.y() ;
    lower_bounds[2] = tmin.z() ; upper_bounds[2] = tmax.z() ;
    lower_bounds[3] = rmin.x() ; upper_bounds[3] = rmax.x() ;
    lower_bounds[4] = rmin.y() ; upper_bounds[4] = rmax.y() ;
    lower_bounds[5] = rmin.z() ; upper_bounds[5] = rmax.z() ;

    double X = g_rng.uniform(lower_bounds[0], upper_bounds[0]) ;
    double Y = g_rng.uniform(lower_bounds[1], upper_bounds[1]) ;
    double Z = g_rng.uniform(lower_bounds[2], upper_bounds[2]) ;
    double r = g_rng.uniform(lower_bounds[3], upper_bounds[3]) ;
    double p = g_rng.uniform(lower_bounds[4], upper_bounds[4]) ;
    double y = g_rng.uniform(lower_bounds[5], upper_bounds[5]) ;

    q = AngleAxisd(r, Vector3d::UnitX()) * AngleAxisd(p, Vector3d::UnitY()) * AngleAxisd(y, Vector3d::UnitZ());
    t = Vector3d{X, Y, Z} ;

}

MoveGroupInterfaceNode::MoveGroupInterfaceNode(const rclcpp::NodeOptions &options) : rclcpp::Node("move_group_interface", options)
{
}

void MoveGroupInterfaceNode::setup()
{
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    move_group_interface_.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), "dual_arm"));
    //  planning_scene_monitor_->startWorldGeometryMonitor() ;
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
    robot_state_publisher_ = create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 10);
}

static Eigen::Isometry3d poseFromEigen(const Vector3d &c, const Quaterniond &r)
{
    Isometry3d t;
    t.setIdentity();
    t.linear() = r.toRotationMatrix();
    t.translation() = c;
    return t;
}

struct ReachTest
{
    Vector3d l_, r_;
    int candidate_;
    Quaterniond rot_;
    double lm_, rm_;
    std::vector<double> ls_, rs_;
};

void MoveGroupInterfaceNode::filterGrasps(const std::vector<grasp_planner_msgs::msg::Grasp> &candidates,
                                          const GraspCandidateFilterParams &params, bool tactile,
                                          std::vector<grasp_planner_msgs::msg::Grasp> &filtered,
                                          std::vector<GraspCandidate> &result)
{
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

    string prefix = (tactile) ? "_tactile" : "";

    MoveItIKSolver solver_left_arm(planning_scene, "iiwa_left_arm" + prefix, "iiwa_left" + prefix + "_ee", params.clearence_thresh_);
    MoveItIKSolver solver_right_arm(planning_scene, "iiwa_right_arm" + prefix, "iiwa_right" + prefix + "_ee", params.clearence_thresh_);

    std::vector<ReachTest> tests;

    const uint n_attempts = tactile ? params.n_attempts_6dof_ : params.n_attempts_7dof_ ;

    uint count = 0;
    for (const auto &grasp : candidates)
    {
        const auto &trv = grasp.translation;
        const auto &rotv = grasp.rotation;

        Eigen::Vector3d c(trv.x, trv.y, trv.z);
        Eigen::Quaterniond rot(rotv.w, rotv.x, rotv.y, rotv.z);

        for (uint k = 0; k < n_attempts; k++)
        {
            if (k > 0)
                randomizePose(c, rot, tactile ? params.tol_6dof_ : params.tol_7dof_ );

            Eigen::Matrix3d m = rot.toRotationMatrix();

            auto approach = m.col(0).eval();
            auto binormal = m.col(1).eval();
            auto xc = m.col(2).eval();

            if (binormal.y() < 0)
            {
                binormal = -binormal;
                xc = -xc;
            }
            double hw = 0.5 * grasp.width;
            double hand_depth = grasp.depth;

            Vector3d left_tip = c - (hw + params.finger_width_ / 2) * binormal + approach * (hand_depth - params.offset_);
            Vector3d right_tip = c + (hw + params.finger_width_ / 2) * binormal + approach * (hand_depth - params.offset_);

            Matrix3d trot;
            trot.col(0) = approach;
            trot.col(1) = binormal;
            trot.col(2) = xc;

            Quaterniond qrot(trot);

            ReachTest test;
            test.l_ = left_tip;
            test.r_ = right_tip;
            test.rot_ = trot;
            test.candidate_ = count++;

            tests.emplace_back(test);
        }
    }

    // left hand reachability

    moveit::core::RobotState state(planning_scene->getRobotModel());
    auto jml = state.getJointModelGroup("iiwa_left_arm");
    auto rml = state.getJointModelGroup("iiwa_right_arm");
    state.setToDefaultValues("iiwa_right_arm", "upright");

#ifdef DEBUG
    cout << "left arm" << endl;
#endif
#pragma omp parallel for
    for (size_t i = 0; i < tests.size(); i++)
    {
        auto &test = tests[i];
        auto [ls, manip] = solver_left_arm.solveIK(poseFromEigen(test.l_, test.rot_));

        if (!ls.empty())
        {
            if (tactile)
                ls.push_back(0);
#ifdef DEBUG
            for (int j = 0; j < ls.size(); j++)
                cout << ls[j] * 180 / M_PI << endl;
            cout << endl;
#endif
            test.ls_ = ls;
            test.lm_ = manip;
        }
    }

    // right hand reachability

    state.setToDefaultValues("iiwa_left_arm", "upright");

#ifdef DEBUG
    cout << "right arm" << endl;
#endif

#pragma omp parallel for
    for (size_t i = 0; i < tests.size(); i++)
    {
        auto &test = tests[i];
        if (test.ls_.empty())
            continue;
        auto [rs, manip] = solver_right_arm.solveIK(poseFromEigen(test.r_, test.rot_));

        if (!rs.empty())
        {
            if (tactile)
                rs.push_back(M_PI);
#ifdef DEBUG
            for (int j = 0; j < rs.size(); j++)
                cout << rs[j] * 180 / M_PI << endl;
            cout << endl;
#endif
            test.rs_ = rs;
            test.rm_ = manip;
        }
    }

    // test if found solutions are in self collision

    std::set<int> filtered_set;

    for (auto &test : tests)
    {
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
}

void MoveGroupInterfaceNode::visualizeResult(const GraspCandidate &r)
{
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);
    moveit::core::RobotState state(planning_scene->getRobotModel());

    auto jml = state.getJointModelGroup("iiwa_left_arm");
    auto rml = state.getJointModelGroup("iiwa_right_arm");

    state.setJointGroupPositions(jml, r.jl_);
    state.setJointGroupPositions(rml, r.jr_);
    state.update();

    moveit_msgs::msg::DisplayRobotState drs;

    moveit::core::robotStateToRobotStateMsg(state, drs.state);

    robot_state_publisher_->publish(drs);
}

bool MoveGroupInterfaceNode::jointStateToRobotState(const sensor_msgs::msg::JointState &joint_state, moveit::core::RobotState &state)
{
    if (joint_state.name.size() != joint_state.position.size())
    {
        RCLCPP_ERROR(get_logger(), "Different number of names and positions in JointState message: %u, %u",
                     (unsigned int)joint_state.name.size(), (unsigned int)joint_state.position.size());
        return false;
    }

    std::map<std::string, double> joint_state_map;
    for (unsigned int i = 0; i < joint_state.name.size(); ++i)
        joint_state_map[joint_state.name[i]] = joint_state.position[i];

    state.setVariablePositions(joint_state_map);
    return true;
}

void MoveGroupInterfaceNode::computeMotionPlans(std::vector<GraspCandidate> &candidates, const sensor_msgs::msg::JointState &start_joint_state, uint max_results)
{
    moveit::planning_interface::MoveGroupInterface move_group_interface(shared_from_this(), "dual_arm");
    move_group_interface.setPlannerId("RRTConnectkConfigDefault"); // default planener
    move_group_interface.setPlanningTime(5);
    move_group_interface.setGoalTolerance(0.01);
    move_group_interface.setMaxAccelerationScalingFactor(1);
    move_group_interface.setMaxVelocityScalingFactor(1);
    move_group_interface.setNumPlanningAttempts(10);

    if (start_joint_state.position.empty())
        move_group_interface.setStartStateToCurrentState();
    else
    {
        moveit::core::RobotState start_state(move_group_interface.getRobotModel());
        if (!jointStateToRobotState(start_joint_state, start_state))
            return;

        move_group_interface.setStartState(start_state);
    }

    uint count = 0;
    for (auto &grasp : candidates)
    {
        moveit::core::RobotState state(move_group_interface.getRobotModel());

        auto jml = state.getJointModelGroup("iiwa_left_arm");
        auto rml = state.getJointModelGroup("iiwa_right_arm");

        state.setJointGroupPositions(jml, grasp.jl_);
        state.setJointGroupPositions(rml, grasp.jr_);
        state.update();

        move_group_interface.setJointValueTarget(state);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (static_cast<bool>(move_group_interface.plan(plan)))
        {
            grasp.start_state_ = plan.start_state_;
            grasp.trajectory_ = plan.trajectory_;

            ++count;
            if (count == max_results)
                break;
            // move_group_interface.execute(plan) ;
        }
    }
}