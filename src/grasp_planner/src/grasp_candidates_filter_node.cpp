#include "grasp_candidates_filter_node.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

#include <cvx/math/rng.hpp>

#include <grasp_planner_msgs/msg/filtered_grasp.hpp>

using namespace std;
using namespace Eigen;
using namespace cvx;

RNG g_rng;

static void randomizePose(Vector3d &t, Quaterniond &q, const std::vector<double> &tol)
{
    Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    Vector3d xyz_tol{tol[0], tol[1], tol[2]};
    Vector3d rpy_tol{tol[3], tol[4], tol[5]};

    Vector3d tmin = t - xyz_tol;
    Vector3d tmax = t + xyz_tol;

    Vector3d rmin, rmax;
    rmin.x() = euler.x() - rpy_tol.x();
    rmax.x() = euler.x() + rpy_tol.x();

    rmin.y() = euler.y() - rpy_tol.y();
    rmax.y() = euler.y() + rpy_tol.y();

    rmin.z() = euler.z() - rpy_tol.z();
    rmax.z() = euler.z() + rpy_tol.z();

    double lower_bounds[6], upper_bounds[6];

    lower_bounds[0] = tmin.x();
    upper_bounds[0] = tmax.x();
    lower_bounds[1] = tmin.y();
    upper_bounds[1] = tmax.y();
    lower_bounds[2] = tmin.z();
    upper_bounds[2] = tmax.z();
    lower_bounds[3] = rmin.x();
    upper_bounds[3] = rmax.x();
    lower_bounds[4] = rmin.y();
    upper_bounds[4] = rmax.y();
    lower_bounds[5] = rmin.z();
    upper_bounds[5] = rmax.z();

    double X = g_rng.uniform(lower_bounds[0], upper_bounds[0]);
    double Y = g_rng.uniform(lower_bounds[1], upper_bounds[1]);
    double Z = g_rng.uniform(lower_bounds[2], upper_bounds[2]);
    double r = g_rng.uniform(lower_bounds[3], upper_bounds[3]);
    double p = g_rng.uniform(lower_bounds[4], upper_bounds[4]);
    double y = g_rng.uniform(lower_bounds[5], upper_bounds[5]);

    q = AngleAxisd(r, Vector3d::UnitX()) * AngleAxisd(p, Vector3d::UnitY()) * AngleAxisd(y, Vector3d::UnitZ());
    t = Vector3d{X, Y, Z};
}

GraspCandidatesFilterNode::GraspCandidatesFilterNode(const rclcpp::NodeOptions &options) : rclcpp::Node("grasp_candidates_filter", options)
{
    string viz_topic = declare_parameter("viz_topic", "/visual_grasps");
    grasps_rviz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(viz_topic, 10);

    declare_parameter("gripper_offset", -0.0f);
    declare_parameter("finger_width", 0.01f);
    declare_parameter("max_results", 3);
    declare_parameter("clearance", 0.01f);
    declare_parameter("n_attempts_6dof", 15);
    declare_parameter("n_attempts_7dof", 1);
    declare_parameter("tol_6dof", std::vector<double>{0.02, 0.02, 0.02, 0.05, 0.2, 0.05});
    declare_parameter("tol_7dof", std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01});

    service_ = create_service<GraspCandidatesFilterSrv>("grasp_candidates_filter_service", std::bind(&GraspCandidatesFilterNode::filter, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);
}

void GraspCandidatesFilterNode::setup()
{
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    //  move_group_interface_.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), "dual_arm"));
    //  planning_scene_monitor_->startWorldGeometryMonitor() ;
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
    //  robot_state_publisher_ = create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 10);
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
    ReachTest(int c, const Vector3d &l, const Vector3d &r, const Quaterniond &lq, const Quaterniond &rq):
        candidate_(c), l_(l), r_(r), lq_(lq), rq_(rq) {}

    Vector3d l_, r_;
    int candidate_;
    Quaterniond lq_, rq_;
    double lm_, rm_;
    std::vector<double> ls_, rs_;
};

extern visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
    const std::vector<grasp_planner_msgs::msg::Grasp> &hands,
    double hand_length, double finger_width, double finger_height,
    const std::string &frame_id, const Vector4f &clr, const std::string &ns);

void GraspCandidatesFilterNode::filterGrasps(const std::vector<grasp_planner_msgs::msg::Grasp> &candidates,
                                             const GraspCandidateFilterParams &params, bool l_tactile, bool r_tactile,
                                             std::vector<grasp_planner_msgs::msg::Grasp> &filtered,
                                             std::vector<GraspCandidate> &result)
{
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

    string prefix_l = (l_tactile) ? "_tactile" : "";
    string prefix_r = (r_tactile) ? "_tactile" : "";

    MoveItIKSolver solver_left_arm(planning_scene, "iiwa_left_arm" + prefix_l, "iiwa_left" + prefix_l + "_ee", params.clearence_thresh_);
    MoveItIKSolver solver_right_arm(planning_scene, "iiwa_right_arm" + prefix_r, "iiwa_right" + prefix_r + "_ee", params.clearence_thresh_);

    std::vector<ReachTest> tests;

    const uint n_attempts_l = l_tactile ? params.n_attempts_6dof_ : params.n_attempts_7dof_;
    const uint n_attempts_r = r_tactile ? params.n_attempts_6dof_ : params.n_attempts_7dof_;

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

      //  tests.emplace_back(count, left_tip, right_tip, qrot, qrot);
        
        for (uint k = 0; k < n_attempts_l; k++)
        {
            Vector3d lc(left_tip);
            Quaterniond lq(qrot) ;

            if ( k > 0 ) 
                randomizePose(lc, lq, l_tactile ? params.tol_6dof_ : params.tol_7dof_);

            for (uint r = 0; r < n_attempts_r; r++)
            {
                Vector3d rc(right_tip);
                Quaterniond rq(qrot) ;
            
                if ( r > 0 ) 
                    randomizePose(rc, rq, r_tactile ? params.tol_6dof_ : params.tol_7dof_);

                tests.emplace_back(count, lc, rc, lq, rq);
            }
        }

        ++count ;
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
        auto [ls, manip] = solver_left_arm.solveIK(poseFromEigen(test.l_, test.lq_));

        if (!ls.empty())
        {
            if (l_tactile)
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
        auto [rs, manip] = solver_right_arm.solveIK(poseFromEigen(test.r_, test.rq_));

        if (!rs.empty())
        {
            if (r_tactile)
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
            result.emplace_back(test.candidate_, grasp.score, test.ls_, test.lm_, test.l_, test.lq_,
                                test.rs_, test.rm_, test.r_, test.rq_);
        }
    }

    for (int idx : filtered_set)
    {
        filtered.emplace_back(candidates[idx]);
    }
}

void GraspCandidatesFilterNode::visualizeResult(const GraspCandidate &r)
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

bool GraspCandidatesFilterNode::jointStateToRobotState(const sensor_msgs::msg::JointState &joint_state, moveit::core::RobotState &state)
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

void GraspCandidatesFilterNode::filter(const std::shared_ptr<GraspCandidatesFilterSrv::Request> request, std::shared_ptr<GraspCandidatesFilterSrv::Response> response)
{

    vector<grasp_planner_msgs::msg::Grasp> grasps_filtered;
    std::vector<GraspCandidate> results;

    size_t max_results = get_parameter("max_results").as_int();
    bool tactile_l = request->left_arm_ee == GraspCandidatesFilterSrv::Request::GRASP_WITH_TACTILE;
    bool tactile_r = request->right_arm_ee == GraspCandidatesFilterSrv::Request::GRASP_WITH_TACTILE;

    GraspCandidateFilterParams fparams;
    fparams.clearence_thresh_ = get_parameter("clearance").as_double();
    fparams.offset_ = get_parameter("gripper_offset").as_double();
    fparams.finger_width_ = get_parameter("finger_width").as_double();
    fparams.n_attempts_6dof_ = get_parameter("n_attempts_6dof").as_int();
    fparams.n_attempts_7dof_ = get_parameter("n_attempts_7dof").as_int();
    fparams.tol_6dof_ = get_parameter("tol_6dof").as_double_array();
    fparams.tol_7dof_ = get_parameter("tol_7dof").as_double_array();

    filterGrasps(request->grasps, fparams, tactile_l, tactile_r, grasps_filtered, results);
    grasps_rviz_pub_->publish(convertToVisualGraspMsg(grasps_filtered, 0.05, 0.01, 0.01, "world", {1, 0, 0.0f, 0.5f}, "filtered"));

    // sort candidates
    std::sort(results.begin(), results.end(), [](const GraspCandidate &a, const GraspCandidate &b)
              {
                  double ma = a.lm_ + a.rm_, mb = b.lm_ + b.rm_;
                  return ma > mb; // sort based on manipulability
              });

    std::vector<grasp_planner_msgs::msg::FilteredGrasp> filtered_grasps ;

    for( uint i=0 ; i<std::min(max_results, results.size()) ; i++ ) {
        grasp_planner_msgs::msg::FilteredGrasp fg ;

        const GraspCandidate &result = results[i] ;
        fg.lc.x = result.lpos_.x() ;
        fg.lc.y = result.lpos_.y() ;
        fg.lc.z = result.lpos_.z() ;

        fg.lq.x = result.ldir_.x() ;
        fg.lq.y = result.ldir_.y() ;
        fg.lq.z = result.ldir_.z() ;
        fg.lq.w = result.ldir_.w() ;

        fg.rc.x = result.rpos_.x() ;
        fg.rc.y = result.rpos_.y() ;
        fg.rc.z = result.rpos_.z() ;

        fg.rq.x = result.rdir_.x() ;
        fg.rq.y = result.rdir_.y() ;
        fg.rq.z = result.rdir_.z() ;
        fg.rq.w = result.rdir_.w() ;

        fg.jl = result.jl_ ;
        fg.jr = result.jr_ ;

        response->grasps.emplace_back(fg) ;
    }

    response->result = (response->grasps.empty()) ? GraspCandidatesFilterSrv::Response::RESULT_NOT_FOUND : GraspCandidatesFilterSrv::Response::RESULT_OK;
    
}