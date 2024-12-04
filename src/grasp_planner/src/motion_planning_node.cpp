#include "motion_planning_node.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/planning_scene.hpp>


using namespace std;
using namespace Eigen;

MotionPlanningNode::MotionPlanningNode(const rclcpp::NodeOptions &options) : rclcpp::Node("motion_planning_node", options)
{
    service_ = create_service<MotionPlanningSrv>("motion_planning_service", std::bind(&MotionPlanningNode::plan, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);
}

void MotionPlanningNode::setup()
{
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    move_group_interface_.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), "dual_arm"));
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
}

bool MotionPlanningNode::jointStateToRobotState(const sensor_msgs::msg::JointState &joint_state, moveit::core::RobotState &state)
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

void MotionPlanningNode::plan(const std::shared_ptr<MotionPlanningSrv::Request> request, std::shared_ptr<MotionPlanningSrv::Response> response)
{
    moveit::planning_interface::MoveGroupInterface move_group_interface(shared_from_this(), "dual_arm");
    move_group_interface.setPlannerId("RRTConnectkConfigDefault"); // default planener
    move_group_interface.setPlanningTime(5);
    move_group_interface.setGoalTolerance(0.01);
    move_group_interface.setMaxAccelerationScalingFactor(1);
    move_group_interface.setMaxVelocityScalingFactor(1);
    move_group_interface.setNumPlanningAttempts(10);

    if (request->start_state.position.empty())
        move_group_interface.setStartStateToCurrentState();
    else
    {
        moveit::core::RobotState start_state(move_group_interface.getRobotModel());
        if (!jointStateToRobotState(request->start_state, start_state))
        {
            response->result = MotionPlanningSrv::Response::RESULT_ERROR;
            return;
        }

        move_group_interface.setStartState(start_state);
    }

    moveit::core::RobotState state(move_group_interface.getRobotModel());

    auto jml = state.getJointModelGroup("iiwa_left_arm");
    auto rml = state.getJointModelGroup("iiwa_right_arm");

    state.setJointGroupPositions(jml, request->left_arm_js);
    state.setJointGroupPositions(rml, request->right_arm_js);
    state.update();

    move_group_interface.setJointValueTarget(state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (static_cast<bool>(move_group_interface.plan(plan)))
    {
        response->result = MotionPlanningSrv::Response::RESULT_OK;
        response->traj = plan.trajectory_.joint_trajectory;
    }
    else
    {
        response->result = MotionPlanningSrv::Response::RESULT_NOT_FOUND;
    }
}