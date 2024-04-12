#pragma once

#include <vector>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <functional>
#include <map>
#include <Eigen/Geometry>

namespace moveit
{
    namespace core
    {
        class RobotModel;
        typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
        class JointModelGroup;
        class RobotState;
    } // namespace core
} // namespace moveit

namespace planning_scene
{
    class PlanningScene;
    typedef std::shared_ptr<const PlanningScene> PlanningSceneConstPtr;
} // namespace planning_scene

namespace planning_scene_monitor
{
    class PlanningSceneMonitor;
    typedef std::shared_ptr<PlanningSceneMonitor> PlanningSceneMonitorPtr;
} 

class MoveItIKSolver
{
public:
    MoveItIKSolver(const planning_scene::PlanningSceneConstPtr &scene,  
        const std::string &planning_group, const std::string &ee_link, double dist_threshold = 0.0);

    std::vector<double> solveIK(const Eigen::Isometry3d &target,
                                             const std::map<std::string, double> &seed) const;
    std::tuple<std::vector<double>, double> solveIK(const Eigen::Isometry3d &target) const;

    std::vector<std::string> getJointNames() const;

    std::string getKinematicBaseFrame() const;

    moveit::core::RobotState getRobotState() const;

protected:

    bool isIKSolutionValid(moveit::core::RobotState *state, const moveit::core::JointModelGroup *jmg,
                           const double *ik_solution) const;

    static std::vector<double> extractSubset(const std::map<std::string, double>& input, const std::vector<std::string>& keys);


    const moveit::core::JointModelGroup *jmg_;
    const double distance_threshold_;
    std::string group_, ee_link_ ;

    planning_scene::PlanningSceneConstPtr planning_scene_;
    kinematics_metrics::KinematicsMetricsPtr kinematic_metrics_ ;
};

