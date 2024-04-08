#pragma once

#include <vector>
#include <moveit_msgs/msg/planning_scene.hpp>
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
    typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
} // namespace planning_scene

namespace planning_scene_monitor
{
    class PlanningSceneMonitor;
    typedef std::shared_ptr<PlanningSceneMonitor> PlanningSceneMonitorPtr;
} 

class MoveItIKSolver
{
public:
    MoveItIKSolver(const planning_scene_monitor::PlanningSceneMonitorPtr &model,  
        const std::string &planning_group, const std::string &ee_link, double dist_threshold = 0.01);

    std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d &target,
                                             const std::map<std::string, double> &seed) const;
    std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d &target) const;

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

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};

