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

class MoveItIKSolver
{
public:
    MoveItIKSolver(moveit::core::RobotModelConstPtr model, planning_scene::PlanningScenePtr pscene, const std::string &planning_group, double dist_threshold = 0.01);

    std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d &target,
                                             const std::map<std::string, double> &seed) const;
    std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d &target) const;

    std::vector<std::string> getJointNames() const;

    std::string getKinematicBaseFrame() const;

protected:
    bool isIKSolutionValid(moveit::core::RobotState *state, const moveit::core::JointModelGroup *jmg,
                           const double *ik_solution) const;

    static std::vector<double> extractSubset(const std::map<std::string, double>& input, const std::vector<std::string>& keys);

    moveit::core::RobotModelConstPtr model_;
    const moveit::core::JointModelGroup *jmg_;
    const double distance_threshold_;

    planning_scene::PlanningScenePtr scene_;
};

