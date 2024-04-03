#include "moveit_ik.hpp"

#include <moveit/planning_scene/planning_scene.h>

using namespace std::placeholders;

MoveItIKSolver::MoveItIKSolver(moveit::core::RobotModelConstPtr model, planning_scene::PlanningScenePtr scene, 
    const std::string &planning_group,  double dist_threshold)
    : model_(model), scene_(scene), jmg_(model_->getJointModelGroup(planning_group)), distance_threshold_(dist_threshold)
{
    if (!jmg_)
        throw std::runtime_error("Failed to initialize joint model group for planning group '" + planning_group + "'");
    if (!jmg_->getSolverInstance())
        throw std::runtime_error("No kinematics solver instantiated for planning group '" + planning_group +
                                 "'. Check that the 'kinematics.yaml' file was loaded as a parameter");
}

std::vector<std::vector<double>> MoveItIKSolver::solveIK(const Eigen::Isometry3d &target,
                                                         const std::map<std::string, double> &seed) const {
    moveit::core::RobotState state(model_);

    const std::vector<std::string> &joint_names = jmg_->getActiveJointModelNames();

    std::vector<double> seed_subset = extractSubset(seed, joint_names);
    state.setJointGroupPositions(jmg_, seed_subset);
    state.update();

    if (state.setFromIK(jmg_, target, 0.0,
                        std::bind(&MoveItIKSolver::isIKSolutionValid, this, std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3)))
    {
        std::vector<double> solution;
        state.copyJointGroupPositions(jmg_, solution);

        return {solution};
    }

    return {};
}

std::vector<std::vector<double>> MoveItIKSolver::solveIK(const Eigen::Isometry3d &target) const {
    moveit::core::RobotState state(model_);

    const std::vector<std::string> &joint_names = jmg_->getActiveJointModelNames();

    if ( state.setFromIK(jmg_, target, 0.0,
                        std::bind(&MoveItIKSolver::isIKSolutionValid, this, std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3))) {
        std::vector<double> solution;
        state.copyJointGroupPositions(jmg_, solution);

        return {solution};
    }

    return {};
}

bool MoveItIKSolver::isIKSolutionValid(moveit::core::RobotState *state, const moveit::core::JointModelGroup *jmg,
                                       const double *ik_solution) const
{
    state->setJointGroupPositions(jmg, ik_solution);
    state->update();

    const bool colliding = scene_->isStateColliding(*state, jmg->getName(), false);
    const bool too_close =
        (scene_->distanceToCollision(*state, scene_->getAllowedCollisionMatrix()) < distance_threshold_);

    return (!colliding && !too_close);
}

std::vector<std::string> MoveItIKSolver::getJointNames() const {
    return jmg_->getActiveJointModelNames();
}


std::string MoveItIKSolver::getKinematicBaseFrame() const {
    return jmg_->getSolverInstance()->getBaseFrame();
}

std::vector<double> MoveItIKSolver::extractSubset(const std::map<std::string, double>& input, const std::vector<std::string>& keys)
{
  if (keys.size() > input.size())
    throw std::runtime_error("Input map size was not at least as large as the number of keys");

  // Pull the joints of the planning group out of the input map
  std::vector<double> values;
  values.reserve(keys.size());
  for (const std::string& name : keys)
  {
    const auto it = input.find(name);
    if (it == input.end())
      throw std::runtime_error("Key '" + name + "' is not in the input map");

    values.push_back(it->second);
  }

  return values;
}
