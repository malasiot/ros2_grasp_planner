#include "moveit_ik.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

using namespace std::placeholders;

MoveItIKSolver::MoveItIKSolver(const planning_scene::PlanningSceneConstPtr &scene, 
    const std::string &planning_group,  const std::string &ee_link, double dist_threshold)
    : planning_scene_(scene), group_(planning_group), ee_link_(ee_link), distance_threshold_(dist_threshold)
{
   kinematic_metrics_.reset(new kinematics_metrics::KinematicsMetrics(scene->getRobotModel()));
}


moveit::core::RobotState MoveItIKSolver::getRobotState() const {
    moveit::core::RobotState rs = planning_scene_->getCurrentState();
    rs.update();
    return rs;
}

std::vector<double> MoveItIKSolver::solveIK(const Eigen::Isometry3d &target,
                                                         const std::map<std::string, double> &seed) const {
  /*  moveit::core::RobotState state(model_);

    const std::vector<std::string> &joint_names = jmg_->getActiveJointModelNames();

    std::vector<double> seed_subset = extractSubset(seed, joint_names);
    state.setJointGroupPositions(jmg_, seed_subset);
    state.update();

    if (state.setFromIK(jmg_, target, ee_link_, 0.0,
                        std::bind(&MoveItIKSolver::isIKSolutionValid, this, std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3)))
    {
        std::vector<double> solution;
        state.copyJointGroupPositions(jmg_, solution);

        return {solution};
    }*/

    return {};
}

std::tuple<std::vector<double>, double, double> MoveItIKSolver::solveIK(const Eigen::Isometry3d &target) const {
    moveit::core::RobotState state = getRobotState() ;
    auto jmg = state.getJointModelGroup(group_) ;

    std::vector<double> values ;
    state.copyJointGroupPositions(jmg, values) ;
    const std::vector<std::string> &joint_names = jmg->getActiveJointModelNames();

    kinematics::KinematicsQueryOptions options ;
    options.discretization_method = kinematics::DiscretizationMethods::SOME_DISCRETIZED ;
    if ( state.setFromIK(jmg, target, ee_link_, 0.05,
                        std::bind(&MoveItIKSolver::isIKSolutionValid, this, std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3), options)) {
        std::vector<double> solution;
        state.copyJointGroupPositions(jmg, solution);

        double condition_number, clearance ;
        kinematic_metrics_->getManipulability(state, jmg, condition_number) ;
        clearance = planning_scene_->distanceToCollision(state, planning_scene_->getAllowedCollisionMatrix()) ;
      
        return { solution, clearance, condition_number } ;
    }

    return {};
}

bool MoveItIKSolver::isIKSolutionValid(moveit::core::RobotState *state, const moveit::core::JointModelGroup *jmg,
                                       const double *ik_solution) const
{
    state->setJointGroupPositions(jmg, ik_solution);
    state->update();

    if ( !planning_scene_->isStateColliding(*state, jmg->getName(), false) ) {
      //  if (planning_scene_->distanceToCollision(*state, planning_scene_->getAllowedCollisionMatrix()) < distance_threshold_) 
      //    return true ;
      return true ;
    }

    return false ;
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
