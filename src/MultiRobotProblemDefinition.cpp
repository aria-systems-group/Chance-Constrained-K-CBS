#include "MultiRobotProblemDefinition.h"


MotionPlanningProblem::MotionPlanningProblem(const oc::SpaceInformationPtr si, const ob::ProblemDefinitionPtr pdef, const ConstraintRespectingPlannerPtr planner):
    si_(si), pdef_(pdef), planner_(planner) {};

oc::SpaceInformationPtr MotionPlanningProblem::getSpaceInformation()
{
    return si_;
}

ob::ProblemDefinitionPtr MotionPlanningProblem::getProblemDefinition()
{
    return pdef_;
}

ConstraintRespectingPlannerPtr MotionPlanningProblem::getPlanner()
{
    return planner_;
}

MultiRobotProblemDefinition::MultiRobotProblemDefinition(std::vector<MotionPlanningProblemPtr> mrmp_problem):
	ob::ProblemDefinition(mrmp_problem[0]->getSpaceInformation()), mrmp_problem_(mrmp_problem)
{
	// for (auto itr = mrmp_problem.begin(); itr != mrmp_problem.end(); itr++) {
	// 	// std::cout << *itr << std::endl;
	// }
}

void MultiRobotProblemDefinition::setMultiRobotInstance(InstancePtr &mrmp_instance)
{
	mrmp_instance_ = mrmp_instance;
}

void MultiRobotProblemDefinition::setMerger(MergerPtr &merger)
{
	merger_ = merger;
}

void MultiRobotProblemDefinition::setPlanValidator(PlanValidityCheckerPtr &validator)
{
	validator_ = validator;
}

const oc::SpaceInformationPtr MultiRobotProblemDefinition::getRobotSpaceInformationPtr(const int idx)
{
	return mrmp_problem_[idx]->getSpaceInformation();
}

const ob::ProblemDefinitionPtr MultiRobotProblemDefinition::getRobotProblemDefinitionPtr(const int idx)
{
	return mrmp_problem_[idx]->getProblemDefinition();;
}

const std::vector<MotionPlanningProblemPtr> MultiRobotProblemDefinition::getAllProblemInformation()
{
	return mrmp_problem_;
}

const MotionPlanningProblemPtr MultiRobotProblemDefinition::getRobotMotionPlanningProblemPtr(const int idx)
{
	return mrmp_problem_[idx];
}
