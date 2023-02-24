#include "MultiRobotProblemDefinition.h"


MotionPlanningProblem::MotionPlanningProblem(const oc::SpaceInformationPtr si, const ob::ProblemDefinitionPtr pdef, const PlannerPtr planner):
    si_(si), pdef_(pdef), planner_(planner) {};

oc::SpaceInformationPtr MotionPlanningProblem::getSpaceInformation()
{
    return si_;
}

ob::ProblemDefinitionPtr MotionPlanningProblem::getProblemDefinition()
{
    return pdef_;
}

PlannerPtr MotionPlanningProblem::getPlanner()
{
    return planner_;
}


MultiRobotProblemDefinition::MultiRobotProblemDefinition(std::vector<MotionPlanningProblemPtr> mrmp_problem):
	ob::ProblemDefinition(mrmp_problem[0]->getSpaceInformation()), mrmp_problem_(mrmp_problem) {}

void MultiRobotProblemDefinition::replacePlanner(PlannerPtr old_planner, const int idx)
{
	// create another planner instance based on pre-computed spaceInformation and problem definiton
	// NOTE: this is problem specific
	std::string ll_solver = mrmp_instance_->getLowLevelPlannerName();
	Robot *r = mrmp_instance_->getRobots()[idx];
	auto si = getRobotSpaceInformationPtr(idx);
	auto pdef = getRobotProblemDefinitionPtr(idx);
	std::string dynamics_model = r->getDynamicsModel();
	if (ll_solver == "BSST") {
		// create (and provide) the low-level motion planner object
        PlannerPtr planner(std::make_shared<oc::ConstraintRespectingBSST>(si));
        planner->as<oc::ConstraintRespectingBSST>()->setProblemDefinition(pdef);
        planner->as<oc::ConstraintRespectingBSST>()->setPlanValidator(validator_);
        planner->as<oc::ConstraintRespectingBSST>()->setup();
        mrmp_problem_[idx]->replacePlanner(planner);
	}
	else {
		OMPL_ERROR("%s: You must add the ability to replace a planner!", "MultiRobotProblemDefinition");
	}
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
	for (MotionPlanningProblemPtr &mp: mrmp_problem_) {
		mp->getPlanner()->as<ConstraintRespectingPlanner>()->setPlanValidator(validator_);
	}
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
