#pragma once
#include "common.h"
#include "Instance.h"
#include "Mergers/Merger.h"
#include "PlanValidityCheckers/PlanValidityChecker.h"
#include "Planners/ConstraintRespectingPlanner.h"
#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/SpaceInformation.h>


OMPL_CLASS_FORWARD(MotionPlanningProblem);
class MotionPlanningProblem
{
public:
    MotionPlanningProblem(const oc::SpaceInformationPtr si, const ob::ProblemDefinitionPtr pdef, const ConstraintRespectingPlannerPtr planner);

    oc::SpaceInformationPtr getSpaceInformation();
    ob::ProblemDefinitionPtr getProblemDefinition();
    ConstraintRespectingPlannerPtr getPlanner();

private:
    oc::SpaceInformationPtr si_;
    ob::ProblemDefinitionPtr pdef_;
    ConstraintRespectingPlannerPtr planner_;
};

OMPL_CLASS_FORWARD(Merger);
OMPL_CLASS_FORWARD(MultiRobotProblemDefinition);
OMPL_CLASS_FORWARD(PlanValidityChecker);
class MultiRobotProblemDefinition: public ob::ProblemDefinition
{
public:
	MultiRobotProblemDefinition(std::vector<MotionPlanningProblemPtr> mrmp_problem);

	void setMultiRobotInstance(InstancePtr &mrmp_instance);

	void setMerger(MergerPtr &merger);

	void setPlanValidator(PlanValidityCheckerPtr &validator);

	const MergerPtr getMerger() const {return merger_;};

	const InstancePtr getInstance() const {return mrmp_instance_;};

	const PlanValidityCheckerPtr getPlanValidator() const {return validator_;};

	const oc::SpaceInformationPtr getRobotSpaceInformationPtr(const int idx);

	const ob::ProblemDefinitionPtr getRobotProblemDefinitionPtr(const int idx);

	const std::vector<MotionPlanningProblemPtr> getAllProblemInformation();

	const MotionPlanningProblemPtr getRobotMotionPlanningProblemPtr(const int idx);

	double getSystemStepSize()
	{
		const oc::SpaceInformationPtr siPtr = mrmp_problem_[0]->getSpaceInformation();
		const double sys_dt = siPtr->getPropagationStepSize();
		return sys_dt;
	}

private:
	std::vector<MotionPlanningProblemPtr> mrmp_problem_{};
	InstancePtr mrmp_instance_;
	MergerPtr merger_;
	PlanValidityCheckerPtr validator_;
};