#pragma once
#include "utils/Instance.h"
#include "Mergers/Merger.h"
#include "PlanValidityCheckers/PlanValidityChecker.h"
#include "Planners/ConstraintRespectingBSST.h"
// #include <ompl/control/SpaceInformation.h>


namespace ob = ompl::base;
namespace oc = ompl::control;


OMPL_CLASS_FORWARD(MotionPlanningProblem);
class MotionPlanningProblem
{
public:
    MotionPlanningProblem(const oc::SpaceInformationPtr si, const ob::ProblemDefinitionPtr pdef, const PlannerPtr planner);

    oc::SpaceInformationPtr getSpaceInformation();
    ob::ProblemDefinitionPtr getProblemDefinition();
    PlannerPtr getPlanner();
    void replacePlanner(PlannerPtr new_p)
    {
    	planner_ = new_p;
    }

private:
    oc::SpaceInformationPtr si_;
    ob::ProblemDefinitionPtr pdef_;
    PlannerPtr planner_;
};

OMPL_CLASS_FORWARD(MultiRobotProblemDefinition);
class MultiRobotProblemDefinition: public ob::ProblemDefinition
{
public:
	MultiRobotProblemDefinition(std::vector<MotionPlanningProblemPtr> mrmp_problem);

	void setMultiRobotInstance(InstancePtr &mrmp_instance);

	void setMerger(MergerPtr &merger);

	void setPlanValidator(PlanValidityCheckerPtr &validator);

	const MergerPtr getMerger() const {return merger_;};

	const InstancePtr getInstance() const {return mrmp_instance_;};

    void setMultiRobotInstance(InstancePtr &mrmp_instance) const;

	const PlanValidityCheckerPtr getPlanValidator() const {return validator_;};

	const oc::SpaceInformationPtr getRobotSpaceInformationPtr(const int idx);

	const ob::ProblemDefinitionPtr getRobotProblemDefinitionPtr(const int idx);

	const std::vector<MotionPlanningProblemPtr> getAllProblemInformation();

	const MotionPlanningProblemPtr getRobotMotionPlanningProblemPtr(const int idx);

	 void replacePlanner(PlannerPtr old_planner, const int idx);

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
