#pragma once
#include <ompl/util/Console.h>
#include <ompl/util/ClassForward.h>


OMPL_CLASS_FORWARD(MultiRobotProblemDefinition);
OMPL_CLASS_FORWARD(Merger);
class Merger
{
public:
	Merger(MultiRobotProblemDefinitionPtr mrmp_pdef, std::string name = "Merger"):
		mrmp_pdef_(mrmp_pdef), name_(name){}
	virtual void mergeRobots(const int idx1, const int idx2) = 0;
protected:
	const MultiRobotProblemDefinitionPtr mrmp_pdef_;
	const std::string name_;
};
