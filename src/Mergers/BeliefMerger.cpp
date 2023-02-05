#include "Mergers/BeliefMerger.h"

BeliefMerger::BeliefMerger(MultiRobotProblemDefinitionPtr mrmp_pdef):
	Merger(mrmp_pdef, "BeliefMerger"){}

void BeliefMerger::mergeRobots(const int idx1, const int idx2)
{
	OMPL_ERROR("%s: Unable to merge.", name_.c_str());
	exit(1);
}