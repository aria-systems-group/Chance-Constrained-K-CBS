#include "Mergers/DeterministicMerger.h"

DeterministicMerger::DeterministicMerger(MultiRobotProblemDefinitionPtr mrmp_pdef):
	Merger(mrmp_pdef, "DeterministicMerger"){}

void DeterministicMerger::mergeRobots(const int idx1, const int idx2)
{
	OMPL_ERROR("%s: Unable to merge.", name_.c_str());
	exit(1);
}
