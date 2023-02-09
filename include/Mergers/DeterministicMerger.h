#pragma once
#include "common.h"
#include "Merger.h"


// Class that managed merging
class DeterministicMerger: public Merger
{
public: 
	DeterministicMerger(const MultiRobotProblemDefinitionPtr mrmp_pdef);

	void mergeRobots(const int idx1, const int idx2);
};
