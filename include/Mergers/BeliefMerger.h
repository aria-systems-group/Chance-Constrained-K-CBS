#pragma once
#include "Merger.h"


// Class that managed merging
class BeliefMerger: public Merger
{
public: 
	BeliefMerger(const MultiRobotProblemDefinitionPtr mrmp_pdef);

	void mergeRobots(const int idx1, const int idx2);
};