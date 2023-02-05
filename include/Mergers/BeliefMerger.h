#pragma once
#include "common.h"
#include "Merger.h"
#include "MultiRobotProblemDefinition.h"


// Class that managed merging
class BeliefMerger: public Merger
{
public: 
	BeliefMerger(const MultiRobotProblemDefinitionPtr mrmp_pdef);

	void mergeRobots(const int idx1, const int idx2);
};