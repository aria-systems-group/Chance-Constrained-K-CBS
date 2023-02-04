#pragma once
#include "common.h"


OMPL_CLASS_FORWARD(Conflict);
struct Conflict
{
	Conflict(int agent1Idx, int agent2Idx, int timeStep);
	const int agent1Idx_; 
	const int agent2Idx_;
	const int timeStep_;

	bool operator==(const Conflict &other) const;
};