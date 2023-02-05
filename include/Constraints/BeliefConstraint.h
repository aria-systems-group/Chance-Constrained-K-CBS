#pragma once
#include "common.h"
#include "Constraints/Constraint.h"
#include <ompl/base/State.h>

namespace ob = ompl::base;


class BeliefConstraint: public Constraint
{
public:
	BeliefConstraint(int agentIdx, std::vector<double> timeRange, std::vector<ob::State*> beliefStates);
	~BeliefConstraint();
	const std::vector<ob::State*> getStates() const;
private:
	std::vector<ob::State*> belief_states_;
};