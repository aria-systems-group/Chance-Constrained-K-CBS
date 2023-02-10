#pragma once
#include "Constraints/Constraint.h"
#include <ompl/base/State.h>

namespace ob = ompl::base;


class BeliefConstraint: public Constraint
{
public:
	BeliefConstraint(int constrained_agent, int constraining_agent, std::vector<double> timeRange, std::vector<ob::State*> beliefStates);
	~BeliefConstraint();
	const std::vector<ob::State*> getStates() const;
private:
	std::vector<ob::State*> belief_states_;
};
