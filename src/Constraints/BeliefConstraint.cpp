#include "Constraints/BeliefConstraint.h"

BeliefConstraint::BeliefConstraint(int agentIdx, 
	std::vector<double> timeRange, std::vector<ob::State*> beliefStates):
		Constraint(agentIdx, timeRange), belief_states_(beliefStates) {}

BeliefConstraint::~BeliefConstraint(){}

const std::vector<ob::State*> BeliefConstraint::getStates() const 
{
	return belief_states_;
}
