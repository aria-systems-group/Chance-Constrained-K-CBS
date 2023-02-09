#include "Constraints/BeliefConstraint.h"

BeliefConstraint::BeliefConstraint(int constrained_agent, int constraining_agent, 
	std::vector<double> timeRange, std::vector<ob::State*> beliefStates):
		Constraint(constrained_agent, constraining_agent, timeRange), belief_states_(beliefStates) {}

BeliefConstraint::~BeliefConstraint(){}

const std::vector<ob::State*> BeliefConstraint::getStates() const 
{
	return belief_states_;
}
