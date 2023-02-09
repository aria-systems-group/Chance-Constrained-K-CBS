#include "Constraints/DeterministicConstraint.h"


DeterministicConstraint::DeterministicConstraint(int constrained_agent, int constraining_agent, 
	std::vector<double> timeRange, std::vector <Polygon> shapes):
		Constraint(constrained_agent, constraining_agent, timeRange), shapes_(shapes) {}

DeterministicConstraint::~DeterministicConstraint()
{
	shapes_.clear();
}
