#include "Constraints/DeterministicConstraint.h"


DeterministicConstraint::DeterministicConstraint(int agentIdx, 
	std::vector<double> timeRange, std::vector <Polygon> shapes):
		Constraint(agentIdx, timeRange), shapes_(shapes) {}

DeterministicConstraint::~DeterministicConstraint()
{
	shapes_.clear();
}
