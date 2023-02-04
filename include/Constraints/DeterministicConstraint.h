#pragma once
#include "common.h"
#include "Constraints/Constraint.h"


class DeterministicConstraint: public Constraint
{
public:
	DeterministicConstraint(int agentIdx, std::vector<double> timeRange, std::vector <Polygon> shapes);
	~DeterministicConstraint();
	const std::vector<Polygon> getShapes() const {return shapes_;};
private:
	std::vector<Polygon> shapes_;
};
