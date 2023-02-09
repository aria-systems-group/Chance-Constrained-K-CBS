#include "PlanValidityCheckers/DeterministicPlanValidityChecker.h"

DeterministicPlanValidityChecker::DeterministicPlanValidityChecker(MultiRobotProblemDefinitionPtr pdef):
	PlanValidityChecker(pdef, "DeterministicPlanValidityChecker") {};

std::vector<ConflictPtr> DeterministicPlanValidityChecker::validatePlan(Plan p)
{
	std::vector<ConflictPtr> confs{};
	const double step_duration = mrmp_pdef_->getSystemStepSize();

	// interpolate all trajectories to the same discretization and get max size
	int maxStates = 0;
	for (auto itr = p.begin(); itr != p.end(); itr++) {
		itr->interpolate();
		if (maxStates < itr->getStateCount())
			maxStates = itr->getStateCount(); 
	}

	for (int k = 0; k < maxStates; k++) {
		// get shapes and indices of active robots
		std::vector<std::pair<int, Polygon>> activeRobots = getActiveRobots_(p, k);
		ConflictPtr c = checkForConflicts_(activeRobots, k);
		// std::cout << c << std::endl;
		if (c) {
			// found initial conflict at step k
			// must continue to propogate forward until conflict is finished
			int step = k;
			while (c && step < maxStates) {
				// std::cout << "found: " << c << std::endl;
				confs.push_back(c);
				step++;
				activeRobots = getActiveRobots_(p, step, c->agent1Idx_, c->agent2Idx_);
				c = checkForConflicts_(activeRobots, step);
			}
			return confs;
		}
	}
	return confs;
}

ConstraintPtr DeterministicPlanValidityChecker::createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx)
{
	// const double step_duration = mrmp_pdef_->getSystemStepSize();
	// std::vector<double> times;
	// std::vector<Polygon> polys;
	// for (auto itr = conflicts.begin(); itr != conflicts.end(); itr++) {
	// 	times.push_back(((*itr)->timeStep_ * step_duration));
	// 	if ( (*itr)->timeStep_ <  p[robotIdx].getStates().size()) {
	// 		Polygon r = getShapeFromState_(p[robotIdx].getState((*itr)->timeStep_), robotIdx);
	// 		polys.push_back(r);
	// 	}
	// 	else {
	// 		Polygon r = getShapeFromState_(p[robotIdx].getStates().back(), robotIdx);
	// 		polys.push_back(r);
	// 	}
		
	// }
	// ConstraintPtr c = std::make_shared<DeterministicConstraint>(robotIdx, times, polys);
	OMPL_ERROR("Not implemented yet!");
	exit(-1);
	return nullptr;
}

ConflictPtr DeterministicPlanValidityChecker::checkForConflicts_(std::vector<std::pair<int, Polygon>> shapes, const int step)
{
	ConflictPtr c = nullptr;
	for (int ai = 0; ai < shapes.size(); ai++) {
		for (int aj = ai + 1; aj < shapes.size(); aj++) {
			if (! boost::geometry::disjoint(shapes[ai].second, shapes[aj].second)) {
				c = std::make_shared<Conflict>(ai, aj, step);
				return c;
			}
		}
	}
	return c;
}

std::vector<std::pair<int, Polygon>> DeterministicPlanValidityChecker::getActiveRobots_(Plan p, const int step, const int a1, const int a2)
{
	std::vector<std::pair<int, Polygon>> activeRobots{};

	// using no pre-defined agent pair, return vector of all robots
	if (a1 == -1 || a2 == -1) {
		int idx = 0;
		auto itr = p.begin();
		for(; itr < p.end(); itr++, idx++ )
		{
			if (step < itr->getStateCount())
			{
				Polygon robotShape = getShapeFromState_(itr->getState(step), idx);
				activeRobots.emplace_back(idx, robotShape);
			}
			else
			{
				Polygon robotShape = getShapeFromState_(itr->getStates().back(), idx);
				activeRobots.emplace_back(idx, robotShape);
			}
		}
	}
	else {
		// provided agent pair, return vector with only those two robots
		if (step < p[a1].getStateCount()) {
			Polygon robotShape1 = getShapeFromState_(p[a1].getState(step), a1);
			activeRobots.emplace_back(a1, robotShape1);
		}
		else {
			Polygon robotShape1 = getShapeFromState_(p[a1].getStates().back(), a1);
			activeRobots.emplace_back(a1, robotShape1);
		}
		if (step < p[a2].getStateCount()) {
			Polygon robotShape2 = getShapeFromState_(p[a2].getState(step), a2);
			activeRobots.emplace_back(a2, robotShape2);
		}
		else {
			Polygon robotShape2 = getShapeFromState_(p[a2].getStates().back(), a2);
			activeRobots.emplace_back(a2, robotShape2);
		}
	}
	return activeRobots;
}

Polygon DeterministicPlanValidityChecker::getShapeFromState_(ob::State *st, const int robotIdx)
{
	auto compState = st->as<ob::CompoundStateSpace::StateType>();
	auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
	const double cx = xyState->values[0];
	const double cy = xyState->values[1];
	const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;

	// beta test: use boost transform instread of manual polygon creation
    Polygon raw;
    Polygon result;
    Robot* r = mrmp_pdef_->getInstance()->getRobots()[robotIdx];
    if (r->getDynamicsModel() != "FirstOrderCar" && 
    	r->getDynamicsModel() != "SecondOrderCar" && 
    	r->getDynamicsModel() != "SecondOrderUnicycle") {
    		OMPL_WARN("%s: Dynamics Model may not be suitable for this PlanValidityChecker! Be sure to check.", name_.c_str());
    }
    Polygon initial = r->getShape();
    const double ix = r->getStartLocation().x_;
    const double iy = r->getStartLocation().y_;
    bg::correct(raw);
    bg::assign(raw, initial);
    bg::strategy::transform::matrix_transformer<double, 2, 2> xfrm(
             cos(theta), sin(theta), (cx - ix),
            -sin(theta), cos(theta), (cy - iy),
                      0,          0,  1);
    bg::transform(raw, result, xfrm);
    return result;
}
