#include "ConstraintValidityCheckers/DeterministicCVC.h"

bool DeterministicCVC::satisfiesConstraints(oc::PathControl p, std::vector<ConstraintPtr> constraints)
{
	std::vector<ob::State*> states = p.getStates();
	for (auto itr = states.begin(); itr != states.end(); itr++) {
		Polygon robot_shape = getShapeFromState_(*itr);
		for (auto c_itr = constraints.begin(); c_itr != constraints.end(); c_itr++) {
			const std::vector<Polygon> c_itr_shapes = (*c_itr)->as<DeterministicConstraint>()->getShapes();
			for (auto c_shape_itr = c_itr_shapes.begin(); c_shape_itr != c_itr_shapes.end(); c_shape_itr++) {
				if (! boost::geometry::disjoint(robot_shape, *c_shape_itr))
					return false;
			}
		}
	}
	return true;
}

Polygon DeterministicCVC::getShapeFromState_(ob::State *st)
{
	auto compState = st->as<ob::CompoundStateSpace::StateType>();
	auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
	const double cx = xyState->values[0];
	const double cy = xyState->values[1];
	const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;

	// beta test: use boost transform instread of manual polygon creation
    Polygon raw;
    Polygon result;
    if (robot_->getDynamicsModel() != "FirstOrderCar" && 
    	robot_->getDynamicsModel() != "SecondOrderCar" && 
    	robot_->getDynamicsModel() != "SecondOrderUnicycle") {
    		OMPL_WARN("%s: Dynamics Model may not be suitable for this PlanValidityChecker! Be sure to check.", name_.c_str());
    }
    Polygon initial = robot_->getShape();
    const double ix = robot_->getStartLocation().x_;
    const double iy = robot_->getStartLocation().y_;
    bg::correct(raw);
    bg::assign(raw, initial);
    bg::strategy::transform::matrix_transformer<double, 2, 2> xfrm(
             cos(theta), sin(theta), (cx - ix),
            -sin(theta), cos(theta), (cy - iy),
                      0,          0,  1);
    bg::transform(raw, result, xfrm);
    return result;
}
