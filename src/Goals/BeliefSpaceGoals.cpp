#include "Goals/BeliefSpaceGoals.h"


ChanceConstrainedGoal::ChanceConstrainedGoal(const oc::SpaceInformationPtr &si, const Location goal, const double toll, const double p_safe):
	ob::Goal(si), threshold_(toll), p_safe_(p_safe), A_(4, 2), B_(4, 1), goal_(2, 1)
{
	/* Save the goal as a 2d eigen vector */
	goal_(0, 0) = goal.x_;
	goal_(1, 0) = goal.y_;
	/* Create a square Polygon object centered at goal with side length == toll */
    Point bott_left(	goal.x_ - (threshold_ / 2), 	goal.y_ - (threshold_ / 2));
    Point bott_right(	goal.x_ + (threshold_ / 2), 	goal.y_ - (threshold_ / 2));
    Point top_left(		goal.x_ - (threshold_ / 2), 	goal.y_ + (threshold_ / 2));
    Point top_right(	goal.x_ + (threshold_ / 2), 	goal.y_ + (threshold_ / 2));

    Polygon goal_poly;
    bg::append(goal_poly.outer(), bott_left);
    bg::append(goal_poly.outer(), bott_right);
    bg::append(goal_poly.outer(), top_right);
    bg::append(goal_poly.outer(), top_left);
    bg::append(goal_poly.outer(), bott_left);

    // assures that the polygon (1) has counter-clockwise points, and (2) is closed
    bg::correct(goal_poly);

    // set-up half-planes
    setHalfPlanes_(goal_poly);
}

bool ChanceConstrainedGoal::isSatisfied(const ob::State *st) const
{
	/* Determine if in goal region */
	return isSafe_(st);
}

bool ChanceConstrainedGoal::isSatisfied(const ob::State *st, double *distance) const
{
	/* Determine if in goal region - update distance while there */
	return isSafe_(st, distance);
}

void ChanceConstrainedGoal::setHalfPlanes_(Polygon combined_poly)
{
	/* Converts a Polygon to a set of halfplanes */
	auto exterior_points = bg::exterior_ring(combined_poly);
	if (exterior_points.size() != 5) {
		OMPL_WARN("%s: Generating Halfplanes currently supports closed rectangles. Please check the implementation.", "PCCBlackmoreSVC");
	}
    for (int i=0; i<exterior_points.size()-1; i++)
    {
        double x1 = bg::get<0>(exterior_points[i]);
        double y1 = bg::get<1>(exterior_points[i]);
        double x2 = bg::get<0>(exterior_points[i+1]);
        double y2 = bg::get<1>(exterior_points[i+1]);

        double a = y2 - y1;
        double b = x1 - x2;
        double c = a * x1 + b * y1;

        A_(i, 0) = a;
        A_(i, 1) = b;
        B_(i, 0) = c;
    }
}

std::pair<Eigen::Vector2d, Eigen::Matrix2d> ChanceConstrainedGoal::getDistFromState_(const ob::State* st) const
{
	Eigen::Vector2d mu = st->as<R2BeliefSpace::StateType>()->getXY();
	// std::cout << "mu: " << mu << std::endl;
	Eigen::Matrix2d Sigma = st->as<R2BeliefSpace::StateType>()->getCovariance();
	std::pair<Eigen::Vector2d, Eigen::Matrix2d> p(mu, Sigma);
	return p;
}

bool ChanceConstrainedGoal::isSafe_(const ob::State* st, double *distance) const
{
	/* Calculate mu_ab */
	std::pair<Eigen::Vector2d, Eigen::Matrix2d> belief = getDistFromState_(st);
	const Eigen::Vector2d mu_ab = belief.first;
	const Eigen::Matrix2d Sigma_ab = belief.second;

	/* Update Distance to goal */
	if (distance) {
		const double dx = (belief.first(0, 0) - goal_(0, 0));
		const double dy = (belief.first(1, 0) - goal_(1, 0));
		*distance = (dx*dx + dy*dy);
	}

	auto const n_rows = A_.rows();
	for (int i = 0; i < n_rows; i++) {
        const double tmp = (A_.row(i) * Sigma_ab * A_.row(i).transpose()).value();
		const double Pv = sqrt(tmp);
		const double vbar = sqrt(2) * Pv * bm::erf_inv(1 - (2 * p_safe_));
        if( (A_(i, 0) * mu_ab[0] + A_(i, 1) * mu_ab[1] - B_(i, 0) >= vbar) ) {
            return false;
        };
	}
    return true;
}

CentralizedCCGoal::CentralizedCCGoal(const oc::SpaceInformationPtr &si, const ob::State* goal, const double toll, const double p_safe):
    ob::Goal(si), threshold_(toll), p_safe_(p_safe)
{

}

bool CentralizedCCGoal::isSatisfied(const ob::State *st) const
{
    /* Determine if in goal region */
    return false;
}

bool CentralizedCCGoal::isSatisfied(const ob::State *st, double *distance) const
{
    /* Determine if in goal region - update distance while there */
    return false;
}
