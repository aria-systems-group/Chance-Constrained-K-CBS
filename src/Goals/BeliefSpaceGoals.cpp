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
    Eigen::Vector2d mu;
    Eigen::Matrix2d Sigma;
    double* vals = st->as<RealVectorBeliefSpace::StateType>()->values;
    Sigma = st->as<RealVectorBeliefSpace::StateType>()->getCovariance().block<2, 2>(0, 0);
    for (int d = 0; d < 2; d++) {
        mu[d] = vals[d];
    }
    std::pair<Eigen::Vector2d, Eigen::Matrix2d> b(mu, Sigma);
    return b;
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
    ob::Goal(si), threshold_(toll), p_safe_(p_safe), num_agents_(si->getStateSpace()->getDimension() / 2), goal_(Eigen::VectorXd::Zero(si->getStateSpace()->getDimension()))
{
    // fill goal vector
    const double* all_goal_values = goal->as<RealVectorBeliefSpace::StateType>()->values;
    for (int idx = 0; idx < si->getStateSpace()->getDimension(); idx++)
        goal_[idx] = all_goal_values[idx];

    // create goal half-planes
    for (auto a = 0; a < num_agents_; a++) {
        /* Save the goal as a 2d eigen vector */
        Eigen::Vector2d goal_a{goal_[2 * a], goal_[2 * a + 1]};
        /* Create a square Polygon object centered at goal with side length == toll */
        Point bott_left(    goal_a[0] - (threshold_ / 2),     goal_a[1] - (threshold_ / 2));
        Point bott_right(   goal_a[0] + (threshold_ / 2),     goal_a[1] - (threshold_ / 2));
        Point top_left(     goal_a[0] - (threshold_ / 2),     goal_a[1] + (threshold_ / 2));
        Point top_right(    goal_a[0] + (threshold_ / 2),     goal_a[1] + (threshold_ / 2));

        Polygon goal_poly;
        bg::append(goal_poly.outer(), bott_left);
        bg::append(goal_poly.outer(), bott_right);
        bg::append(goal_poly.outer(), top_right);
        bg::append(goal_poly.outer(), top_left);
        bg::append(goal_poly.outer(), bott_left);

        bg::correct(goal_poly);

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> hp = createHalfPlanes_(goal_poly);

        half_plane_map_[a] = hp;
    }
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> CentralizedCCGoal::createHalfPlanes_(Polygon combined_poly)
{
    /* Converts a Polygon to a set of halfplanes */
    auto exterior_points = bg::exterior_ring(combined_poly);

    Eigen::MatrixXd A(exterior_points.size()-1, 2);
    Eigen::MatrixXd B(exterior_points.size()-1, 1);

    if (exterior_points.size() != 5) {
        OMPL_WARN("%s: Generating Halfplanes currently supports closed rectangles. Please check the implementation.", "PCCBlackmoreSVC");
    }
    // for (int i=0; i<exterior_points.size(); i++) {
    //     double x1 = bg::get<0>(exterior_points[i]);
    //     double y1 = bg::get<1>(exterior_points[i]);
    //     std::cout << x1 << "," << y1 << std::endl; 
    // }
    for (int i=0; i<exterior_points.size()-1; i++)
    {
        double x1 = bg::get<0>(exterior_points[i]);
        double y1 = bg::get<1>(exterior_points[i]);
        double x2 = bg::get<0>(exterior_points[i+1]);
        double y2 = bg::get<1>(exterior_points[i+1]);

        double a = y2 - y1;
        double b = x1 - x2;
        double c = a * x1 + b * y1;

        A(i, 0) = a;
        A(i, 1) = b;
        B(i, 0) = c;
    }
    return {A, B};
}

bool CentralizedCCGoal::isSatisfied(const ob::State *st) const
{
    /* Determine if in goal region */
    const double* all_st_values = st->as<RealVectorBeliefSpace::StateType>()->values;
    const Eigen::MatrixXd all_sigmas = st->as<RealVectorBeliefSpace::StateType>()->sigma_;
    const Eigen::MatrixXd all_lambdas = st->as<RealVectorBeliefSpace::StateType>()->lambda_;

    for (int a = 0; a < num_agents_;  a++) {
        Eigen::Vector2d mu_a{all_st_values[2 * a], all_st_values[2 * a + 1]};
        Eigen::Matrix2d sigma_a = all_sigmas.block<2, 2>(2 * a, 2 * a);
        Eigen::Vector2d goal_a{goal_[2 * a], goal_[2 * a + 1]};
        if (!isSafe_(mu_a, sigma_a, goal_a, half_plane_map_.at(a), nullptr))
            return false;
    }
    return true;
}

bool CentralizedCCGoal::isSatisfied(const ob::State *st, double *distance) const
{
    
    /* Determine if in goal region */
    const double* all_st_values = st->as<RealVectorBeliefSpace::StateType>()->values;
    const Eigen::MatrixXd all_sigmas = st->as<RealVectorBeliefSpace::StateType>()->sigma_;
    // const Eigen::MatrixXd all_lambdas = st->as<RealVectorBeliefSpace::StateType>()->lambda_;

    bool result = true;

    for (int a = 0; a < num_agents_;  a++) {
        Eigen::Vector2d mu_a{all_st_values[2 * a], all_st_values[2 * a + 1]};
        Eigen::Matrix2d sigma_a = all_sigmas.block<2, 2>(2 * a, 2 * a);
        Eigen::Vector2d goal_a{goal_[2 * a], goal_[2 * a + 1]};
        if (!isSafe_(mu_a, sigma_a, goal_a, half_plane_map_.at(a), distance))
            result = false;
    }
    // std::cout << *distance << std::endl;
    return result;
}

bool CentralizedCCGoal::isSingleAgentSatisfied(const ob::State *st, const int idx) const
{
    /* Determine if in goal region */
    const double* all_st_values = st->as<RealVectorBeliefSpace::StateType>()->values;
    const Eigen::MatrixXd all_sigmas = st->as<RealVectorBeliefSpace::StateType>()->sigma_;

    Eigen::Vector2d mu_a{all_st_values[2 * idx], all_st_values[2 * idx + 1]};
    Eigen::Matrix2d sigma_a = all_sigmas.block<2, 2>(2 * idx, 2 * idx);
    Eigen::Vector2d goal_a{goal_[2 * idx], goal_[2 * idx + 1]};
    
    if (!isSafe_(mu_a, sigma_a, goal_a, half_plane_map_.at(idx), nullptr))
        return false;
    return true;
}

bool CentralizedCCGoal::isSafe_(const Eigen::Vector2d mu, const Eigen::Matrix2d sigma, const Eigen::Vector2d goal, std::pair<Eigen::MatrixXd, Eigen::MatrixXd> hp, double *distance) const
{
    Eigen::MatrixXd A = hp.first;
    Eigen::MatrixXd B = hp.second;

    /* Update Distance to goal */
    if (distance) {
        const double dx = (mu[0] - goal[0]);
        const double dy = (mu[1] - goal[1]);
        *distance += (dx*dx + dy*dy);
    }

    auto const n_rows = A.rows();
    for (int i = 0; i < n_rows; i++) {
        const double tmp = (A.row(i) * sigma * A.row(i).transpose()).value();
        const double Pv = sqrt(tmp);
        const double vbar = sqrt(2) * Pv * bm::erf_inv(1 - (2 * p_safe_));
        if( (A(i, 0) * mu[0] + A(i, 1) * mu[1] - B(i, 0) >= vbar) ) {
            return false;
        };
    }
    return true;
}
