#include "ConstraintValidityCheckers/BeliefCVC.h"


BeliefCVC::BeliefCVC(Robot *robot, const int num_obs): ConstraintValidityChecker(robot, "BeliefCVC"),
    p_safe_(0.99), 
    p_coll_dist_( (1-p_safe_) / num_obs)
{
    // this can be generalized in the future based on robot shapes
    HalfPlanes_.push_back({1.0,0.0,0.5}) ; // right side
    HalfPlanes_.push_back({-1.0,0.0,0.5}) ; // left side
    HalfPlanes_.push_back({0.0,1.0,0.5}) ; // top side
    HalfPlanes_.push_back({0.0,-1.0,0.5}) ; // bottom side
}

bool BeliefCVC::satisfiesConstraints(oc::PathControl p, std::vector<ConstraintPtr> constraints)
{
    std::vector<ob::State*> states = p.getStates();
    const double step_size = p.getControlDuration(0);
    double current_time = 0.0;
    for (auto itr = states.begin(); itr != states.end(); itr++) {
    	// for every state along path, check if the time coincides with constraints
        for (auto c_itr = constraints.begin(); c_itr != constraints.end(); c_itr++) {
        	const std::vector<double> times = (*c_itr)->as<BeliefConstraint>()->getTimes();
        	// check if current_time coincides with constraint times (avoid weird rounding error)
        	auto it = std::find_if(times.begin(), times.end(), 
        		[&current_time](const double& t) { return abs(t - current_time) < 1E-9;});
        	if (it == times.end()) {
        		break;
        	}
        	// must check this constraint at this time
        	int idx = it - times.begin();
        	// get the constraint belief
        	ob::State* c_st = (*c_itr)->as<BeliefConstraint>()->getStates()[idx];
        	std::pair<Eigen::Vector2d, Eigen::Matrix2d> c_belief = getDistFromState_(c_st);
        	// get the robot belief
        	std::pair<Eigen::Vector2d, Eigen::Matrix2d> r_belief = getDistFromState_(*itr);
        	// check chance constraint
            const Eigen::Vector2d mu_ab = r_belief.first - c_belief.first;
            const Eigen::Matrix2d Sigma_ab = r_belief.second + c_belief.second;
            if (! isSafe_(mu_ab, Sigma_ab))
            	return false;
        }
        current_time += step_size;
    }
	return true;
}

std::pair<Eigen::Vector2d, Eigen::Matrix2d> BeliefCVC::getDistFromState_(ob::State* st)
{
    Eigen::Vector2d mu = st->as<R2BeliefSpace::StateType>()->getXY();
    Eigen::Matrix2d Sigma = st->as<R2BeliefSpace::StateType>()->getCovariance();
    std::pair<Eigen::Vector2d, Eigen::Matrix2d> p(mu, Sigma);
    return p;
}

bool BeliefCVC::isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab)
{
    Eigen::Vector2d a;
    for (std::vector<double> hp: HalfPlanes_){
        a << hp[0],
             hp[1];
        double b = hp[0];
        double Pv = sqrt(double(a.transpose() * Sigma_ab * a));
        double vbar = sqrt(2) * Pv * boost::math::erf_inv(1 - 2*p_coll_dist_);
        bool sat = (hp[0] * mu_ab[0] + hp[1] * mu_ab[1] - hp[2] >= vbar);  
        if(sat) {
            return true;
        };      
    }
    return false;
}
