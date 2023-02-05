#include "ConstraintValidityCheckers/BeliefCVC.h"


BeliefCVC::BeliefCVC(Robot *robot, const int num_obs): ConstraintValidityChecker(robot, "BeliefCVC"),
    p_safe_(0.7), 
    p_coll_dist_( (1-p_safe_) / num_obs)
{
    // this can be generalized in the future based on robot shapes
    HalfPlanes_.push_back({1.0,0.0,1.0}) ; // right side
    HalfPlanes_.push_back({-1.0,0.0,1.0}) ; // left side
    HalfPlanes_.push_back({0.0,1.0,1.0}) ; // top side
    HalfPlanes_.push_back({0.0,-1.0,1.0}) ; // bottom side
}

bool BeliefCVC::satisfiesConstraints(oc::PathControl p, std::vector<ConstraintPtr> constraints)
{
    // std::cout << "entering satisfiesConstraints" << std::endl;
    std::vector<ob::State*> states = p.getStates();
    for (auto itr = states.begin(); itr != states.end(); itr++) {
        std::pair<Eigen::Vector2d, Eigen::Matrix2d> r_st = getDistFromState_(*itr);
        for (auto c_itr = constraints.begin(); c_itr != constraints.end(); c_itr++) {
            const std::vector<ob::State*> c_itr_states = (*c_itr)->as<BeliefConstraint>()->getStates();
            for (auto c_states_itr = c_itr_states.begin(); c_states_itr != c_itr_states.end(); c_states_itr++) {
                // std::cout << *c_states_itr << std::endl;
                // std::cout << *c_states_itr << std::endl;
                std::pair<Eigen::Vector2d, Eigen::Matrix2d> c_st = getDistFromState_((*c_states_itr));
                // Eigen::Vector2d mu = (*c_states_itr)->as<R2BeliefSpace::StateType>()->getXY();
                // Eigen::Matrix2d Sigma = (*c_states_itr)->as<R2BeliefSpace::StateType>()->getCovariance();
                // transform dists into one
                const Eigen::Vector2d mu_ab = r_st.first - c_st.first;
                const Eigen::Matrix2d Sigma_ab = r_st.second - c_st.second;
                if (! isSafe_(mu_ab, Sigma_ab))
                    return false;
            }
        }
    }
    // std::cout << "exiting satisfiesConstraints" << std::endl;
    // exit(1);
	return true;
}

std::pair<Eigen::Vector2d, Eigen::Matrix2d> BeliefCVC::getDistFromState_(ob::State* st)
{
    // std::cout << "in getDistFromState_" << std::endl;
    Eigen::Vector2d mu = st->as<R2BeliefSpace::StateType>()->getXY();
    Eigen::Matrix2d Sigma = st->as<R2BeliefSpace::StateType>()->getCovariance();
    std::pair<Eigen::Vector2d, Eigen::Matrix2d> p(mu, Sigma);
    // std::cout << "out getDistFromState_" << std::endl;
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
