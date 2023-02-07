#include "PlanValidityCheckers/BeliefPlanValidityChecker.h"

BeliefPlanValidityChecker::BeliefPlanValidityChecker(MultiRobotProblemDefinitionPtr pdef):
	PlanValidityChecker(pdef, "BeliefPlanValidityChecker"), 
	p_safe_(0.7), 
	p_coll_dist_( (1-p_safe_) / pdef->getInstance()->getObstacles().size())
{
	// this can be generalized in the future based on robot shapes
	HalfPlanes_.push_back({1.0,0.0,0.5}) ; // right side
    HalfPlanes_.push_back({-1.0,0.0,0.5}) ; // left side
    HalfPlanes_.push_back({0.0,1.0,0.5}) ; // top side
    HalfPlanes_.push_back({0.0,-1.0,0.5}) ; // bottom side

    
};

BeliefPlanValidityChecker::~BeliefPlanValidityChecker(){};

std::vector<ConflictPtr> BeliefPlanValidityChecker::validatePlan(Plan p)
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
		std::vector<std::pair<int, std::pair<Eigen::Vector2d, Eigen::Matrix2d>>> activeRobots = getActiveRobots_(p, k);
		ConflictPtr c = checkForConflicts_(activeRobots, k);
		if (c) {
			// found initial conflict at step k
			// must continue to propogate forward until conflict is finished
			int step = k;
			while (c != nullptr && step < maxStates) {
				// std::cout << "found: " << c << std::endl;
				confs.push_back(c);
				step++;
				activeRobots = getActiveRobots_(p, step, c->agent1Idx_, c->agent2Idx_);
				c = checkForConflicts_(activeRobots, step);
			}
			return confs;
		}
	}
	return {};
}

ConstraintPtr BeliefPlanValidityChecker::createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx)
{
	const double step_duration = mrmp_pdef_->getSystemStepSize();
	std::vector<double> times;
	std::vector<ob::State*> states;
	for (auto itr = conflicts.begin(); itr != conflicts.end(); itr++) {
		times.push_back(((*itr)->timeStep_ * step_duration));
		if ( (*itr)->timeStep_ <  p[robotIdx].getStates().size()) {
            ob::State* st = mrmp_pdef_->getRobotSpaceInformationPtr(robotIdx)->allocState();
            mrmp_pdef_->getRobotSpaceInformationPtr(robotIdx)->copyState(st, p[robotIdx].getState((*itr)->timeStep_));
			states.push_back(st);
		}
		else {
            ob::State* st = mrmp_pdef_->getRobotSpaceInformationPtr(robotIdx)->allocState();
            mrmp_pdef_->getRobotSpaceInformationPtr(robotIdx)->copyState(st, p[robotIdx].getStates().back());
			states.push_back(st);
		}
	}
	ConstraintPtr c = std::make_shared<BeliefConstraint>(robotIdx, times, states);
	return c;
}

std::vector<std::pair<int, std::pair<Eigen::Vector2d, Eigen::Matrix2d>>> BeliefPlanValidityChecker::getActiveRobots_(Plan p, const int step, const int a1, const int a2)
{
	std::vector<std::pair<int, std::pair<Eigen::Vector2d, Eigen::Matrix2d>>> activeRobots{};

	// using no pre-defined agent pair, return vector of all robots
	if (a1 == -1 || a2 == -1) {
		int idx = 0;
		auto itr = p.begin();
		for(; itr < p.end(); itr++, idx++ )
		{
			if (step < itr->getStateCount()){
				std::pair<Eigen::Vector2d, Eigen::Matrix2d> st = getDistFromState_(itr->getState(step));
				activeRobots.emplace_back(idx, st);
			}
			else{
				std::pair<Eigen::Vector2d, Eigen::Matrix2d> st = getDistFromState_(itr->getStates().back());
				activeRobots.emplace_back(idx, st);
			}
		}
	}
	else {
		// provided agent pair, return vector with only those two robots
		if (step < p[a1].getStateCount()) {
			std::pair<Eigen::Vector2d, Eigen::Matrix2d> st = getDistFromState_(p[a1].getState(step));
			activeRobots.emplace_back(a1, st);
		}
		else {
			std::pair<Eigen::Vector2d, Eigen::Matrix2d> st = getDistFromState_(p[a1].getStates().back());
			activeRobots.emplace_back(a1, st);
		}
		if (step < p[a2].getStateCount()) {
			std::pair<Eigen::Vector2d, Eigen::Matrix2d> st = getDistFromState_(p[a2].getState(step));
			activeRobots.emplace_back(a2, st);
		}
		else {
			std::pair<Eigen::Vector2d, Eigen::Matrix2d> st = getDistFromState_(p[a2].getStates().back());
			activeRobots.emplace_back(a2, st);
		}
	}
	return activeRobots;
}

std::pair<Eigen::Vector2d, Eigen::Matrix2d> BeliefPlanValidityChecker::getDistFromState_(ob::State* st)
{
	Eigen::Vector2d mu = st->as<R2BeliefSpace::StateType>()->getXY();
	Eigen::Matrix2d Sigma = st->as<R2BeliefSpace::StateType>()->getCovariance();
	std::pair<Eigen::Vector2d, Eigen::Matrix2d> p(mu, Sigma);
	return p;
}

ConflictPtr BeliefPlanValidityChecker::checkForConflicts_(std::vector<std::pair<int, std::pair<Eigen::Vector2d, Eigen::Matrix2d>>> states, const int step)
{
	ConflictPtr c = nullptr;
	for (int ai = 0; ai < states.size(); ai++) {
		// Belief for ai
		const Eigen::Vector2d mu_ai = states[ai].second.first;
		const Eigen::Matrix2d Sigma_ai = states[ai].second.second;
		for (int aj = ai + 1; aj < states.size(); aj++) {
			// Belief for aj
			const Eigen::Vector2d mu_aj = states[aj].second.first;
			const Eigen::Matrix2d Sigma_aj = states[aj].second.second;
			const Eigen::Vector2d mu_ab = mu_ai - mu_aj;
			const Eigen::Matrix2d Sigma_ab = Sigma_ai + Sigma_aj;
			if (!isSafe_(mu_ab, Sigma_ab)) {
				c = std::make_shared<Conflict>(ai, aj, step);
				return c;
			}
		}
	}
	return c;
}

bool BeliefPlanValidityChecker::isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab)
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



