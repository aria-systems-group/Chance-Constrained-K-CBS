#include "PlanValidityCheckers/ChiSquaredBoundaryPVC.h"


ChiSquaredBoundaryPVC::ChiSquaredBoundaryPVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe):
    BeliefPVC(pdef, "ChiSquaredBoundaryPVC", p_safe), sc_(-1)
{
    std::vector<Robot*> robots = mrmp_pdef_->getInstance()->getRobots();
    boost::tokenizer< boost::char_separator<char> >::iterator beg1;
    boost::tokenizer< boost::char_separator<char> >::iterator beg2;
    boost::char_separator<char> sep(" ");
    for (auto itr1 = robots.begin(); itr1 != robots.end(); itr1++) {
        std::string r1_name = (*itr1)->getName();
        boost::tokenizer< boost::char_separator<char> > tok1(r1_name, sep);
        double max_rad = (*itr1)->getBoundingRadius();
        boundingRadii_map.insert({r1_name, max_rad});
    }

    // set sc_ value
    bm::chi_squared mydist(2);
    sc_ = bm::cdf(mydist, p_safe_agnts_);
}

std::vector<ConflictPtr> ChiSquaredBoundaryPVC::validatePlan(Plan p)
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
        std::map<std::string, Belief> activeRobots = getActiveRobots_(p, k);
        ConflictPtr c = checkForConflicts_(activeRobots, k);
        if (c) {
            // found initial conflict at step k
            // must continue to propogate forward until conflict is finished
            int step = k;
            while (c != nullptr && step < maxStates) {
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

bool ChiSquaredBoundaryPVC::satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints)
{
    /* Assume that the constrained robot is the "planning" robot. Check for satisfaction of all constraints */
    const int constrained_robot = constraints.front()->getConstrainedAgent();
    const std::string constrained_robot_name = mrmp_pdef_->getInstance()->getRobots()[constrained_robot]->getName();
    std::vector<ob::State*> states = path.getStates();
    const double step_size = path.getControlDuration(0);
    double current_time = 0.0;
    for (auto itr = states.begin(); itr != states.end(); itr++) {
        // for every state along path, check if the time coincides with any constraints
        for (auto c_itr = constraints.begin(); c_itr != constraints.end(); c_itr++) {
            const std::vector<double> times = (*c_itr)->getTimes();
            auto it = std::find_if(times.begin(), times.end(), 
                [&current_time](const double& t) { return abs(t - current_time) < 1E-9;});
            if (it == times.end()) {
                break;
            }
            // must check this constraint at this time
            const int constraining_robot = (*c_itr)->getConstrainingAgent();
            const std::string constraining_robot_name = mrmp_pdef_->getInstance()->getRobots()[constraining_robot]->getName();
            assert(constrained_robot != constraining_robot);

            // get both beliefs
            int idx = it - times.begin();
            Belief constrained_robot_belief = getDistribution_(*itr);
            Belief constraining_robot_belief = getDistribution_((*c_itr)->as<BeliefConstraint>()->getStates()[idx]);

            if (!isSafe_(constrained_robot_belief, boundingRadii_map[constrained_robot_name], constraining_robot_belief, boundingRadii_map[constraining_robot_name]))
                return false;
        }
        current_time += step_size;
    }
    return true;
}

ConflictPtr ChiSquaredBoundaryPVC::checkForConflicts_(std::map<std::string, Belief> states_map, const int step)
{
    ConflictPtr c = nullptr;
    
    boost::tokenizer< boost::char_separator<char> >::iterator beg_a;
    boost::tokenizer< boost::char_separator<char> >::iterator beg_b;
    boost::char_separator<char> sep(" ");

    for (auto itr_a = states_map.begin(); itr_a != states_map.end(); itr_a++) {
        // Belief for ai
        const std::string r_a_name = itr_a->first;
        const Belief r_a_belief = itr_a->second;
        boost::tokenizer< boost::char_separator<char> > tok_a(r_a_name, sep);
        beg_a = tok_a.begin();
        beg_a++;
        auto itr_b = itr_a;
        std::advance(itr_b, 1);
        for (; itr_b != states_map.end(); itr_b++) {
            // Belief for aj
            const std::string r_b_name = itr_b->first;
            const Belief r_b_belief = itr_b->second;
            boost::tokenizer< boost::char_separator<char> > tok_b(r_b_name, sep);
            beg_b = tok_b.begin();
            beg_b++;

            if (!isSafe_(r_a_belief, boundingRadii_map[r_a_name], r_b_belief, boundingRadii_map[r_b_name])) {
                int a_idx = std::atoi((*beg_a).c_str());
                int b_idx = std::atoi((*beg_b).c_str());
                c = std::make_shared<Conflict>(a_idx, b_idx, step);
                return c;
            }
        }
    }
    return c;
}

bool ChiSquaredBoundaryPVC::isSafe_(const Belief belief_a, const double rad_a, const Belief belief_b, const double rad_b)
{
    /* Find maximum eigenvalues of the covariances */
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver_a(belief_a.second.rows());
    solver_a.compute(belief_a.second);

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver_b(belief_b.second.rows());
    solver_b.compute(belief_b.second);

    const double max_lambda_a = solver_a.eigenvalues().maxCoeff();
    const double max_lambda_b = solver_b.eigenvalues().maxCoeff();

    /* Calculate the "upper-bounded" distance between the distributions and the safety boundary */
    auto mu_a = belief_a.first;
    auto mu_b = belief_b.first;

    const double mu_diff_x = mu_a(0, 0) - mu_b(0, 0);
    const double mu_diff_y = mu_a(1, 0) - mu_b(1, 0);

    const double dist =  sqrt( mu_diff_x*mu_diff_x + mu_diff_y*mu_diff_y );
    const double boundary = (max_lambda_a * sc_ + rad_a) + (max_lambda_b * sc_ + rad_b);
    
    /* Check if safe */
    if (dist > boundary)
        return true;
    else
        return false;
}
