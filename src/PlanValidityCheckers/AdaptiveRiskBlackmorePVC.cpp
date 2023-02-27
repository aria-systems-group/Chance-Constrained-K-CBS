#include "PlanValidityCheckers/AdaptiveRiskBlackmorePVC.h"


AdaptiveRiskBlackmorePVC::AdaptiveRiskBlackmorePVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe):
    BeliefPVC(pdef, "AdaptiveRiskBlackmorePVC", p_safe)
{
    std::vector<Robot*> robots = mrmp_pdef_->getInstance()->getRobots();
    boost::tokenizer< boost::char_separator<char> >::iterator beg1;
    boost::tokenizer< boost::char_separator<char> >::iterator beg2;
    boost::char_separator<char> sep(" ");
    for (auto itr1 = robots.begin(); itr1 != robots.end(); itr1++) {
        std::string r1_name = (*itr1)->getName();
        boost::tokenizer< boost::char_separator<char> > tok1(r1_name, sep);
        beg1 = tok1.begin();
        beg1++;
        for (auto itr2 = itr1 + 1; itr2 != robots.end(); itr2++) {
            std::string r2_name = (*itr2)->getName();
            boost::tokenizer< boost::char_separator<char> > tok2(r2_name, sep);
            beg2 = tok2.begin();
            beg2++;
            std::string key_name = (*beg1) + "," + (*beg2);
            Polygon combined_poly = getMinkowskiSumOfRobots_(*itr1, *itr2);
            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> robot_pair_halfplanes = getHalfPlanes_(combined_poly);
            halfPlane_map_.insert({key_name, robot_pair_halfplanes});
        }
    }
}

std::vector<ConflictPtr> AdaptiveRiskBlackmorePVC::validatePlan(Plan p)
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

bool AdaptiveRiskBlackmorePVC::satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints)
{
    /* Assume that the constrained robot is the "planning" robot. Check for satisfaction of all constraints */
    const int constrained_robot = constraints.front()->getConstrainedAgent();
    const std::string r_a_name = mrmp_pdef_->getInstance()->getRobots()[constrained_robot]->getName();
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
                continue;
            }
            // must check this constraint at this time
            const int constraining_robot = (*c_itr)->getConstrainingAgent();
            assert(constrained_robot != constraining_robot);
            
            // get the correct half-plane for comparing constrained_robot with constraining_robot
            std::string key_name = std::to_string(constrained_robot) + "," + std::to_string(constraining_robot);
            if (halfPlane_map_.find(key_name) == halfPlane_map_.end())
                key_name = std::to_string(constraining_robot) + "," + std::to_string(constrained_robot);

            // get both beliefs
            const std::string r_b_name = mrmp_pdef_->getInstance()->getRobots()[constraining_robot]->getName();
            int idx = it - times.begin();
            Belief constrained_robot_belief = getDistribution_(*itr);
            Belief constraining_robot_belief = getDistribution_((*c_itr)->as<BeliefConstraint>()->getStates()[idx]);
            std::map<std::string, Belief> states_map{{r_a_name, constrained_robot_belief}, {r_b_name, constraining_robot_belief}};

            // calculate the the eta_i's for agent *itr_a
            std::unordered_map<std::string, double> eta_map = createEtaMap_(r_a_name, states_map);

            // check if constraint is violated
            Eigen::Vector2d mu_ab = constrained_robot_belief.first - constraining_robot_belief.first;
            Eigen::Matrix2d Sigma_ab = constrained_robot_belief.second + constraining_robot_belief.second;

            if (!isSafe_(mu_ab, Sigma_ab, halfPlane_map_[key_name], eta_map[r_b_name]))
                return false;
        }
        current_time += step_size;
    }
    return true;
}

bool AdaptiveRiskBlackmorePVC::independentCheck(ob::State* state1, ob::State* state2)
{
    /* Only to be used for independent collision testing. Not called during MRMP planning (yet?) */

    std::string key_name = "0,1";
    if (halfPlane_map_.find(key_name) == halfPlane_map_.end())
        key_name = "1,0";

    Belief r0_belief = getDistribution_(state1);
    Belief r1_belief = getDistribution_(state2);

    // calculate the the eta_i's for agent *itr_a
    std::map<std::string, Belief> states_map{{"Robot 0", r0_belief}, {"Robot 1", r1_belief}};
    std::unordered_map<std::string, double> eta_map = createEtaMap_("Robot 0", states_map);

    Eigen::Vector2d mu_ab = r0_belief.first - r1_belief.first;
    Eigen::Matrix2d Sigma_ab = r0_belief.second + r1_belief.second;

    if (isSafe_(mu_ab, Sigma_ab, halfPlane_map_[key_name], eta_map["Robot 1"]))
        return true;
    return false;
}

ConflictPtr AdaptiveRiskBlackmorePVC::checkForConflicts_(std::map<std::string, Belief> states_map, const int step)
{
    ConflictPtr c = nullptr;
    
    boost::tokenizer< boost::char_separator<char> >::iterator beg_a;
    boost::tokenizer< boost::char_separator<char> >::iterator beg_b;
    boost::char_separator<char> sep(" ");

    for (auto itr_a = states_map.begin(); itr_a != states_map.end(); itr_a++) {
        // Belief for ai
        const std::string r_a_name = itr_a->first;
        boost::tokenizer< boost::char_separator<char> > tok_a(r_a_name, sep);
        beg_a = tok_a.begin();
        beg_a++;
        const Eigen::Vector2d mu_a = (itr_a->second).first;
        const Eigen::Matrix2d Sigma_a = (itr_a->second).second;
        // calculate the the eta_i's for agent *itr_a
        std::unordered_map<std::string, double> eta_map = createEtaMap_(r_a_name, states_map);
        // set for itr_b
        auto itr_b = itr_a;
        std::advance(itr_b, 1);
        for (; itr_b != states_map.end(); itr_b++) {
            // Belief for aj
            const std::string r_b_name = itr_b->first;
            boost::tokenizer< boost::char_separator<char> > tok_b(r_b_name, sep);
            beg_b = tok_b.begin();
            beg_b++;
            std::string key_name = (*beg_a) + "," + (*beg_b);
            const Eigen::Vector2d mu_b = (itr_b->second).first;
            const Eigen::Matrix2d Sigma_b = (itr_b->second).second;
            const Eigen::Vector2d mu_ab = mu_a - mu_b;
            const Eigen::Matrix2d Sigma_ab = Sigma_a + Sigma_b;

            if (halfPlane_map_.find(key_name) == halfPlane_map_.end())
                key_name = (*beg_b) + "," + (*beg_a);

            if (!isSafe_(mu_ab, Sigma_ab, halfPlane_map_[key_name], eta_map[r_b_name])) {
                int a_idx = std::atoi((*beg_a).c_str());
                int b_idx = std::atoi((*beg_b).c_str());
                c = std::make_shared<Conflict>(a_idx, b_idx, step);
                return c;
            }
        }
    }
    return c;
}

std::unordered_map<std::string, double> AdaptiveRiskBlackmorePVC::createEtaMap_(const std::string r_a_name, std::map<std::string, Belief> states_map)
{
    /* Find d_i's from r_a_name mean to all other robot means and sum the d_i's */
    const Belief r_a_belief = states_map[r_a_name];
    std::unordered_map<std::string, double> di_map;
    double sum = 0;
    for (auto itr_b = states_map.begin(); itr_b != states_map.end(); itr_b++) {
        if (itr_b->first == r_a_name)
            continue;
        
        const Belief r_b_belief = itr_b->second;
        const double mu_diff_x = r_a_belief.first(0, 0) - r_b_belief.first(0, 0);
        const double mu_diff_y = r_a_belief.first(1, 0) - r_b_belief.first(1, 0);
        const double di =  sqrt( mu_diff_x*mu_diff_x + mu_diff_y*mu_diff_y );
        di_map[itr_b->first] = di;
        sum += (1 / di);
    }

    const double alpha = 1 / sum;

    /* Calculate all the eta_i and return */
    std::unordered_map<std::string, double> eta_map;
    for (auto di_itr = di_map.begin(); di_itr != di_map.end(); di_itr++) {
        const double eta_i = (alpha / di_itr->second) * p_coll_agnts_;
        eta_map[di_itr->first] = eta_i; 
    }
    return eta_map;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> AdaptiveRiskBlackmorePVC::getHalfPlanes_(Polygon combined_poly)
{
    /* Converts a Polygon to a set of halfplanes */
    auto exterior_points = bg::exterior_ring(combined_poly);
    if (exterior_points.size() != 5) {
        OMPL_WARN("%s: Generating Halfplanes currently supports closed rectangles. Please check the implementation.", name_.c_str());
    }
    Eigen::MatrixXd A(4, 2);
    Eigen::MatrixXd B(4, 1);
    for (int i=0; i<exterior_points.size()-1; i++)
    {
        double x1 = bg::get<0>(exterior_points[i]);
        double y1 = bg::get<1>(exterior_points[i]);
        double x2 = bg::get<0>(exterior_points[i+1]);
        double y2 = bg::get<1>(exterior_points[i+1]);
        // std::cout << x1 << "," << y1 << std::endl;

        double a = y2 - y1;
        double b = x1 - x2;
        double c = a * x1 + b * y1;

        A(i, 0) = a;
        A(i, 1) = b;
        B(i,0) = c;
    }
    return std::make_pair(A,B);
}

bool AdaptiveRiskBlackmorePVC::isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab, const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> halfPlanes, const double eta_i)
{
    Eigen::MatrixXd A = halfPlanes.first;
    Eigen::MatrixXd B = halfPlanes.second;

    auto const n_rows = A.rows();
    for (int i = 0; i < n_rows; i++) {
        const double tmp = (A.row(i) * Sigma_ab * A.row(i).transpose()).value();
        const double Pv = sqrt(tmp);
        const double vbar = sqrt(2) * Pv * bm::erf_inv(1 - (2 * eta_i));
        if( (A(i, 0) * mu_ab[0] + A(i, 1) * mu_ab[1] - B(i, 0) >= vbar) ) {
            return true;
        };
    }
    return false;
}

Polygon AdaptiveRiskBlackmorePVC::getMinkowskiSumOfRobots_(Robot* r1, Robot* r2)
{
    /* Perform the Minkowski Sum */
    std::vector<Point> shape1_points = boost::geometry::exterior_ring(r1->getBoundingShape());
    std::vector<Point> shape2_points = boost::geometry::exterior_ring(r2->getBoundingShape());

    shape1_points.push_back(shape1_points[1]);
    shape2_points.push_back(shape2_points[1]);

    std::vector<Point> result;
    int i = 0, j = 0;
    while(i < shape1_points.size() - 2 || j < shape2_points.size() - 2){
        result.push_back(addPoints_(shape1_points[i], shape2_points[j]));
        Point s1_diff = subtractPoints_(shape1_points[i + 1], shape1_points[i]);
        Point s2_diff = subtractPoints_(shape2_points[j + 1], shape2_points[j]);
        double cross = crossProduct_(s1_diff, s2_diff);
        if(cross >= 0)
            ++i;
        if(cross <= 0)
            ++j;
    }

    result.push_back(result.front()); // make polygon closed

    Polygon result_poly;

    for (auto itr = result.begin(); itr != result.end(); itr++) {
        // double x = bg::get<0>(*itr);
        // double y = bg::get<1>(*itr);
        // std::cout << x << "," << y << std::endl;
        bg::append(result_poly.outer(), *itr);
    }
    bg::correct(result_poly);
    return result_poly;
}

double AdaptiveRiskBlackmorePVC::crossProduct_(const Point &a, const Point &b)
{
    /* Performs A \times B */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    return (ax * by) - (ay * bx);
}

Point AdaptiveRiskBlackmorePVC::subtractPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax-bx, ay-by);
    return difference;
}

Point AdaptiveRiskBlackmorePVC::addPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax+bx, ay+by);
    return difference;
}
