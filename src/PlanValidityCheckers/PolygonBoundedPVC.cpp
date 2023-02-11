#include "PlanValidityCheckers/PolygonBoundedPVC.h"


PolygonBoundedPVC::PolygonBoundedPVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe):
    BeliefPVC(pdef, "PolygonBoundedPVC", p_safe)
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

std::vector<ConflictPtr> PolygonBoundedPVC::validatePlan(Plan p)
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
        std::unordered_map<std::string, Belief> activeRobots = getActiveRobots_(p, k);
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

bool PolygonBoundedPVC::satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints)
{
    /* Assume that the constrained robot is the "planning" robot. Check for satisfaction of all constraints */
    const int constrained_robot = constraints.front()->getConstrainedAgent();
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
            assert(constrained_robot != constraining_robot);
            
            // get the correct half-plane for comparing constrained_robot with constraining_robot
            std::string key_name = std::to_string(constrained_robot) + "," + std::to_string(constraining_robot);
            if (halfPlane_map_.find(key_name) == halfPlane_map_.end())
                key_name = std::to_string(constraining_robot) + "," + std::to_string(constrained_robot);

            // get both beliefs
            int idx = it - times.begin();
            Belief constrained_robot_belief = getDistribution_(*itr);
            Belief constraining_robot_belief = getDistribution_((*c_itr)->as<BeliefConstraint>()->getStates()[idx]);

            // check if constraint is violated
            Eigen::Vector2d mu_ab = constrained_robot_belief.first - constraining_robot_belief.first;
            Eigen::Matrix2d Sigma_ab = constrained_robot_belief.second + constraining_robot_belief.second;

            if (!isSafe_(mu_ab, Sigma_ab, halfPlane_map_[key_name]))
                return false;
        }
        current_time += step_size;
    }
    return true;
}

ConflictPtr PolygonBoundedPVC::checkForConflicts_(std::unordered_map<std::string, Belief> states_map, const int step)
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

            if (!isSafe_(mu_ab, Sigma_ab, halfPlane_map_[key_name])) {
                int a_idx = std::atoi((*beg_a).c_str());
                int b_idx = std::atoi((*beg_b).c_str());
                c = std::make_shared<Conflict>(a_idx, b_idx, step);
                return c;
            }
        }
    }
    return c;
}

bool PolygonBoundedPVC::isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab, const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> halfPlanes)
{
    Eigen::MatrixXd A = halfPlanes.first;
    Eigen::MatrixXd B = halfPlanes.second;

    auto const n_rows = A.rows();
    for (int i = 0; i < n_rows; i++) {
        const double tmp = (A.row(i) * Sigma_ab * A.row(i).transpose()).value();
        const double Pv = sqrt(tmp);
        const double vbar = sqrt(2) * Pv * bm::erf_inv(1 - (2 * p_safe_));
        if( (A(i, 0) * mu_ab[0] + A(i, 1) * mu_ab[1] - B(i, 0) >= vbar) ) {
            return true;
        };
    }
    return false;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> PolygonBoundedPVC::getHalfPlanes_(Polygon combined_poly)
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

Polygon PolygonBoundedPVC::getMinkowskiSumOfRobots_(Robot* r1, Robot* r2)
{
    // // test: triangle and rectangle from web (centered at origin!)
    // // https://cp-algorithms.com/geometry/minkowski.html#algorithm
    // Polygon shape1{{{0, -0.5}, {0.5, 0.5}, {-0.5, 0.5}}}; 
    // Polygon shape2{{{-1, -1}, {1, -1}, {1, 1}, {-1,1}}};

    /* First, center robot 1 at origin */
    Polygon r1_initial_poly;
    Polygon r1_origin_centered;

    const double r1_ix = r1->getStartLocation().x_;
    const double r1_iy = r1->getStartLocation().y_;
    bg::correct(r1_initial_poly);
    bg::assign(r1_initial_poly, r1->getShape());
    bg::strategy::transform::matrix_transformer<double, 2, 2> r1_xfrm(
             cos(0), sin(0), (0 - r1_ix),
            -sin(0), cos(0), (0 - r1_iy),
                      0,          0,  1);
    bg::transform(r1_initial_poly, r1_origin_centered, r1_xfrm);
    bg::correct(r1_origin_centered);

    /* Next, center robot 2 at origin */
    Polygon r2_initial_poly;
    Polygon r2_origin_centered;

    const double r2_ix = r2->getStartLocation().x_;
    const double r2_iy = r2->getStartLocation().y_;
    bg::correct(r2_initial_poly);
    bg::assign(r2_initial_poly, r2->getShape());
    bg::strategy::transform::matrix_transformer<double, 2, 2> r2_xfrm(
             cos(0), sin(0), (0 - r2_ix),
            -sin(0), cos(0), (0 - r2_iy),
                      0,          0,  1);
    bg::transform(r2_initial_poly, r2_origin_centered, r2_xfrm);
    bg::correct(r2_origin_centered);

    // for (const auto &a : boost::geometry::exterior_ring(r2_initial_poly)) {
    //     const double x = bg::get<0>(a);
    //     const double y = bg::get<1>(a);
    //     std::cout << x << "," << y << std::endl;
    // }
    // std::cout << "moved to" << std::endl;
    // for (const auto &a : boost::geometry::exterior_ring(r2_origin_centered)) {
    //     const double x = bg::get<0>(a);
    //     const double y = bg::get<1>(a);
    //     std::cout << x << "," << y << std::endl;
    // }

    /* Perform the Minkowski Sum */
    std::vector<Point> shape1_points = boost::geometry::exterior_ring(r1_origin_centered);
    std::vector<Point> shape2_points = boost::geometry::exterior_ring(r2_origin_centered);

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

double PolygonBoundedPVC::crossProduct_(const Point &a, const Point &b)
{
    /* Performs A \times B */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    return (ax * by) - (ay * bx);
}

Point PolygonBoundedPVC::subtractPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax-bx, ay-by);
    return difference;
}

Point PolygonBoundedPVC::addPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax+bx, ay+by);
    return difference;
}
