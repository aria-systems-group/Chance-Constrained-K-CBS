#include "PlanValidityCheckers/CDFGridPVC.h"


CDFGridPVC::CDFGridPVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe, int nSteps):
    sc_(chi_squared_quantile_(2, p_safe_agnts_)), disk_steps_(nSteps), BeliefPVC(pdef, "CDFGridPVC", p_safe)
{
    std::string name = "CDFGridPVC(" + std::to_string(nSteps) + ")";
    this->name_ = name;

    std::vector<Robot*> robots = mrmp_pdef_->getInstance()->getRobots();
    boost::tokenizer< boost::char_separator<char> >::iterator beg1;
    boost::tokenizer< boost::char_separator<char> >::iterator beg2;
    boost::char_separator<char> sep(" ");
    for (auto itr1 = robots.begin(); itr1 != robots.end(); itr1++) {
        std::string r1_name = (*itr1)->getName();
        double max_rad_1 = (*itr1)->getBoundingRadius();
        boost::tokenizer< boost::char_separator<char> > tok1(r1_name, sep);
        beg1 = tok1.begin();
        beg1++;
        for (auto itr2 = itr1 + 1; itr2 != robots.end(); itr2++) {
            std::string r2_name = (*itr2)->getName();
            double max_rad_2 = (*itr2)->getBoundingRadius();
            boost::tokenizer< boost::char_separator<char> > tok2(r2_name, sep);
            beg2 = tok2.begin();
            beg2++;
            std::string key_name = (*beg1) + "," + (*beg2);
            Polygon combined_poly = getBoundingBox_(max_rad_1, max_rad_2);
            polygon_map_.insert({key_name, combined_poly});
            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> robot_pair_halfplanes = getHalfPlanes_(combined_poly);
            halfPlane_map_.insert({key_name, robot_pair_halfplanes});
        }
    }
}

 std::vector<ConflictPtr> CDFGridPVC::validatePlan(Plan p)
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

bool CDFGridPVC::satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints)
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
                continue;
            }
            // std::cout << "checking constraint at time: " << (*it) << std::endl;
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

            if (!isSafe_(constrained_robot_belief, constraining_robot_belief, halfPlane_map_[key_name], polygon_map_[key_name]))
                return false;
        }
        current_time += step_size;
    }
    return true;
}

ConflictPtr CDFGridPVC::checkForConflicts_(std::map<std::string, Belief> states_map, const int step)
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

        auto itr_b = itr_a;
        std::advance(itr_b, 1);
        for (; itr_b != states_map.end(); itr_b++) {
            // Belief for aj
            const std::string r_b_name = itr_b->first;
            boost::tokenizer< boost::char_separator<char> > tok_b(r_b_name, sep);
            beg_b = tok_b.begin();
            beg_b++;
            std::string key_name = (*beg_a) + "," + (*beg_b);

            if (halfPlane_map_.find(key_name) == halfPlane_map_.end())
                key_name = (*beg_b) + "," + (*beg_a);

            if (!isSafe_(itr_a->second, itr_b->second, halfPlane_map_[key_name], polygon_map_[key_name])) {
                int a_idx = std::atoi((*beg_a).c_str());
                int b_idx = std::atoi((*beg_b).c_str());
                c = std::make_shared<Conflict>(a_idx, b_idx, step);
                // std::cout << "found conflict between " << a_idx << "," << b_idx << std::endl;
                // std::cout << (*itr_a).second.first << std::endl;
                // std::cout << (*itr_a).second.second << std::endl;
                // std::cout << (*itr_b).second.first << std::endl;
                // std::cout << (*itr_b).second.second << std::endl;
                return c;
            }
        }
    }
    return c;
}

bool CDFGridPVC::independentCheck(ob::State* state1, ob::State* state2)
{
    /* Only to be used for independent collision testing. Not called during MRMP planning (yet?) */
    // const double max_rad0 = boundingRadii_map_["Robot 0"];
    // const double max_rad1 = boundingRadii_map_["Robot 1"];

    Belief r0_belief = getDistribution_(state1);
    Belief r1_belief = getDistribution_(state2);
    
    // the test case
    // Eigen::Vector2d mu_a{0,0};
    // Eigen::Matrix2d sigma_a;
    // sigma_a << 0.11, 0, 0, 0.12;
    // Belief r0_belief(mu_a, sigma_a);

    // Eigen::Vector2d mu_b{1, 2};
    // Eigen::Matrix2d sigma_b;
    // sigma_b << 0.25, 0.1, 0.1, 0.5;
    // Belief r1_belief(mu_b, sigma_b);

    std::string key_name = "0,1";
    if (halfPlane_map_.find(key_name) == halfPlane_map_.end())
        key_name = "1,0";

    if (isSafe_(r0_belief, r1_belief, halfPlane_map_[key_name], polygon_map_[key_name]))
        return true;
    return false;
}

bool CDFGridPVC::isSafe_(const Belief belief_a, const Belief belief_b, std::pair<Eigen::MatrixXd, Eigen::MatrixXd> HP, Polygon V)
{
    // compute difference distribution
    Eigen::Vector2d mu_ab = belief_a.first - belief_b.first;
    Eigen::Matrix2d sigma_ab = belief_a.second + belief_b.second;

    // perform whitten transform on sigma_ab
    Eigen::EigenSolver<Eigen::Matrix2d> eigensolver;
    eigensolver.compute(sigma_ab);
    Eigen::Matrix2d eigen_vals;
    eigen_vals.diagonal() << eigensolver.eigenvalues().real();
    Eigen::Matrix2d eigen_vecs = eigensolver.eigenvectors().real();

    Eigen::LLT<Eigen::Matrix2d> lltOfVal(eigen_vals); // compute the Cholesky decomposition of A
    Eigen::MatrixXd L = lltOfVal.matrixL(); // retrieve factor L  in the decomposition
    // The previous two lines can also be written as "L = A.llt().matrixL()"

    Eigen::Matrix2d R = eigen_vecs * L.inverse();
    Eigen::Vector2d T = mu_ab;

    Eigen::MatrixXd A = HP.first;
    Eigen::MatrixXd B = HP.second;
    Eigen::MatrixXd A_new(A.rows(), A.cols());
    Eigen::MatrixXd B_new(B.rows(), B.cols());
    for (int i = 0; i != A.rows(); i++) {
        auto tmp = A.row(i) * (R.transpose()).inverse();
        A_new(i, 0) = tmp(0, 0);
        A_new(i, 1) = tmp(0, 1);
        B_new(i, 0) = (B.row(i)-A.row(i)*T).value();
    }

    // convert Polygon V to matrix of vertices
    auto P_pts = boost::geometry::exterior_ring(V);
    Eigen::MatrixXd V_mat(2, P_pts.size()-1);
    for (int i = 0; i != P_pts.size() - 1; i++) {
        V_mat(0, i) = bg::get<0>(P_pts[i]);
        V_mat(1, i) = bg::get<1>(P_pts[i]);
    }
    
    Eigen::MatrixXd V_new = R.transpose() * ((-V_mat).colwise() + T);

    // std::cout << V_new << std::endl;

    Polygon V_new_poly;
    // construct the rectangular polygon w/ ref in the center
    for (int c = 0; c != V_new.cols(); c++) {
        Point pt(V_new(0, c), V_new(1, c));
        bg::append(V_new_poly.outer(), pt);
    }
    bg::correct(V_new_poly);

    // for (auto &pt: boost::geometry::exterior_ring(V_new_poly)) {
    //     std::cout << bg::get<0>(pt) << "," << bg::get<1>(pt) << std::endl;
    // }

    // Grid CDF
    const double x_low = V_new.row(0).minCoeff();
    const double x_high = V_new.row(0).maxCoeff();
    const double y_low = V_new.row(1).minCoeff();
    const double y_high = V_new.row(1).maxCoeff();

    std::vector<double> Xgrid = linspace_(x_low, x_high, disk_steps_);
    std::vector<double> Ygrid = linspace_(y_low, y_high, disk_steps_);

    double prob = 0;
    for (int ix = 0; ix != disk_steps_ - 1; ix++) {
        for (int iy = 0; iy != disk_steps_ - 1; iy++) {
            const double x1 = Xgrid[ix];
            const double x2 = Xgrid[ix + 1];
            const double y1 = Ygrid[iy];
            const double y2 = Ygrid[iy + 1];

            Point pt1(x1, y1);
            Point pt2(x2, y1);
            Point pt3(x2, y2);
            Point pt4(x1, y2);

            const std::vector<Point> B{pt1, pt2, pt3, pt4};

            for (auto &pt: B) {
                if (!bg::disjoint(pt, V_new_poly)) {
                    prob += cdfRect_(B);
                    break;
                }
            }
        }
    }
    if (prob < p_coll_agnts_)
        return true;
    return false;
}

double CDFGridPVC::cdfRect_(std::vector<Point> B)
{
    Point v1 = B[0];
    Point v2 = B[1];
    Point v3 = B[2];
    Point v4 = B[3];

    const double p1 = (1.0/2.0) * (1 + bm::erf(bg::get<0>(v1) / sqrt(2))) * (1.0/2.0) * (1 + bm::erf(bg::get<1>(v1) / sqrt(2)));
    const double p2 = (1.0/2.0) * (1 + bm::erf(bg::get<0>(v2) / sqrt(2))) * (1.0/2.0) * (1 + bm::erf(bg::get<1>(v2) / sqrt(2)));
    const double p3 = (1.0/2.0) * (1 + bm::erf(bg::get<0>(v3) / sqrt(2))) * (1.0/2.0) * (1 + bm::erf(bg::get<1>(v3) / sqrt(2)));
    const double p4 = (1.0/2.0) * (1 + bm::erf(bg::get<0>(v4) / sqrt(2))) * (1.0/2.0) * (1 + bm::erf(bg::get<1>(v4) / sqrt(2)));

    return p3 - p2 - p4 + p1;
}

std::vector<double> CDFGridPVC::linspace_(double start, double end, int num)
{
    std::vector<double> linspaced;

    if (num == 0) { return linspaced; }
    if (num == 1) 
    {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i)
    {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
    return linspaced;
}

Polygon CDFGridPVC::getBoundingBox_(const double r_1, const double r_2)
{
    /* Create Bounding Box of both radii */
    const double r = r_1 + r_2;

    /* construct the rectangular polygon w/ ref in the center */
    Polygon integration_box;
    Point bott_left(-r, -r);
    Point bott_right(r, -r);
    Point top_right(r, r);
    Point top_left(-r, r);

    bg::append(integration_box.outer(), bott_left);
    bg::append(integration_box.outer(), bott_right);
    bg::append(integration_box.outer(), top_right);
    bg::append(integration_box.outer(), top_left);
    bg::append(integration_box.outer(), bott_left);

    // assures that the polygon (1) has counter-clockwise points, and (2) is closed
    bg::correct(integration_box);
    return integration_box;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> CDFGridPVC::getHalfPlanes_(Polygon combined_poly)
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
