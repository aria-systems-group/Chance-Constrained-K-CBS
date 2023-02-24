#include "StateValidityCheckers/CentralizedChiSquaredBoundarySVC.h"

CentralizedChiSquaredBoundarySVC::CentralizedChiSquaredBoundarySVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const double accep_prob) :
    ob::StateValidityChecker(si), si_(si.get()), mrmp_instance_(mrmp_instance),
    num_agents_(mrmp_instance->getRobots().size())
{
    // set sc_ value
    sc_ = chi_squared_quantile_(2, accep_prob);

    // create obstacles map
    for (int a = 0; a < num_agents_; a++) {
        Robot* r = mrmp_instance->getRobots()[a];
        boundingRadii_map_[a] = r->getBoundingRadius();
        obs_map_[a] = {};
        for (auto o: mrmp_instance_->getObstacles()) {
            Polygon new_obs_poly = getMinkowskiSum_(r, o);
            obs_map_[a].push_back(new_obs_poly);
        }
    }
}

CentralizedChiSquaredBoundarySVC::~CentralizedChiSquaredBoundarySVC(){};

bool CentralizedChiSquaredBoundarySVC::isValid(const ob::State *state) const
{
    if (!si_->satisfiesBounds(state)) {
        return false;
    }

    const double* all_st_values = state->as<RealVectorBeliefSpace::StateType>()->values;
    const Eigen::MatrixXd all_sigmas = state->as<RealVectorBeliefSpace::StateType>()->sigma_;
    const Eigen::MatrixXd all_lambdas = state->as<RealVectorBeliefSpace::StateType>()->lambda_;

    for (auto a1 = 0; a1 < num_agents_; a1++) {
        // check a1 against all obstacles
        /* get Belief from state */
        Eigen::Vector2d mu_a1{all_st_values[2 * a1], all_st_values[2 * a1 + 1]};
        Eigen::Matrix2d sigma_a1 = all_sigmas.block<2, 2>(2 * a1, 2 * a1);

        /* Find maximum eigenvalues of the covariances */
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver_a1(sigma_a1.rows());
        solver_a1.compute(sigma_a1);

        const double max_lambda_a1 = solver_a1.eigenvalues().maxCoeff();
        const double max_rad_a1 = boundingRadii_map_.at(a1);
        const double boundary_a1 = (max_lambda_a1 * sc_ + max_rad_a1);

        bg::strategy::buffer::distance_symmetric<double> distance_strategy_a1(boundary_a1);

        Point center_a1 = Point(mu_a1[0], mu_a1[1]);
        bg::model::multi_polygon<Polygon> tmp_a1;  // buffer generates multipolygons
        Polygon disk_a1;

        // make disk centered on `center` and of correct `radius`
        bg::buffer(center_a1, tmp_a1, distance_strategy_a1, side_strategy,
               join_strategy, end_strategy, circle_strategy);

        // convert the MultiPolygon output to a simple polygon
        disk_a1 = Polygon(tmp_a1[0]);

        std::vector<Polygon> obs_list_a1 = obs_map_.at(a1);
        for (auto &obs_poly: obs_list_a1) {
            if (!bg::disjoint(disk_a1, obs_poly))
                return false;
        }

        for (auto a2 = a1 + 1; a2 < num_agents_; a2++) {
            // check a1 against a2
            /* get Belief from state */
            Eigen::Vector2d mu_a2{all_st_values[2 * a2], all_st_values[2 * a2 + 1]};
            Eigen::Matrix2d sigma_a2 = all_sigmas.block<2, 2>(2 * a2, 2 * a2);

            /* Find maximum eigenvalues of the covariances */
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver_a2(sigma_a2.rows());
            solver_a1.compute(sigma_a2);

            const double max_lambda_a2 = solver_a2.eigenvalues().maxCoeff();
            const double max_rad_a2 = boundingRadii_map_.at(a2);
            const double boundary_a2 = (max_lambda_a2 * sc_ + max_rad_a2);

            bg::strategy::buffer::distance_symmetric<double> distance_strategy_a2(boundary_a2);

            Point center_a2 = Point(mu_a2[0], mu_a2[1]);
            bg::model::multi_polygon<Polygon> tmp_a2;  // buffer generates multipolygons
            Polygon disk_a2;

            // make disk centered on `center` and of correct `radius`
            bg::buffer(center_a2, tmp_a2, distance_strategy_a2, side_strategy,
                    join_strategy, end_strategy, circle_strategy);

            // convert the MultiPolygon output to a simple polygon
            disk_a2 = Polygon(tmp_a2[0]);

            if (!bg::disjoint(disk_a1, disk_a2))
                return false;
        }
    }
    return true;
}

Polygon CentralizedChiSquaredBoundarySVC::getMinkowskiSum_(const Robot* r, Obstacle* &obs)
{
    /* Next, create copy of obstacle polygon */
    Polygon obs_poly = obs->getPolygon();
    bg::correct(obs_poly);

    /* Perform the Minkowski Sum */
    std::vector<Point> shape1_points = boost::geometry::exterior_ring(r->getBoundingShape());
    std::vector<Point> shape2_points = boost::geometry::exterior_ring(obs_poly);

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

double CentralizedChiSquaredBoundarySVC::crossProduct_(const Point &a, const Point &b)
{
    /* Performs A \times B */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    return (ax * by) - (ay * bx);
}

Point CentralizedChiSquaredBoundarySVC::subtractPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax-bx, ay-by);
    return difference;
}

Point CentralizedChiSquaredBoundarySVC::addPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax+bx, ay+by);
    return difference;
}
