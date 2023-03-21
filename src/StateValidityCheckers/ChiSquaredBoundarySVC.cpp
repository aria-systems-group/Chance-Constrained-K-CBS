#include "StateValidityCheckers/ChiSquaredBoundarySVC.h"

ChiSquaredBoundarySVC::ChiSquaredBoundarySVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, const double accep_prob) :
    ob::StateValidityChecker(si), si_(si.get()), mrmp_instance_(mrmp_instance), robot_(r), max_rad_(robot_->getBoundingRadius())
{
    // set sc_ value
    sc_ = chi_squared_quantile_(2, accep_prob);

    // get obstacle objects (take minkowski sum with obstacle and robot bounding shape)
    // obs_list_(mrmp_instance_->getObstacles())
    for (auto o: mrmp_instance_->getObstacles()) {
        Polygon new_obs_poly = getMinkowskiSum_(robot_, o);
        obs_list_.push_back(new_obs_poly);
    }
}

ChiSquaredBoundarySVC::~ChiSquaredBoundarySVC(){};

bool ChiSquaredBoundarySVC::isValid(const ob::State *state) const
{
    if (!si_->satisfiesBounds(state)) {
        return false;
    }

    if (obs_list_.empty())
        return true;

    /* get Belief from state */
    double* vals = state->as<RealVectorBeliefSpace::StateType>()->values;
    Eigen::Vector2d mu(2);
    for (int d = 0; d < 2; d++) {
        mu[d] = vals[d];
    }

    Eigen::Matrix2d Sigma = state->as<RealVectorBeliefSpace::StateType>()->getCovariance().block<2, 2>(0, 0);

    /* Find maximum eigenvalues of the covariances */
    Eigen::EigenSolver<Eigen::Matrix2d> eigensolver_a;
    eigensolver_a.compute(Sigma);

    const double max_lambda = eigensolver_a.eigenvalues().real().maxCoeff();
    const double boundary = (max_lambda * sc_ + max_rad_);

    bg::strategy::buffer::distance_symmetric<double> distance_strategy(boundary);

    Point center = Point(mu[0], mu[1]);
    bg::model::multi_polygon<Polygon> tmp;  // buffer generates multipolygons
    Polygon disk;

    // make disk centered on `center` and of correct `radius`
    bg::buffer(center, tmp, distance_strategy, side_strategy,
               join_strategy, end_strategy, circle_strategy);

    // convert the MultiPolygon output to a simple polygon
    disk = Polygon(tmp[0]);

    for (auto &obs_poly: obs_list_) {
        if (!bg::disjoint(disk, obs_poly))
            return false;
    }
    return true;
}

Polygon ChiSquaredBoundarySVC::getMinkowskiSum_(const Robot* r, Obstacle* &obs)
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

double ChiSquaredBoundarySVC::crossProduct_(const Point &a, const Point &b)
{
    /* Performs A \times B */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    return (ax * by) - (ay * bx);
}

Point ChiSquaredBoundarySVC::subtractPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax-bx, ay-by);
    return difference;
}

Point ChiSquaredBoundarySVC::addPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax+bx, ay+by);
    return difference;
}
