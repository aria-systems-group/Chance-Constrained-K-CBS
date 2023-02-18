#include "StateValidityCheckers/AdaptiveRiskBlackmoreSVC.h"

AdaptiveRiskBlackmoreSVC::AdaptiveRiskBlackmoreSVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, const double accep_prob) :
    ob::StateValidityChecker(si), si_(si.get()), mrmp_instance_(mrmp_instance), robot_(r), obs_list_(mrmp_instance_->getObstacles()) 
{
    p_collision_ = 1 - accep_prob;

    /*  Generate half-planes for robot to avoid obstacles.
    //  This is done by iterating through all the obstacles, taking the Minkoski sum with robot bounding box, 
    //  and then creating the half-planes for that.
    */ 

    for (auto itr = obs_list_.begin(); itr != obs_list_.end(); itr++) {
        // take Minkoski sum of obstacle and robot
        Polygon result_poly = getMinkowskiSum_(r, *itr);
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> half_plane_matrices = getHalfPlanes_(result_poly);
        A_list_.push_back(half_plane_matrices.first);
        B_list_.push_back(half_plane_matrices.second);
    }
}

AdaptiveRiskBlackmoreSVC::~AdaptiveRiskBlackmoreSVC() {}

bool AdaptiveRiskBlackmoreSVC::isValid(const ob::State *state) const 
{
    
    //=========================================================================
    // Bounds checker
    //=========================================================================
    if (!si_->satisfiesBounds(state)) {
        return false;
    }

    // double x = state->as<R2BeliefSpace::StateType>()->getX();
    // double y = state->as<R2BeliefSpace::StateType>()->getY();

    const Eigen::Vector2d mu = state->as<R2BeliefSpace::StateType>()->getXY();
    const Eigen::Matrix2d Sigma = state->as<R2BeliefSpace::StateType>()->getCovariance();
    //=========================================================================
    // Probabilistic collision checker
    //=========================================================================
    // calculate the the eta_i's for agent *itr_a
    std::vector<double> eta_list = createEtaList_(mu);

    for (int o = 0; o < obs_list_.size(); o++) {
        if (!isSafe_(mu, Sigma, A_list_.at(o), B_list_.at(o), eta_list[0])) {
            return false;
        }
    }
    return true;
}

bool AdaptiveRiskBlackmoreSVC::isSafe_(const Eigen::Vector2d mu_a, const Eigen::Matrix2d Sigma_a, const Eigen::MatrixXd A, const Eigen::MatrixXd B, const double eta_i) const
{
    auto const n_rows = A.rows();
    for (int i = 0; i < n_rows; i++) {
        const double tmp = (A.row(i) * Sigma_a * A.row(i).transpose()).value();
        const double Pv = sqrt(tmp);
        const double vbar = sqrt(2) * Pv * bm::erf_inv(1 - (2 * eta_i));
        if( (A(i, 0) * mu_a[0] + A(i, 1) * mu_a[1] - B(i, 0) >= vbar) ) {
            return true;
        };
    }
    return false;
}

std::vector<double> AdaptiveRiskBlackmoreSVC::createEtaList_(const Eigen::Vector2d mu_a) const
{
    std::vector<double> di_list;
    double sum = 0;
    int idx = 0;
    for (Obstacle* o: obs_list_) {
        const double mu_diff_x = mu_a(0, 0) - o->x_;
        const double mu_diff_y = mu_a(1, 0) - o->y_;
        const double di =  sqrt( mu_diff_x*mu_diff_x + mu_diff_y*mu_diff_y );
        // di_map[idx] = di;
        di_list.push_back(di);
        sum += (1 / di);
    }

    const double alpha = 1 / sum;

    /* Calculate all the eta_i and return */
    std::vector<double> eta_list;
    for (auto di_itr = di_list.begin(); di_itr != di_list.end(); di_itr++) {
        const double eta_i = (alpha / (*di_itr)) * p_collision_;
        eta_list.push_back(eta_i); 
    }
    return eta_list;
}

bool AdaptiveRiskBlackmoreSVC::HyperplaneCCValidityChecker_(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const double &x_pose, const double &y_pose, const Eigen::MatrixXd &PX, const double eta_i) const {
    double PV, b_bar;
    Eigen::MatrixXf Pv_2;

    for (int i = 0; i < 4; i++) {
        double tmp = (A.row(i) * PX * A.row(i).transpose()).value();
        PV = sqrt(tmp);
        b_bar = sqrt(2) * PV * erf_inv_result_;
        if(x_pose * A(i, 0) + y_pose * A(i, 1) >= (B(i, 0) + b_bar)) {
            return true;
        }
    }
    return false;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> AdaptiveRiskBlackmoreSVC::getHalfPlanes_(Polygon combined_poly)
{
    /* Converts a Polygon to a set of halfplanes */
    auto exterior_points = bg::exterior_ring(combined_poly);
    if (exterior_points.size() != 5) {
        OMPL_WARN("%s: Generating Halfplanes currently supports closed rectangles. Please check the implementation.", "AdaptiveRiskBlackmoreSVC");
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

Polygon AdaptiveRiskBlackmoreSVC::getMinkowskiSum_(const Robot* &r, Obstacle* &obs)
{
    // test: triangle and rectangle from web (centered at origin!)
    // https://cp-algorithms.com/geometry/minkowski.html#algorithm
    // Polygon shape1{{{0, -0.5}, {0.5, 0.5}, {-0.5, 0.5}}}; 
    // Polygon shape2{{{-1, -1}, {1, -1}, {1, 1}, {-1,1}}};

    /* First, center robot at obstacle location */
    Polygon r_initial_poly;
    Polygon r_origin_centered;

    const double r_ix = r->getStartLocation().x_;
    const double r_iy = r->getStartLocation().y_;
    // const double o_x = obs->x_;
    // const double o_y = obs->y_;
    bg::correct(r_initial_poly);
    bg::assign(r_initial_poly, r->getShape());
    bg::strategy::transform::matrix_transformer<double, 2, 2> r_xfrm(
             cos(0), sin(0), (0 - r_ix),
            -sin(0), cos(0), (0 - r_iy),
                      0,          0,  1);
    bg::transform(r_initial_poly, r_origin_centered, r_xfrm);
    bg::correct(r_origin_centered);

    // for (const auto &a : boost::geometry::exterior_ring(r_origin_centered)) {
    //  const double x = bg::get<0>(a);
    //  const double y = bg::get<1>(a);
    //  std::cout << x << "," << y << std::endl;
    // }

    /* Next, create copy of obstacle polygon */
    Polygon obs_poly = obs->getPolygon();
    bg::correct(obs_poly);

    /* Perform the Minkowski Sum */
    std::vector<Point> shape1_points = boost::geometry::exterior_ring(r_origin_centered);
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

double AdaptiveRiskBlackmoreSVC::crossProduct_(const Point &a, const Point &b)
{
    /* Performs A \times B */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    return (ax * by) - (ay * bx);
}

Point AdaptiveRiskBlackmoreSVC::subtractPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax-bx, ay-by);
    return difference;
}

Point AdaptiveRiskBlackmoreSVC::addPoints_(const Point &a, const Point &b)
{
    /* Element-wise subtracts B from A ( i.e. A-B ) */
    const double ax = bg::get<0>(a);
    const double ay = bg::get<1>(a);
    const double bx = bg::get<0>(b);
    const double by = bg::get<1>(b);

    Point difference(ax+bx, ay+by);
    return difference;
}