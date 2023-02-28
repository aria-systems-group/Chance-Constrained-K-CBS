#include "StateValidityCheckers/PCCBlackmoreSVC.h"

PCCBlackmoreSVC::PCCBlackmoreSVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, const double accep_prob) :
	ob::StateValidityChecker(si), si_(si.get()), mrmp_instance_(mrmp_instance), robot_(r) {
	p_collision_ = 1 - accep_prob;

	std::vector<Obstacle*> obs_list = mrmp_instance_->getObstacles();
	n_obstacles_ = obs_list.size();
	erf_inv_result_ = computeInverseErrorFunction_(1 - 2 * (p_collision_ / n_obstacles_));


	/* 	Generate half-planes for robot to avoid obstacles.
	//	This is done by iterating through all the obstacles, taking the Minkoski sum with robot bounding box, 
	//	and then creating the half-planes for that.
	*/ 

	for (auto itr = obs_list.begin(); itr != obs_list.end(); itr++) {
		// take Minkoski sum of obstacle and robot
		Polygon result_poly = getMinkowskiSum_(r, *itr);
		std::pair<Eigen::MatrixXd, Eigen::MatrixXd> half_plane_matrices = getHalfPlanes_(result_poly);
		A_list_.push_back(half_plane_matrices.first);
		B_list_.push_back(half_plane_matrices.second);
	}
}

PCCBlackmoreSVC::~PCCBlackmoreSVC() {}

bool PCCBlackmoreSVC::isValid(const ob::State *state) const 
{
    // std::cout << "isValid" << std::endl;
	//=========================================================================
	// Bounds checker
	//=========================================================================
	if (!si_->satisfiesBounds(state)) {
		return false;
	}
    Eigen::Vector2d mu = Eigen::Vector2d::Zero();
    Eigen::Matrix2d Sigma;
    double* vals = state->as<RealVectorBeliefSpace::StateType>()->values;
    Sigma = state->as<RealVectorBeliefSpace::StateType>()->getCovariance().block<2, 2>(0, 0);
    // project mean and covariance onto xy-plane
    for (int d = 0; d < 2; d++) {
        mu[d] = vals[d];
    }
    

    // std::cout << mu << std::endl;
    // std::cout << Sigma << std::endl;
    
	//=========================================================================
	// Probabilistic collision checker
	//=========================================================================
	for (int o = 0; o < n_obstacles_; o++) {
		if (not HyperplaneCCValidityChecker_(A_list_.at(o), B_list_.at(o), mu[0], mu[1], Sigma)) {
			return false;
		}
	}
    // std::cout << "returning valid" << std::endl;
	return true;
}

bool PCCBlackmoreSVC::HyperplaneCCValidityChecker_(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const double &x_pose, const double &y_pose, const Eigen::Matrix2d &PX) const {
	double PV, b_bar;

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

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> PCCBlackmoreSVC::getHalfPlanes_(Polygon combined_poly)
{
	/* Converts a Polygon to a set of halfplanes */
    // for (const auto &pt : boost::geometry::exterior_ring(combined_poly)) {
    //     const double x = bg::get<0>(pt);
    //     const double y = bg::get<1>(pt);
    //     std::cout << x << "," << y << std::endl;
    // }
	auto exterior_points = bg::exterior_ring(combined_poly);
	if (exterior_points.size() != 5) {
		OMPL_WARN("%s: Generating Halfplanes currently supports closed rectangles. Please check the implementation.", "PCCBlackmoreSVC");
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

Polygon PCCBlackmoreSVC::getMinkowskiSum_(const Robot* r, Obstacle* &obs)
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
        bg::append(result_poly.outer(), *itr);
    }
    bg::correct(result_poly);
    return result_poly;
}

double PCCBlackmoreSVC::crossProduct_(const Point &a, const Point &b)
{
	/* Performs A \times B */
	const double ax = bg::get<0>(a);
	const double ay = bg::get<1>(a);
	const double bx = bg::get<0>(b);
	const double by = bg::get<1>(b);

	return (ax * by) - (ay * bx);
}

Point PCCBlackmoreSVC::subtractPoints_(const Point &a, const Point &b)
{
	/* Element-wise subtracts B from A ( i.e. A-B ) */
	const double ax = bg::get<0>(a);
	const double ay = bg::get<1>(a);
	const double bx = bg::get<0>(b);
	const double by = bg::get<1>(b);

	Point difference(ax-bx, ay-by);
	return difference;
}

Point PCCBlackmoreSVC::addPoints_(const Point &a, const Point &b)
{
	/* Element-wise subtracts B from A ( i.e. A-B ) */
	const double ax = bg::get<0>(a);
	const double ay = bg::get<1>(a);
	const double bx = bg::get<0>(b);
	const double by = bg::get<1>(b);

	Point difference(ax+bx, ay+by);
	return difference;
}
