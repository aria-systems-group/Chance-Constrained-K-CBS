#include "StateValidityCheckers/PCCBlackmoreSVC.h"

PCCBlackmoreSVC::PCCBlackmoreSVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, const double accep_prob) :
	ob::StateValidityChecker(si), si_(si.get()), mrmp_instance_(mrmp_instance), robot_(r) {
	p_collision_ = 1 - accep_prob;

	std::vector<Obstacle*> obs_list = mrmp_instance_->getObstacles();
	n_obstacles_ = obs_list.size();
	erf_inv_result_ = computeInverseErrorFunction_(1 - 2 * p_collision_ / n_obstacles_);

	// A_list_.resize(n_obstacles_); 
   
    
    for (auto itr = obs_list.begin(); itr != obs_list.end(); itr++) {
        auto exterior_points = (*itr)->getPolyPoints();
		Eigen::MatrixXd A(4, 2);
        Eigen::MatrixXd B(4, 1);
        for (int i=0; i<exterior_points.size()-1; i++)
        {
            double x1 = boost::geometry::get<0>(exterior_points[i]);
            double y1 = boost::geometry::get<1>(exterior_points[i]);
            double x2 = boost::geometry::get<0>(exterior_points[i+1]);
            double y2 = boost::geometry::get<1>(exterior_points[i+1]);

            double a = y2 - y1;
            double b = x1 - x2;
            double c = a * x1 + b * y1;

            A(i, 0) = a;
            A(i, 1) = b;
            
            B(i,0) = c;
        }
        A_list_.push_back(A);
        B_list_.push_back(B);
    }
}

PCCBlackmoreSVC::~PCCBlackmoreSVC() {}

bool PCCBlackmoreSVC::isValid(const ob::State *state) const {
    
	//=========================================================================
	// Bounds checker
	//=========================================================================
	if (!si_->satisfiesBounds(state)) {
		return false;
	}

    
    // if (x > 100.0 || x < 0.0 || y < 0.0 || y > 100.0){
    //     return false;
    // }
	double x = state->as<R2BeliefSpace::StateType>()->getX();
    double y = state->as<R2BeliefSpace::StateType>()->getY();
    auto PX = state->as<R2BeliefSpace::StateType>()->getCovariance();
	//=========================================================================
	// Probabilistic collision checker
	//=========================================================================
	for (int o = 0; o < n_obstacles_; o++) {
		if (not HyperplaneCCValidityChecker_(A_list_.at(o), B_list_.at(o), x, y, PX)) {
			return false;
		}
	}
	return true;
}

bool PCCBlackmoreSVC::HyperplaneCCValidityChecker_(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const double &x_pose, const double &y_pose, const Eigen::MatrixXd &PX) const {
	double PV, b_bar;
	Eigen::MatrixXf Pv_2;

	for (int i = 0; i < 4; i++) {
        double tmp = (A.row(i).transpose() * PX * A.row(i)).value();
		PV = sqrt(tmp);
		b_bar = sqrt(2) * PV * erf_inv_result_;
		if(x_pose * A(i, 0) + y_pose * A(i, 1) >= (B(i, 0) + b_bar)) {
            return true;
		}
	}
	return false;
}
