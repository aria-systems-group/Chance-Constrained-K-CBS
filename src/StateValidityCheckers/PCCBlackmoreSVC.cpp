#include "StateValidityCheckers/PCCBlackmoreSVC.h"

PCCBlackmoreSVC::PCCBlackmoreSVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, const double accep_prob) :
	ob::StateValidityChecker(si), si_(si.get()), mrmp_instance_(mrmp_instance), robot_(r) {
	p_collision_ = 1 - accep_prob;

	std::vector<Obstacle*> obs_list = mrmp_instance_->getObstacles();
	n_obstacles_ = obs_list.size();
	erf_inv_result_ = computeInverseErrorFunction(1 - 2 * p_collision_ / n_obstacles_);

	A_list_.resize(n_obstacles_); 
	
	B_list_.resize(n_obstacles_); 

	// A_list_ = ; #TODO: this needs to take in the polygons from obstacles and convert them to A and B matrices of the form Ax <= B
	// B_list_ = ;
}

PCCBlackmoreSVC::~PCCBlackmoreSVC() {}

bool PCCBlackmoreSVC::isValid(const ob::State *state) const {
    
	//=========================================================================
	// Bounds checker
	//=========================================================================
	if (!si_->satisfiesBounds(state)) {
		return false;
	}

    // const double x = state->as<R2BeliefSpace::StateType>()->getX();
    // const double y = state->as<R2BeliefSpace::StateType>()->getY();
    // if (x > 100.0 || x < 0.0 || y < 0.0 || y > 100.0){
    //     return false;
    // }

	//=========================================================================
	// Extract the component of the state and cast it to what we expect
	//=========================================================================
	// double x_pose, y_pose;
	// Eigen::MatrixXf PX(3, 3); PX.setZero();

	// x_pose = state->as<R2BeliefSpace::StateType>()->getX();
	// y_pose = state->as<R2BeliefSpace::StateType>()->getY();
	// z_pose = 4.0;
    // PX(0,0) = state->as<R2BeliefSpace::StateType>()->getCovariance()(0,0);
    // PX(1,1) = state->as<R2BeliefSpace::StateType>()->getCovariance()(1,1);
	// // PX(0,0) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[0];
	// // PX(1,1) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[1];
	// // PX(2,2) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[2];

	//=========================================================================
	// Probabilistic collision checker
	//=========================================================================
	bool valid = true;
	for (int o = 0; o < n_obstacles_; o++) {
		if (not HyperplaneCCValidityChecker(A_list_.at(o), B_list_.at(o), x_pose, y_pose, z_pose, PX)) {
			goto exit_switch;
		}
	}

	exit_switch:;
	return valid;
}

bool PCCBlackmoreSVC::HyperplaneCCValidityChecker_(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const {
	
    bool valid = false;
	double PV, b_bar;
	Eigen::MatrixXf Pv_2;

	for (int i = 0; i < 6; i++) {
		Pv_2 = A.row(i) * PX * A.row(i).transpose();
		PV = sqrt(Pv_2(0, 0));

		b_bar = sqrt(2) * PV * erf_inv_result_;

		if(x_pose * A(i, 0) + y_pose * A(i, 1) + z_pose * A(i, 2) >= (B(i, 0) + b_bar)) {
			valid = true;
			break;
		}
	}
	return valid;
}
