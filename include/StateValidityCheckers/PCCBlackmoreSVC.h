#pragma once
#include "common.h"
#include "Instance.h"
#include "Spaces/R2BeliefSpace.h"
// #include <boost/math/special_functions/erf.hpp>
#include <eigen3/Eigen/Dense>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>


OMPL_CLASS_FORWARD(Instance);
class PCCBlackmoreSVC : public ob::StateValidityChecker {
	public:
		PCCBlackmoreSVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, const double accep_prob);
		~PCCBlackmoreSVC();

		virtual bool isValid(const ob::State *state) const;

	private:
		const ob::SpaceInformation *si_;
    	InstancePtr mrmp_instance_;
    	const Robot *robot_; 
		double p_collision_;
		double erf_inv_result_;

		std::vector<Eigen::Matrix<float, 6, 3> > A_list_;
		std::vector<Eigen::Matrix<float, 6, 1> > B_list_;

		// inline double computeInverseErrorFunction_(const double &argument) {
		// 	return boost::math::erf_inv(argument);
		// }
		bool HyperplaneCCValidityChecker_(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const;
};

