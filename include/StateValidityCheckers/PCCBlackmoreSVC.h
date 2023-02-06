#pragma once
#include "Instance.h"
#include "Spaces/R2BeliefSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <eigen3/Eigen/Dense>

namespace ob = ompl::base;
namespace oc = ompl::control;


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
        int n_obstacles_ = -1;

		std::vector<Eigen::Matrix<double, 4, 2> > A_list_;
		std::vector<Eigen::Matrix<double, 4, 1> > B_list_;
        Eigen::Matrix2d PX;

		inline double computeInverseErrorFunction_(const double &argument) {
			return boost::math::erf_inv(argument);
		}
		bool HyperplaneCCValidityChecker_(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const double &x_pose, const double &y_pose, const Eigen::MatrixXd &PX) const;
};

