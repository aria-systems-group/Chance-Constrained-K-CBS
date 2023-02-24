#pragma once
#include "utils/Instance.h"
#include "Spaces/R2BeliefSpace.h"
#include <ompl/control/SpaceInformation.h>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace bm = boost::math;


class AdaptiveRiskBlackmoreSVC : public ob::StateValidityChecker {
    public:
        AdaptiveRiskBlackmoreSVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, const double accep_prob);
        ~AdaptiveRiskBlackmoreSVC();

        virtual bool isValid(const ob::State *state) const;

    private:
        const ob::SpaceInformation *si_;
        InstancePtr mrmp_instance_;
        const Robot *robot_; 
        double p_collision_;
        std::vector<Obstacle*> obs_list_;

        std::vector<Eigen::Matrix<double, 4, 2> > A_list_;
        std::vector<Eigen::Matrix<double, 4, 1> > B_list_;
        Eigen::Matrix2d PX;

        bool isSafe_(const Eigen::Vector2d mu_a, const Eigen::Matrix2d Sigma_a, const Eigen::MatrixXd A, const Eigen::MatrixXd B, const double eta_i) const;
        std::vector<double> createEtaList_(const Eigen::Vector2d mu_a) const;
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getHalfPlanes_(Polygon combined_poly);
        Polygon getMinkowskiSum_(const Robot* &r, Obstacle* &obs);
        Point addPoints_(const Point &a, const Point &b);
        Point subtractPoints_(const Point &a, const Point &b);
        double crossProduct_(const Point &a, const Point &b);
};