#pragma once
#include "Instance.h"
#include "Spaces/R2BeliefSpace.h"
#include <ompl/control/SpaceInformation.h>
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <Eigen/Eigenvalues> 

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace bm = boost::math;


class ChiSquaredBoundarySVC : public ob::StateValidityChecker {
    public:
        ChiSquaredBoundarySVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, const double accep_prob);
        ~ChiSquaredBoundarySVC();

        virtual bool isValid(const ob::State *state) const;

    private:
        double chi_squared_quantile_(double v, double p)
        {
           return quantile(bm::chi_squared(v), p);
        }
        Polygon getMinkowskiSum_(const Robot* r, Obstacle* &obs);
        Point addPoints_(const Point &a, const Point &b);
        Point subtractPoints_(const Point &a, const Point &b);
        double crossProduct_(const Point &a, const Point &b);
        const ob::SpaceInformation *si_;
        InstancePtr mrmp_instance_;
        const Robot *robot_; 
        std::vector<Polygon> obs_list_;
        const double max_rad_;
        double sc_;
        const std::size_t points_per_circle_ = 10;
        bg::strategy::buffer::join_round join_strategy{points_per_circle_};
        boost::geometry::strategy::buffer::end_flat end_strategy;
        bg::strategy::buffer::point_circle circle_strategy{points_per_circle_};
        bg::strategy::buffer::side_straight side_strategy;

        // std::vector<Eigen::Matrix<double, 4, 2> > A_list_;
        // std::vector<Eigen::Matrix<double, 4, 1> > B_list_;
        // Eigen::Matrix2d PX;

        // inline double computeInverseErrorFunction_(const double &argument) {
        //     return boost::math::erf_inv(argument);
        // }
        // bool HyperplaneCCValidityChecker_(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const double &x_pose, const double &y_pose, const Eigen::MatrixXd &PX) const;
        // std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getHalfPlanes_(Polygon combined_poly);
        // Polygon getMinkowskiSum_(const Robot* r, Obstacle* &obs);
        // Point addPoints_(const Point &a, const Point &b);
        // Point subtractPoints_(const Point &a, const Point &b);
        // double crossProduct_(const Point &a, const Point &b);
};