#pragma once
#include "utils/Instance.h"
#include "Spaces/RealVectorBeliefSpace.h"
#include <ompl/control/SpaceInformation.h>
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <Eigen/Eigenvalues> 

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace bm = boost::math;


class CentralizedChiSquaredBoundarySVC : public ob::StateValidityChecker {
    public:
        CentralizedChiSquaredBoundarySVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const double accep_prob);
        ~CentralizedChiSquaredBoundarySVC();

        bool isValid(const ob::State *state) const override;

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
        double sc_;
        const int num_agents_;
        std::unordered_map<int, std::vector<Polygon>> obs_map_;
        std::unordered_map<int, double> boundingRadii_map_;
        const std::size_t points_per_circle_ = 10;
        bg::strategy::buffer::join_round join_strategy{points_per_circle_};
        boost::geometry::strategy::buffer::end_flat end_strategy;
        bg::strategy::buffer::point_circle circle_strategy{points_per_circle_};
        bg::strategy::buffer::side_straight side_strategy;
};